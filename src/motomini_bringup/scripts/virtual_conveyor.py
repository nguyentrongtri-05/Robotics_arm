#!/usr/bin/env python3
"""
Mo phong Toan Dien Bai Thi Nghiem 2, 3, 4 (Bo mon Dieu khien Tu Dong - DHBK TPHCM)
Nang cap cho He Thong 2 CANH TAY MOTOMINI (R1 & R2) + 2 BANG TAI (CONV1 & CONV2) + MAM XOAY TRUC A:

1. Bai Thi Nghiem 2 (Giao tiep Robot va Machine Controller MP3300 qua I/O & Ethernet):
   - Bang 1 Ket noi I/O cua Machine Controller MP3300:
     + DI_00 (START), DI_01 (STOP), DI_02 (Servo_ON), DI_03 (Robot_ON / Reset Alarm)
     + DI_10 (Sensor HOME_A mam xoay), DI_11 (Sensor Bang tai 1 & 2), DI_12 (Tin hieu xung OT#3 tu Robot)
     + DI_24..31 (Switches I0..I7 tren I/O Box)
     + DO_00 (Inverter S1 - Chay Bang tai 1 & 2), DO_01 (Robot IN#1), DO_02 (Robot IN#2 - Bao co vat tren mam xoay)
     + DO_16..23 / DO_24..31 (Den Q0..Q7)
   - Chuong trinh Ladder tu dong (Yeu cau 1.1 & 2.1):
     + Nhan START (DI_00) -> Bang tai chay (DO_00 = ON); Nhan STOP (DI_01) -> Bang tai dung.
     + Khi phoi chan cam bien bang tai (DI_11) -> Bang tai dung, DO_01 (IN#1) bat ON.
     + Khi Robot tha phoi len Mam Xoay / Bang Tai 2 va phat xung OT#3 (DI_12 canh len) -> DO_02 (IN#2) bat, Mam xoay quay 90 do!

2. Bai Thi Nghiem 3 (Dieu khien Servo Mam Xoay A1 & Thanh Truot B1, C1 bang Machine Controller):
   - Ve diem Zero (ZRN Method 19 - Yeu cau 1.1 & 2.2): Do cam bien DI_10 voi Creep Speed 50 deg/s.
   - Chuong trinh Motion MPM002 (Yeu cau 1.2 & 2.3): Moi khi nhan xung DI_12 tu Robot R1 -> Mam xoay A1 xoay 90 deg
     dua phoi sang phia Robot 2 (R2) va Bang Tai 2!

3. Bai Thi Nghiem 4 (Giao tiep Robot & May Tinh qua High-Speed Ethernet Server UDP Port 10040 + Giao dien Qt/PC):
   - Hien thi truc quan goi tin 32-byte Header "YERC" (0x43524559) + Data Division (Command 0x83 Servo ON/OFF,
     0x86 Start JOB, 0x72 Read Status, 0x75 Read Position).
   - Quy doi xung Encoder tung khop S, L, U, R, B, T theo dung Bang ty le xung/do (Trang 9 - Bai 4):
     S: 34816/30, L: 102400/90, U: 51200/90, R/B/T: 10204/30.

4. He Thong 2 Bang Tai (Conveyor 1 & Conveyor 2) co Encoder + Cam bien vi tri + Auto-Drop:
   - Bang tai 1 (X = 0.22m, chạy hướng +Y): Cấp phôi tự động (Auto-Drop) cho Cánh tay R1.
   - Bang tai 2 (X = 0.38m, chạy hướng -Y): Nhận phôi thành phẩm từ Cánh tay R2, đo tọa độ bằng Encoder 2.
"""

import os
import math
import time
import threading

for var in ('GTK_PATH', 'GIO_MODULE_DIR', 'LOCPATH'):
    os.environ.pop(var, None)
if 'LD_LIBRARY_PATH' in os.environ:
    os.environ['LD_LIBRARY_PATH'] = ':'.join(
        p for p in os.environ['LD_LIBRARY_PATH'].split(':') if '/snap/' not in p
    )

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener


class DualConveyorMP3300Node(Node):
    CONVEYOR_MODES = [
        'STOP_AT_SENSOR',   # Chuẩn Bài TN 2 & 3: Chạy khi DO_00=ON, dừng khi gặp cảm biến DI_11
        'CONTINUOUS',       # Chạy liên tục (Conveyor Tracking bằng Encoder)
        'INDEXING_STEP',    # Nhích từng bước 100mm theo xung Encoder
        'MANUAL_PAUSE',     # Tạm dừng cưỡng bức
    ]

    def __init__(self):
        super().__init__('virtual_conveyor')

        # Publishers: 3D Markers + Tín hiệu I/O sang Robot
        self.marker_pub = self.create_publisher(
            MarkerArray, '/visualization_marker_array', 10
        )
        self.in1_pub = self.create_publisher(Bool, '/yaskawa/din1', 10)   # DO_01 -> Robot IN#1
        self.in2_pub = self.create_publisher(Bool, '/yaskawa/din2', 10)   # DO_02 -> Robot IN#2
        self.enc_pub = self.create_publisher(
            PointStamped, '/conveyor/encoder_tracked_point', 10
        )

        # Subscribers: Nhận tín hiệu từ Robot R1 (OUT#1, OUT#3) và R2 (OUT#2)
        self.vac_r1_sub = self.create_subscription(
            Bool, '/yaskawa/dout1', self.vac_r1_cb, 10
        )
        self.vac_r2_sub = self.create_subscription(
            Bool, '/yaskawa/dout2', self.vac_r2_cb, 10
        )
        self.dout1_pub = self.create_publisher(Bool, '/yaskawa/dout1', 10)
        self.dout2_pub = self.create_publisher(Bool, '/yaskawa/dout2', 10)

        # Đọc góc khớp thực tế của cả 2 cánh tay R1 & R2 để quy đổi xung Pulse (Bài TN 4)
        self.r1_joints = [0.0] * 6
        self.r2_joints = [0.0] * 6
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 10
        )

        # Action Clients điều khiển phối hợp 2 cánh tay R1 & R2
        self.r1_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/manipulator_controller/follow_joint_trajectory',
        )
        self.r2_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/manipulator_r2_controller/follow_joint_trajectory',
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # =================================================================
        # THÔNG SỐ HÌNH HỌC 2 BĂNG TẢI ĐẶT VUÔNG GÓC 90° (L-SHAPE) + MÂM XOAY A1
        #   - Băng tải 1 (CONV1): Chạy dọc theo trục +Y tại X = 0.22m (từ Y = -0.45m đến Y = +0.16m)
        #   - Mâm xoay Trục A1: Đặt ngay góc giao nhau 90° (X = 0.22m, Y = 0.28m)
        #   - Băng tải 2 (CONV2): Đặt VUÔNG GÓC với Băng tải 1, chạy dọc theo trục +X tại Y = 0.38m
        #     (từ X = 0.35m đến X = 0.95m)
        # =================================================================
        self.conv1_x = 0.22           # Băng tải 1 (dọc trục +Y)
        self.conv2_y = 0.38           # Băng tải 2 (vuông góc 90°, dọc trục +X)
        self.rotary_x = 0.22          # Mâm xoay trung gian Trục A tại góc giao 90° (X=0.22, Y=0.28)
        self.rotary_y = 0.28
        self.rotary_angle_deg = 0.0   # Góc mâm xoay Trục A1 (deg)
        self.rotary_target_deg = 0.0
        self.rotary_zeroed = True

        self.spawn1_y = -0.42         # Điểm Auto-Drop phôi xuống đầu Băng tải 1
        self.s1_trig_y = -0.26        # Cảm biến vị trí S1 trên Băng tải 1 (chốt xung Encoder 1)
        self.s2_pick_y = 0.00         # Cảm biến DI_11_1 (Vị trí gắp của R1 trên Băng tải 1)

        self.s3_trig_x = 0.44         # Cảm biến vị trí S3 đầu Băng tải 2 (chốt xung Encoder 2 theo trục X)
        self.s4_exit_x = 0.92         # Cảm biến vị trí S4 cuối Băng tải 2 (DI_11_2 theo trục X)

        self.part_size = 0.04         # Kích thước phôi 40mm
        self.surface_z = 0.02

        self.shape_catalog = [
            ('Khối Hộp Vuông (CUBE)', Marker.CUBE, (1.0, 0.60, 0.0)),
            ('Khối Trụ Tròn (CYLINDER)', Marker.CYLINDER, (0.85, 0.25, 0.90)),
            ('Khối Cầu (SPHERE)', Marker.SPHERE, (0.95, 0.85, 0.10)),
        ]

        # =================================================================
        # TRẠNG THÁI MACHINE CONTROLLER MP3300 (BÀI THÍ NGHIỆM 2 & 3)
        # =================================================================
        self.mc_start = True          # DI_00 (START) -> Cho phép hệ thống băng tải chạy
        self.mc_servo_on = True       # DI_02 (Servo_ON cho các trục phụ A1, B1, C1)
        self.mc_alarm = False         # DO_19 (Alarm Lamp)
        self.conveyor_mode = 'STOP_AT_SENSOR'
        self.conveyor_speed = 0.08    # 80 mm/s
        self.pulses_per_mm = 20.0     # 20 xung/mm

        # Encoder 1 (Băng tải 1) & Encoder 2 (Băng tải 2)
        self.enc1_pulses = 0
        self.enc2_pulses = 0
        self.enc1_latch = None
        self.enc2_latch = None
        self.belt1_running = True
        self.belt2_running = True
        self.index1_target = None
        self.index2_target = None

        # Tín hiệu I/O chuẩn Bảng 1 (Trang 3 & Trang 8 Bài TN 2):
        self.di_11_conv1 = False      # Cảm biến có phôi tại vị trí gắp Băng tải 1
        self.di_11_conv2 = False      # Cảm biến có phôi tại vị trí cuối Băng tải 2
        self.do_01_in1 = False        # MC DO_01 -> Robot IN#1 (Báo có phôi trên Băng tải 1)
        self.do_02_in2 = False        # MC DO_02 -> Robot IN#2 (Báo có phôi trên Mâm xoay / Trạm trung chuyển)
        self.di_12_ot3 = False        # Robot OT#3 -> MC DI_12 (Báo đã đặt phôi lên Mâm xoay -> Xoay 90 độ)

        # Trạng thái Phôi #1 (Cấp từ Băng tải 1 -> R1 gắp -> Đặt lên Mâm xoay / Chuyển sang Băng tải 2)
        self.auto_drop_enabled = True
        self.p1_shape = 0
        self.p1_state = 'DROPPING'    # 'DROPPING', 'CONV1_MOVING', 'CONV1_READY', 'HELD_R1', 'ON_ROTARY', 'HELD_R2', 'CONV2_MOVING', 'CONV2_DONE'
        self.p1_x = self.conv1_x
        self.p1_y = self.spawn1_y
        self.p1_z = 0.18
        self.p1_vz = 0.0
        self.p1_rotary_offset_angle = 0.0
        self.p1_done_timer = 0.0

        # Tọa độ vật thể ước lượng bằng Encoder 1 / Encoder 2
        self.est_x = self.conv1_x
        self.est_y = self.spawn1_y
        self.est_z = self.surface_z
        self.active_enc_name = 'ENC#1 (Băng tải 1)'

        self.vac_r1 = False
        self.vac_r2 = False
        self.coop_running = False
        self.status_msg = 'Hệ thống MP3300 + 2 Cánh tay (R1, R2) + 2 Băng tải đang hoạt động.'
        self.last_yerc_packet = (
            'TX [UDP:10040] "YERC" (0x43524559) | Hdr:0x20 | Data:0x04 | Cmd:0x83 (Servo ON) | Inst:0x02 | Attr:0x1001'
        )

        self.dt = 0.033
        self.timer = self.create_timer(self.dt, self.update_loop)

    def joint_state_cb(self, msg: JointState):
        n2p = dict(zip(msg.name, msg.position))
        r1_n = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        r2_n = ['r2_joint_1', 'r2_joint_2', 'r2_joint_3', 'r2_joint_4', 'r2_joint_5', 'r2_joint_6']
        if all(k in n2p for k in r1_n):
            self.r1_joints = [n2p[k] for k in r1_n]
        if all(k in n2p for k in r2_n):
            self.r2_joints = [n2p[k] for k in r2_n]

    def vac_r1_cb(self, msg: Bool):
        self.vac_r1 = msg.data

    def vac_r2_cb(self, msg: Bool):
        self.vac_r2 = msg.data

    def set_vac_r1(self, on: bool):
        self.vac_r1 = on
        m = Bool()
        m.data = on
        self.dout1_pub.publish(m)

    def set_vac_r2(self, on: bool):
        self.vac_r2 = on
        m = Bool()
        m.data = on
        self.dout2_pub.publish(m)

    def get_tcp_pos(self, frame_id: str):
        try:
            t = self.tf_buffer.lookup_transform('world', frame_id, rclpy.time.Time())
            return (
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            )
        except Exception:
            return None

    def trigger_new_part_conv1(self):
        """Thả tự động (Auto-Drop) một vật mẫu mới xuống đầu Băng tải 1 (X=0.22, Y=-0.36)."""
        self.p1_shape = (self.p1_shape + 1) % len(self.shape_catalog)
        self.p1_x = self.conv1_x
        self.p1_y = self.spawn1_y
        self.p1_z = 0.18
        self.p1_vz = 0.0
        self.p1_state = 'DROPPING'
        self.enc1_latch = None
        self.enc2_latch = None
        self.do_02_in2 = False

    def trigger_rotary_90deg(self):
        """Bài TN 2 & 3 (Yêu cầu 1.2): Khi nhận xung OT#3 (DI_12) từ Robot 1 -> Mâm xoay A1 quay 90 độ."""
        if not self.mc_servo_on:
            self.mc_alarm = True
            self.status_msg = '! ALARM [CallMotion]: Chưa bật Servo_ON (DI_02) mà thực hiện lệnh MOV[A1]900!'
            return
        self.di_12_ot3 = True
        self.rotary_target_deg += 90.0
        self.status_msg = (
            f'🔄 [MP3300 Motion MPM002] Nhận xung OT#3 (DI_12) -> Mâm xoay A1 xoay +90° (đến {self.rotary_target_deg:.0f}°)!'
        )

    def home_rotary_axis_a1(self):
        """Bài TN 3 (Yêu cầu 1.1 - ZRN Method 19): Về điểm Zero cho Mâm xoay A1 theo cảm biến DI_10."""
        self.rotary_target_deg = 0.0
        self.rotary_zeroed = True
        self.mc_alarm = False
        self.status_msg = '🏠 [ZRN[A1]0 - Method 19] Đã dò cảm biến DI_10 và đưa Mâm xoay A1 về gốc 0.0°!'

    def start_index_step(self, step_mm: float = 100.0):
        self.conveyor_mode = 'INDEXING_STEP'
        dp = int(abs(step_mm) * self.pulses_per_mm)
        self.index1_target = self.enc1_pulses + dp
        self.index2_target = self.enc2_pulses + dp
        if self.p1_state == 'CONV1_READY':
            self.p1_state = 'CONV1_MOVING'

    def update_loop(self):
        tcp1 = self.get_tcp_pos('tcp_link')
        tcp2 = self.get_tcp_pos('r2_tcp_link')

        # -----------------------------------------------------------------
        # 1. CẬP NHẬT CHUYỂN ĐỘNG MÂM XOAY SERVO TRỤC A1 (BÀI TN 2 & 3)
        # -----------------------------------------------------------------
        if abs(self.rotary_target_deg - self.rotary_angle_deg) > 0.5:
            step_deg = 90.0 * self.dt  # 90 deg/s
            if self.rotary_target_deg > self.rotary_angle_deg:
                self.rotary_angle_deg = min(self.rotary_target_deg, self.rotary_angle_deg + step_deg)
            else:
                self.rotary_angle_deg = max(self.rotary_target_deg, self.rotary_angle_deg - step_deg)
        else:
            self.rotary_angle_deg = self.rotary_target_deg
            self.di_12_ot3 = False

        # -----------------------------------------------------------------
        # 2. CHƯƠNG TRÌNH LADDER ĐIỀU KHIỂN 2 BĂNG TẢI & ENCODER 1, 2
        # -----------------------------------------------------------------
        if not self.mc_start or self.conveyor_mode == 'MANUAL_PAUSE':
            self.belt1_running = False
            self.belt2_running = False
        elif self.conveyor_mode == 'STOP_AT_SENSOR':
            # Chuẩn Yêu cầu 1.1 (Bài TN 2 Trang 8): Khi không có vật chắn cảm biến DI_11 thì băng tải chạy
            self.belt1_running = (self.p1_state != 'CONV1_READY')
            self.belt2_running = (self.p1_state != 'CONV2_DONE')
        elif self.conveyor_mode == 'CONTINUOUS':
            self.belt1_running = True
            self.belt2_running = True
            if self.p1_state == 'CONV1_READY':
                self.p1_state = 'CONV1_MOVING'
        elif self.conveyor_mode == 'INDEXING_STEP':
            self.belt1_running = (self.index1_target is not None and self.enc1_pulses < self.index1_target)
            self.belt2_running = (self.index2_target is not None and self.enc2_pulses < self.index2_target)

        dy1 = (self.conveyor_speed * self.dt) if self.belt1_running else 0.0
        dy2 = (self.conveyor_speed * self.dt) if self.belt2_running else 0.0

        if dy1 > 0.0:
            self.enc1_pulses += int(round((dy1 * 1000.0) * self.pulses_per_mm))
        if dy2 > 0.0:
            self.enc2_pulses += int(round((dy2 * 1000.0) * self.pulses_per_mm))

        # -----------------------------------------------------------------
        # 3. CẬP NHẬT TRẠNG THÁI VẬT THỂ QUA 2 BĂNG TẢI, MÂM XOAY & 2 ROBOT
        # -----------------------------------------------------------------
        if self.p1_state == 'DROPPING':
            self.p1_vz += 1.8 * self.dt
            self.p1_z -= self.p1_vz
            if self.p1_z <= self.surface_z:
                self.p1_z = self.surface_z
                self.p1_vz = 0.0
                self.p1_state = 'CONV1_MOVING'

        elif self.p1_state == 'CONV1_MOVING':
            prev_y = self.p1_y
            self.p1_y += dy1
            # Chốt xung Encoder 1 khi cắt Cảm biến vị trí S1 (Y = -240mm)
            if prev_y < self.s1_trig_y <= self.p1_y and self.enc1_latch is None:
                self.enc1_latch = self.enc1_pulses
                self.status_msg = f'[ENC#1 LATCH] Phôi qua Cảm biến S1 (Băng tải 1) tại xung = {self.enc1_latch}!'

            # Chạm cảm biến vị trí gắp S2 (DI_11_1 tại Y = 0.00m)
            if self.conveyor_mode == 'STOP_AT_SENSOR' and self.p1_y >= self.s2_pick_y:
                self.p1_y = self.s2_pick_y
                self.p1_state = 'CONV1_READY'
                self.status_msg = '● [MC Ladder] Cảm biến DI_11 (Băng tải 1) = ON -> Dừng Băng tải 1, bật DO_01 (IN#1)!'

            if self.p1_y > 0.38:
                self.p1_state = 'CONV2_DONE'
                self.p1_done_timer = 0.0

            self._check_pickup(tcp1, tcp2)

        elif self.p1_state == 'CONV1_READY':
            self._check_pickup(tcp1, tcp2)

        elif self.p1_state == 'HELD_R1':
            if tcp1 is not None:
                self.p1_x, self.p1_y = tcp1[0], tcp1[1]
                self.p1_z = max(self.surface_z, tcp1[2] - self.part_size / 2.0)
            if not self.vac_r1:
                # Kiểm tra nếu R1 đặt phôi lên Mâm Xoay Trục A (X~0.22, Y~0.28) hay đặt sang Băng tải 2 (Y~0.38)
                if math.hypot(self.p1_x - self.rotary_x, self.p1_y - self.rotary_y) < 0.095:
                    self.p1_state = 'ON_ROTARY'
                    self.p1_z = self.surface_z
                    self.do_02_in2 = True
                    self.p1_rotary_offset_angle = math.atan2(
                        self.p1_y - self.rotary_y, self.p1_x - self.rotary_x
                    ) - math.radians(self.rotary_angle_deg)
                elif abs(self.p1_y - self.conv2_y) < 0.075:
                    self.p1_y = self.conv2_y
                    self.p1_z = self.surface_z
                    self.p1_state = 'CONV2_MOVING'
                    self.enc2_latch = self.enc2_pulses
                else:
                    self.p1_z = self.surface_z
                    self.p1_state = 'CONV2_DONE'
                    self.p1_done_timer = 0.0

        elif self.p1_state == 'ON_ROTARY':
            # Vật thể quay cùng Mâm xoay Trục A1 (bán kính r = 0.045m trên mâm xoay tại góc giao 90° của 2 băng tải)
            r_rad = 0.045
            ang = math.radians(self.rotary_angle_deg) + self.p1_rotary_offset_angle
            self.p1_x = self.rotary_x + r_rad * math.cos(ang)
            self.p1_y = self.rotary_y + r_rad * math.sin(ang)
            self.p1_z = self.surface_z
            self.do_02_in2 = True
            self._check_pickup(tcp1, tcp2)

        elif self.p1_state == 'HELD_R2':
            self.do_02_in2 = False
            if tcp2 is not None:
                self.p1_x, self.p1_y = tcp2[0], tcp2[1]
                self.p1_z = max(self.surface_z, tcp2[2] - self.part_size / 2.0)
            if not self.vac_r2:
                # Nếu R2 đặt phôi lên Băng tải 2 (vuông góc, nằm tại Y ~ 0.38m) -> Phôi chạy dọc trục +X!
                if abs(self.p1_y - self.conv2_y) < 0.085:
                    self.p1_y = self.conv2_y
                    self.p1_z = self.surface_z
                    self.p1_state = 'CONV2_MOVING'
                    self.enc2_latch = self.enc2_pulses
                    self.status_msg = (
                        f'[ENC#2 LATCH] R2 đã đặt phôi lên Băng tải 2 vuông góc (X={self.p1_x*1000:.0f}mm, Y={self.conv2_y*1000:.0f}mm) '
                        f'tại xung Encoder 2 = {self.enc2_latch}!'
                    )
                else:
                    self.p1_z = self.surface_z
                    self.p1_state = 'CONV2_DONE'
                    self.p1_done_timer = 0.0

        elif self.p1_state == 'CONV2_MOVING':
            # Băng tải 2 nằm VUÔNG GÓC với Băng tải 1 -> Chạy dọc theo chiều +X đưa thành phẩm về X = 0.92m!
            self.p1_x += dy2
            if self.p1_x >= self.s4_exit_x:
                self.p1_x = self.s4_exit_x
                self.p1_state = 'CONV2_DONE'
                self.p1_done_timer = 0.0
                self.status_msg = '✅ Phôi thành phẩm đã tới Cảm biến S4 cuối Băng tải 2 vuông góc (X=920mm, Y=380mm)!'
            self._check_pickup(tcp1, tcp2)

        elif self.p1_state == 'CONV2_DONE':
            self._check_pickup(tcp1, tcp2)
            if self.auto_drop_enabled and not self.coop_running:
                self.p1_done_timer += self.dt
                if self.p1_done_timer >= 2.2:
                    self.trigger_new_part_conv1()

        # -----------------------------------------------------------------
        # 4. TÍNH TỌA ĐỘ VẬT THỂ BẰNG ENCODER 1 (TRỤC Y) & ENCODER 2 (TRỤC X VUÔNG GÓC)
        # -----------------------------------------------------------------
        self.di_11_conv1 = (self.p1_state in ('CONV1_MOVING', 'CONV1_READY') and abs(self.p1_y - self.s2_pick_y) < 0.022)
        self.di_11_conv2 = (self.p1_state in ('CONV2_MOVING', 'CONV2_DONE') and abs(self.p1_x - self.s4_exit_x) < 0.025)
        self.do_01_in1 = self.di_11_conv1

        if self.p1_state in ('CONV1_MOVING', 'CONV1_READY') and self.enc1_latch is not None:
            dp1 = self.enc1_pulses - self.enc1_latch
            self.est_x = self.conv1_x
            self.est_y = self.s1_trig_y + (dp1 / self.pulses_per_mm) * 0.001
            self.est_z = self.surface_z
            self.active_enc_name = 'ENC#1 (Băng tải 1 - Trục Y)'
        elif self.p1_state in ('CONV2_MOVING', 'CONV2_DONE') and self.enc2_latch is not None:
            dp2 = self.enc2_pulses - self.enc2_latch
            self.est_x = self.s3_trig_x + (dp2 / self.pulses_per_mm) * 0.001
            self.est_y = self.conv2_y
            self.est_z = self.surface_z
            self.active_enc_name = 'ENC#2 (Băng tải 2 Vuông góc - Trục X)'
        else:
            self.est_x, self.est_y, self.est_z = self.p1_x, self.p1_y, self.p1_z
            self.active_enc_name = 'TRỰC TIẾP (TF / Mâm Xoay A1)'

        # Xuất tín hiệu DO_01 (IN#1) và DO_02 (IN#2) sang Robot Controller
        m1 = Bool()
        m1.data = self.do_01_in1
        self.in1_pub.publish(m1)

        m2 = Bool()
        m2.data = self.do_02_in2
        self.in2_pub.publish(m2)

        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = 'world'
        pt.point.x, pt.point.y, pt.point.z = float(self.est_x), float(self.est_y), float(self.est_z)
        self.enc_pub.publish(pt)

        self.publish_markers(tcp1, tcp2)

    def _check_pickup(self, tcp1, tcp2):
        top_z = self.p1_z + self.part_size / 2.0
        if tcp1 is not None and self.vac_r1:
            if math.hypot(tcp1[0] - self.p1_x, tcp1[1] - self.p1_y) < 0.065 and abs(tcp1[2] - top_z) < 0.065:
                self.p1_state = 'HELD_R1'
                return
        if tcp2 is not None and self.vac_r2:
            if math.hypot(tcp2[0] - self.p1_x, tcp2[1] - self.p1_y) < 0.065 and abs(tcp2[2] - top_z) < 0.065:
                self.p1_state = 'HELD_R2'
                return

    # =====================================================================
    # CHU TRÌNH TỰ ĐỘNG BÀI THÍ NGHIỆM 2 & 3 VỚI 2 CÁNH TAY + 2 BĂNG TẢI
    # =====================================================================
    def send_traj(self, client: ActionClient, joint_names, q_target, duration_sec=1.2):
        if not client.wait_for_server(timeout_sec=1.5):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q_target]
        pt.velocities = [0.0] * 6
        sec = int(duration_sec)
        nsec = int((duration_sec - sec) * 1e9)
        pt.time_from_start = Duration(seconds=sec, nanoseconds=nsec).to_msg()
        goal.trajectory.points = [pt]
        fut = client.send_goal_async(goal)
        while not fut.done():
            time.sleep(0.02)
        gh = fut.result()
        if not gh or not gh.accepted:
            return False
        rf = gh.get_result_async()
        while not rf.done():
            time.sleep(0.02)
        return True

    def run_full_lab2_lab3_dual_cycle(self):
        """
        Thực hiện trọn vẹn quy trình Bài Thí Nghiệm 2 & 3 (Mở rộng 2 Cánh Tay & 2 Băng Tải):
          1. Băng tải 1 chạy (DO_00=ON), phôi qua Cảm biến S1 (chốt xung Encoder 1) và dừng tại Cảm biến DI_11 (Y=0.0m).
          2. Machine Controller bật DO_01 (IN#1 = ON).
          3. Robot 1 (R1) từ P000 di chuyển đến P001 -> xuống P002 trên Băng tải 1 (X=0.22, Y=0.0) -> Bật hút OT#1.
          4. Robot 1 nâng lên P003 -> Di chuyển tới P004/P005 trên Mâm Xoay Trục A (X=0.26, Y=0.22) -> Tắt hút OT#1.
          5. Robot 1 lùi về P000 và xuất xung OT#3 (100ms) vào chân DI_12 của Machine Controller MP3300.
          6. Chương trình Motion MPM002 của MP3300 quay Mâm xoay A1 một góc 90° đưa phôi sang phía Robot 2 (X=0.34, Y=0.22) và bật DO_02 (IN#2 = ON).
          7. Robot 2 (R2) lập tức gắp phôi từ Mâm xoay đặt sang đầu Băng tải 2 (X=0.38, Y=0.16).
          8. Băng tải 2 chạy (đo bằng Encoder 2) đưa phôi thành phẩm về cuối Băng tải 2 (Y=-0.25m)!
        """
        if self.coop_running:
            return
        self.coop_running = True

        def _worker():
            r1_j = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            r2_j = ['r2_joint_1', 'r2_joint_2', 'r2_joint_3', 'r2_joint_4', 'r2_joint_5', 'r2_joint_6']

            home_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # Tư thế R1 (gốc X=0.0, Y=0.0) gắp tại Băng tải 1 (X=0.22, Y=0.0)
            r1_p001_above = [0.000, 0.420, 0.160, 0.000, 0.260, 0.000]
            r1_p002_pick  = [0.000, 0.640, -0.020, 0.000, 0.660, 0.000]
            # Tư thế R1 đặt lên Mâm xoay Trục A tại góc giao 90° (X=0.22, Y=0.235)
            r1_p004_rot_above = [0.818, 0.560, 0.240, 0.000, 0.320, 0.000]
            r1_p005_rot_place = [0.818, 0.740, 0.060, 0.000, 0.680, 0.000]

            # Tư thế R2 (gốc X=0.58, Y=0.14, xoay +90° hướng +Y) gắp từ Mâm xoay sau khi xoay 90° (X=0.265, Y=0.28)
            r2_rot_above = [1.130, 0.560, 0.240, 0.000, 0.320, 0.000]
            r2_rot_pick  = [1.130, 0.740, 0.060, 0.000, 0.680, 0.000]
            # Tư thế R2 đặt sang đầu Băng tải 2 vuông góc (X=0.44, Y=0.38)
            r2_conv2_above = [0.520, 0.480, 0.180, 0.000, 0.300, 0.000]
            r2_conv2_place = [0.520, 0.680, 0.000, 0.000, 0.680, 0.000]

            try:
                self.mc_start = True
                if self.p1_state not in ('CONV1_MOVING', 'CONV1_READY', 'DROPPING'):
                    self.trigger_new_part_conv1()

                self.status_msg = '⏳ [Bài TN 2 - Bước 1] Đang chờ phôi trên Băng tải 1 chạm cảm biến DI_11 -> Bật DO_01 (IN#1)...'
                t_wait = 0.0
                while self.p1_state != 'CONV1_READY' and t_wait < 8.0:
                    time.sleep(0.05)
                    t_wait += 0.05

                self.last_yerc_packet = (
                    'TX [UDP:10040 -> R1] "YERC" | Cmd:0x86 (Start JOB:LAB2-R1) | IN#1=ON, IN#2=OFF'
                )
                self.status_msg = '🤖 [Bài TN 2 - Bước 2] IN#1=ON & IN#2=OFF -> Robot 1 (R1) đến P001 -> P002 gắp vật trên Băng tải 1...'
                self.send_traj(self.r1_client, r1_j, r1_p001_above, 1.0)
                self.send_traj(self.r1_client, r1_j, r1_p002_pick, 0.85)
                self.set_vac_r1(True)
                time.sleep(0.3)
                self.send_traj(self.r1_client, r1_j, r1_p001_above, 0.9)

                self.status_msg = '🤖 [Bài TN 2 - Bước 3 & 4] R1 đưa phôi qua điểm trung gian P003 -> P004 -> Đặt xuống Mâm xoay P005...'
                self.send_traj(self.r1_client, r1_j, r1_p004_rot_above, 1.1)
                self.send_traj(self.r1_client, r1_j, r1_p005_rot_place, 0.85)
                # Gán tọa độ phôi tại mép dưới Mâm xoay (-90° so với tâm mâm xoay X=0.22, Y=0.28)
                self.p1_x, self.p1_y = 0.22, 0.235
                self.set_vac_r1(False)
                self.p1_state = 'ON_ROTARY'
                self.p1_rotary_offset_angle = (-math.pi / 2.0) - math.radians(self.rotary_angle_deg)
                time.sleep(0.3)
                self.send_traj(self.r1_client, r1_j, r1_p004_rot_above, 0.85)
                self.send_traj(self.r1_client, r1_j, home_q, 1.0)

                # Bước 4 (Trang 8 Bài TN 2): R1 phát xung OT#3 (100ms) báo cho Machine Controller xoay mâm 90 độ
                self.status_msg = '⚡ [Bài TN 2 & 3] R1 phát xung PULSE OT#(3) T=0.10s -> MP3300 nhận DI_12 kích Mâm xoay A1 quay 90°!'
                self.trigger_rotary_90deg()
                time.sleep(1.3)

                # Mâm xoay đã quay 90° đưa phôi sang hướng +X (X=0.265, Y=0.28) -> Robot 2 gắp đặt sang Băng tải 2 vuông góc
                self.last_yerc_packet = (
                    'TX [UDP:10040 -> R2] "YERC" | Cmd:0x86 (Start JOB:LAB3-R2) | DO_02 (IN#2)=ON'
                )
                self.status_msg = '🤝 [Phối hợp R2] Mâm xoay đã quay 90° -> Robot 2 (R2) gắp phôi đặt sang Băng tải 2 vuông góc...'
                self.send_traj(self.r2_client, r2_j, r2_rot_above, 1.0)
                self.send_traj(self.r2_client, r2_j, r2_rot_pick, 0.85)
                self.set_vac_r2(True)
                self.p1_state = 'HELD_R2'
                time.sleep(0.3)
                self.send_traj(self.r2_client, r2_j, r2_rot_above, 0.9)

                self.send_traj(self.r2_client, r2_j, r2_conv2_above, 1.0)
                self.send_traj(self.r2_client, r2_j, r2_conv2_place, 0.85)
                self.p1_x, self.p1_y = self.s3_trig_x, self.conv2_y
                self.set_vac_r2(False)
                self.p1_state = 'CONV2_MOVING'
                self.enc2_latch = self.enc2_pulses
                time.sleep(0.3)
                self.send_traj(self.r2_client, r2_j, r2_conv2_above, 0.85)
                self.send_traj(self.r2_client, r2_j, home_q, 1.0)

                self.status_msg = '📦 [Băng tải 2 Vuông góc + Encoder 2] Phôi đang chạy dọc trục +X về cuối Băng tải 2...'
            finally:
                self.coop_running = False

        threading.Thread(target=_worker, daemon=True).start()

    def publish_markers(self, tcp1, tcp2):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        _, shape_type, base_rgb = self.shape_catalog[self.p1_shape]

        # 1. Marker Phôi 3D
        pm = Marker()
        pm.ns = 'workcell'
        pm.id = 1
        pm.type = shape_type
        pm.action = Marker.ADD
        pm.scale.x = pm.scale.y = pm.scale.z = self.part_size

        if self.p1_state == 'HELD_R1':
            pm.header.frame_id = 'tcp_link'
            pm.header.stamp = rclpy.time.Time().to_msg()
            pm.pose.position.z = float(self.part_size / 2.0)
            pm.pose.orientation.w = 1.0
            pm.frame_locked = True
        elif self.p1_state == 'HELD_R2':
            pm.header.frame_id = 'r2_tcp_link'
            pm.header.stamp = rclpy.time.Time().to_msg()
            pm.pose.position.z = float(self.part_size / 2.0)
            pm.pose.orientation.w = 1.0
            pm.frame_locked = True
        else:
            pm.header.frame_id = 'world'
            pm.header.stamp = now
            pm.pose.position.x = float(self.p1_x)
            pm.pose.position.y = float(self.p1_y)
            pm.pose.position.z = float(max(self.surface_z, self.p1_z))
            pm.pose.orientation.w = 1.0
            pm.frame_locked = False

        if self.p1_state in ('HELD_R1', 'HELD_R2'):
            pm.color.r, pm.color.g, pm.color.b = 0.1, 0.85, 1.0
        else:
            pm.color.r, pm.color.g, pm.color.b = base_rgb
        pm.color.a = 1.0
        ma.markers.append(pm)

        # 2. Cảm biến S1 & S2 trên Băng tải 1 (dọc trục Y) | Cảm biến S3 & S4 trên Băng tải 2 (vuông góc dọc trục X)
        for mid, mx, my, active, rgb_on, rgb_off in [
            (2, self.conv1_x - 0.08, self.s2_pick_y, self.di_11_conv1, (0.0, 1.0, 0.0), (0.8, 0.1, 0.1)),
            (4, self.conv1_x - 0.08, self.s1_trig_y, abs(self.p1_y - self.s1_trig_y) < 0.025, (0.0, 1.0, 1.0), (1.0, 0.65, 0.0)),
            (6, self.s3_trig_x, self.conv2_y + 0.08, abs(self.p1_x - self.s3_trig_x) < 0.025 and self.p1_state == 'CONV2_MOVING', (0.0, 1.0, 1.0), (1.0, 0.65, 0.0)),
            (7, self.s4_exit_x, self.conv2_y + 0.08, self.di_11_conv2, (0.0, 1.0, 0.0), (0.8, 0.1, 0.1)),
        ]:
            sm = Marker()
            sm.header.frame_id = 'world'
            sm.header.stamp = now
            sm.ns = 'workcell'
            sm.id = mid
            sm.type = Marker.SPHERE
            sm.action = Marker.ADD
            sm.pose.position.x = mx
            sm.pose.position.y = my
            sm.pose.position.z = 0.02
            sm.pose.orientation.w = 1.0
            sm.scale.x = sm.scale.y = sm.scale.z = 0.024
            c = rgb_on if active else rgb_off
            sm.color.r, sm.color.g, sm.color.b, sm.color.a = c[0], c[1], c[2], 1.0
            ma.markers.append(sm)

        # 3. Vạch chữ thập đen trên Mâm xoay Trục A (Giống hệt Hình 3 Bài TN 2 & 3) quay theo góc rotary_angle_deg
        cross = Marker()
        cross.header.frame_id = 'world'
        cross.header.stamp = now
        cross.ns = 'workcell'
        cross.id = 8
        cross.type = Marker.CUBE
        cross.action = Marker.ADD
        cross.pose.position.x = self.rotary_x
        cross.pose.position.y = self.rotary_y
        cross.pose.position.z = 0.002
        half_yaw = math.radians(self.rotary_angle_deg) * 0.5
        cross.pose.orientation.z = math.sin(half_yaw)
        cross.pose.orientation.w = math.cos(half_yaw)
        cross.scale.x = 0.14
        cross.scale.y = 0.022
        cross.scale.z = 0.004
        cross.color.r, cross.color.g, cross.color.b, cross.color.a = 0.15, 0.15, 0.18, 1.0
        ma.markers.append(cross)

        self.marker_pub.publish(ma)


# =====================================================================
# GIAO DIỆN SỐ 2: MP3300 MACHINE CONTROLLER + 2 BĂNG TẢI ENCODER + ETHERNET
# (Tích hợp đầy đủ Bài Thí Nghiệm 2, 3, 4 cho 2 Cánh Tay & 2 Băng Tải)
# =====================================================================
class MP3300DualConveyorGUI:
    # Tỷ lệ xung/độ chuẩn Bảng trang 9 - Bài Thí Nghiệm 4:
    # S: 34816/30, L: 102400/90, U: 51200/90, R/B/T: 10204/30
    PULSE_RATIOS = [
        34816.0 / 30.0,
        102400.0 / 90.0,
        51200.0 / 90.0,
        10204.0 / 30.0,
        10204.0 / 30.0,
        10204.0 / 30.0,
    ]

    def __init__(self, node: DualConveyorMP3300Node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('MP3300 MACHINE CONTROLLER & 2 BĂNG TẢI ENCODER (Bài Thí Nghiệm 2, 3, 4 - ĐHBK TPHCM)')
        scr_w = self.root.winfo_screenwidth()
        scr_h = self.root.winfo_screenheight()
        win_w = min(900, scr_w // 2 - 10)
        win_h = 540
        pos_y = max(30, scr_h - win_h - 55)
        self.root.geometry(f'{win_w}x{win_h}+0+{pos_y}')
        self.root.configure(bg='#1e2229')

        self._build_ui()
        self.root.after(80, self._update_ui)

    def _build_ui(self):
        hdr = tk.Frame(self.root, bg='#0b7285', padx=8, pady=4)
        hdr.pack(fill=tk.X)
        tk.Label(
            hdr,
            text='🎛️ MP3300 MACHINE CONTROLLER + 2 BĂNG TẢI ENCODER + 2 ROBOT MOTOMINI (BÀI TN 2, 3, 4)',
            fg='white',
            bg='#0b7285',
            font=('Ubuntu', 9, 'bold'),
        ).pack(side=tk.LEFT)

        # Thanh trạng thái chung
        self.lbl_status = tk.Label(
            self.root,
            text='',
            bg='#212529',
            fg='#ffd43b',
            font=('Ubuntu', 8, 'bold'),
            anchor='w',
            padx=8,
            pady=3,
        )
        self.lbl_status.pack(fill=tk.X)

        main = tk.Frame(self.root, bg='#1e2229', padx=6, pady=4)
        main.pack(fill=tk.BOTH, expand=True)

        left_col = tk.Frame(main, bg='#1e2229')
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 3))

        right_col = tk.Frame(main, bg='#1e2229')
        right_col.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(3, 0))

        # --- KHUNG 1 (TRÁI TRÊN): ENCODER 2 BĂNG TẢI & TỌA ĐỘ VẬT THỂ ---
        f_enc = tk.LabelFrame(
            left_col,
            text=' 1. ĐO LƯỜNG 2 ENCODER BĂNG TẢI & TỌA ĐỘ VẬT THỂ ',
            bg='#282c34',
            fg='#66d9e8',
            font=('Ubuntu', 8, 'bold'),
            padx=6,
            pady=4,
        )
        f_enc.pack(fill=tk.X, pady=(0, 4))

        self.lbl_enc12 = tk.Label(
            f_enc,
            text='',
            bg='#1b1f24',
            fg='#ffd43b',
            font=('Ubuntu Mono', 8, 'bold'),
            justify=tk.LEFT,
            anchor='w',
            padx=6,
            pady=3,
        )
        self.lbl_enc12.pack(fill=tk.X, pady=1)

        self.lbl_pose = tk.Label(
            f_enc,
            text='',
            bg='#1b1f24',
            fg='#74c0fc',
            font=('Ubuntu Mono', 8, 'bold'),
            justify=tk.LEFT,
            anchor='w',
            padx=6,
            pady=3,
        )
        self.lbl_pose.pack(fill=tk.X, pady=1)

        # --- KHUNG 2 (TRÁI GIỮA): BẢNG KẾT NỐI I/O MP3300 & MÂM XOAY TRỤC A1 (BÀI TN 2 & 3) ---
        f_mp = tk.LabelFrame(
            left_col,
            text=' 2. GIAO TIẾP I/O MP3300 & SERVO MÂM XOAY A1 (BÀI TN 2 & 3) ',
            bg='#282c34',
            fg='#fcc419',
            font=('Ubuntu', 8, 'bold'),
            padx=6,
            pady=4,
        )
        f_mp.pack(fill=tk.X, pady=(0, 4))

        self.lbl_mp_io = tk.Label(
            f_mp,
            text='',
            bg='#1b1f24',
            fg='#8ce99a',
            font=('Ubuntu Mono', 8, 'bold'),
            justify=tk.LEFT,
            anchor='w',
            padx=6,
            pady=3,
        )
        self.lbl_mp_io.pack(fill=tk.X, pady=2)

        mp_btns = tk.Frame(f_mp, bg='#282c34')
        mp_btns.pack(fill=tk.X, pady=2)
        tk.Button(
            mp_btns,
            text='▶ DI_00: START',
            bg='#2b8a3e',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            command=lambda: setattr(self.node, 'mc_start', True),
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        tk.Button(
            mp_btns,
            text='⏹ DI_01: STOP',
            bg='#c92a2a',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            command=lambda: setattr(self.node, 'mc_start', False),
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        tk.Button(
            mp_btns,
            text='🏠 ZRN[A1]0 (Về Zero)',
            bg='#1864ab',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            command=self.node.home_rotary_axis_a1,
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        tk.Button(
            mp_btns,
            text='🔄 Xoay A1 +90° (DI_12)',
            bg='#e67700',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            command=self.node.trigger_rotary_90deg,
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        # --- KHUNG 3 (TRÁI DƯỚI): BÀI THÍ NGHIỆM 4 - HIGH SPEED ETHERNET (YERC UDP:10040) & XUNG KHỚP ---
        f_eth = tk.LabelFrame(
            left_col,
            text=' 3. BÀI TN 4: HIGH-SPEED ETHERNET (UDP:10040) & XUNG KHỚP (PULSE) ',
            bg='#282c34',
            fg='#ff8787',
            font=('Ubuntu', 8, 'bold'),
            padx=6,
            pady=4,
        )
        f_eth.pack(fill=tk.BOTH, expand=True)

        self.lbl_yerc = tk.Label(
            f_eth,
            text='',
            bg='#1b1f24',
            fg='#ffa94d',
            font=('Ubuntu Mono', 7, 'bold'),
            anchor='w',
            padx=4,
            pady=2,
        )
        self.lbl_yerc.pack(fill=tk.X, pady=1)

        self.lbl_pulses_r12 = tk.Label(
            f_eth,
            text='',
            bg='#1b1f24',
            fg='#dee2e6',
            font=('Ubuntu Mono', 8, 'bold'),
            justify=tk.LEFT,
            anchor='nw',
            padx=6,
            pady=3,
        )
        self.lbl_pulses_r12.pack(fill=tk.BOTH, expand=True, pady=1)

        # --- KHUNG 4 (PHẢI TRÊN): CHẾ ĐỘ 2 BĂNG TẢI & AUTO-DROP ---
        f_mode = tk.LabelFrame(
            right_col,
            text=' 4. CHẾ ĐỘ CHẠY 2 BĂNG TẢI & AUTO-DROP PHÔI ',
            bg='#282c34',
            fg='#69db7c',
            font=('Ubuntu', 8, 'bold'),
            padx=6,
            pady=4,
        )
        f_mode.pack(fill=tk.X, pady=(0, 4))

        mgrid = tk.Frame(f_mode, bg='#282c34')
        mgrid.pack(fill=tk.X)
        self.m_btns = {}
        for mc, txt, r, c in [
            ('STOP_AT_SENSOR', '1. Chạy -> Dừng Cảm Biến', 0, 0),
            ('CONTINUOUS', '2. Chạy Liên Tục (Tracking)', 0, 1),
            ('INDEXING_STEP', '3. Nhích 100mm (Encoder)', 1, 0),
            ('MANUAL_PAUSE', '4. Tạm Dừng 2 Băng Tải', 1, 1),
        ]:
            b = tk.Button(
                mgrid,
                text=txt,
                font=('Ubuntu', 8, 'bold'),
                command=lambda m=mc: self._set_mode(m),
            )
            b.grid(row=r, column=c, sticky='we', padx=2, pady=2)
            self.m_btns[mc] = b
        mgrid.columnconfigure(0, weight=1)
        mgrid.columnconfigure(1, weight=1)

        spd_row = tk.Frame(f_mode, bg='#282c34')
        spd_row.pack(fill=tk.X, pady=2)
        tk.Label(
            spd_row, text='Tốc độ 2 băng tải (mm/s):', bg='#282c34', fg='white', font=('Ubuntu', 8, 'bold')
        ).pack(side=tk.LEFT)
        self.spd_scale = tk.Scale(
            spd_row,
            from_=20,
            to=200,
            orient=tk.HORIZONTAL,
            bg='#282c34',
            fg='#ffd43b',
            highlightthickness=0,
            font=('Ubuntu', 8, 'bold'),
            command=lambda v: setattr(self.node, 'conveyor_speed', float(v) / 1000.0),
        )
        self.spd_scale.set(80)
        self.spd_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)

        drop_row = tk.Frame(f_mode, bg='#282c34')
        drop_row.pack(fill=tk.X, pady=2)
        self.btn_ad = tk.Button(
            drop_row,
            text='✅ AUTO-DROP: BẬT',
            bg='#2b8a3e',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            command=self._toggle_autodrop,
        )
        self.btn_ad.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        tk.Button(
            drop_row,
            text='⬇️ Thả Phôi Xuống Băng Tải 1',
            bg='#e67700',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            command=self.node.trigger_new_part_conv1,
        ).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=2)

        # --- KHUNG 5 (PHẢI DƯỚI): CHẠY PHỐI HỢP 2 CÁNH TAY + 2 BĂNG TẢI + MÂM XOAY ---
        f_coop = tk.LabelFrame(
            right_col,
            text=' 5. KỊCH BẢN PHỐI HỢP 2 CÁNH TAY & 2 BĂNG TẢI (BÀI TN 2, 3, 4) ',
            bg='#282c34',
            fg='#ffa94d',
            font=('Ubuntu', 8, 'bold'),
            padx=6,
            pady=4,
        )
        f_coop.pack(fill=tk.BOTH, expand=True)

        tk.Button(
            f_coop,
            text='🚀 CHẠY CHU TRÌNH BÀI TN 2 & 3:\nBăng tải 1 -> R1 gắp đặt Mâm xoay -> Xoay 90° -> R2 gắp sang Băng tải 2',
            bg='#1864ab',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            pady=6,
            command=self.node.run_full_lab2_lab3_dual_cycle,
        ).pack(fill=tk.X, pady=3)

        tk.Label(
            f_coop,
            text=(
                'Quy trình tự động hoàn chỉnh:\n'
                '• B1: Phôi rơi xuống Băng tải 1, qua Cảm biến S1 chốt Encoder 1, dừng ở DI_11.\n'
                '• B2: MP3300 bật DO_01 (IN#1) -> R1 gắp phôi đặt lên Mâm xoay A1.\n'
                '• B3: R1 phát xung OT#3 (DI_12) -> Mâm xoay A1 quay 90° đưa phôi sang R2.\n'
                '• B4: R2 gắp phôi từ Mâm xoay đặt sang Băng tải 2 (Encoder 2 đo tọa độ).'
            ),
            bg='#1b1f24',
            fg='#ced4da',
            font=('Ubuntu', 8),
            justify=tk.LEFT,
            anchor='nw',
            padx=6,
            pady=4,
        ).pack(fill=tk.BOTH, expand=True)

    def _set_mode(self, m: str):
        if m == 'INDEXING_STEP':
            self.node.start_index_step(100.0)
        else:
            self.node.conveyor_mode = m

    def _toggle_autodrop(self):
        self.node.auto_drop_enabled = not self.node.auto_drop_enabled
        if self.node.auto_drop_enabled:
            self.btn_ad.config(text='✅ AUTO-DROP: BẬT', bg='#2b8a3e')
        else:
            self.btn_ad.config(text='⏸ AUTO-DROP: TẮT', bg='#495057')

    def _update_ui(self):
        self.lbl_status.config(text=f'📢 {self.node.status_msg}')

        l1 = str(self.node.enc1_latch) if self.node.enc1_latch is not None else '---'
        l2 = str(self.node.enc2_latch) if self.node.enc2_latch is not None else '---'
        self.lbl_enc12.config(
            text=(
                f'Băng tải 1 (Dọc +Y)       : {self.node.enc1_pulses:7d} pulses (Latch S1: {l1})\n'
                f'Băng tải 2 (Vuông góc +X): {self.node.enc2_pulses:7d} pulses (Latch S3: {l2})'
            )
        )

        sname = self.node.shape_catalog[self.node.p1_shape][0]
        self.lbl_pose.config(
            text=(
                f'Phôi: {sname} | TT: {self.node.p1_state}\n'
                f'Nguồn đo: {self.node.active_enc_name}\n'
                f'Tọa độ Encoder: X={self.node.est_x*1000:5.1f}  Y={self.node.est_y*1000:6.1f}  Z={self.node.est_z*1000:4.1f} mm\n'
                f'Tọa độ Thực tế: X={self.node.p1_x*1000:5.1f}  Y={self.node.p1_y*1000:6.1f}  Z={self.node.p1_z*1000:4.1f} mm'
            )
        )

        self.lbl_mp_io.config(
            text=(
                f'DO_00 (Inverter Đai): {"ON " if self.node.mc_start else "OFF"} | '
                f'DI_11 (Sensor Đai 1): {"ON " if self.node.di_11_conv1 else "OFF"} | '
                f'DO_01 (Robot IN#1): {"ON" if self.node.do_01_in1 else "OFF"}\n'
                f'DI_12 (Robot OT#3)  : {"ON " if self.node.di_12_ot3 else "OFF"} | '
                f'DO_02 (Robot IN#2)  : {"ON " if self.node.do_02_in2 else "OFF"} | '
                f'Mâm xoay A1: {self.node.rotary_angle_deg:5.1f}°'
            )
        )

        self.lbl_yerc.config(text=self.node.last_yerc_packet)

        # Tính xung khớp S, L, U, R, B, T theo đúng Bảng tỷ lệ xung/độ trang 9 Bài TN 4
        r1_p = [int(round(math.degrees(self.node.r1_joints[i]) * self.PULSE_RATIOS[i])) for i in range(6)]
        r2_p = [int(round(math.degrees(self.node.r2_joints[i]) * self.PULSE_RATIOS[i])) for i in range(6)]
        self.lbl_pulses_r12.config(
            text=(
                f'R1 Pulses: S={r1_p[0]:6d} L={r1_p[1]:6d} U={r1_p[2]:6d} R={r1_p[3]:5d} B={r1_p[4]:5d} T={r1_p[5]:5d}\n'
                f'R2 Pulses: S={r2_p[0]:6d} L={r2_p[1]:6d} U={r2_p[2]:6d} R={r2_p[3]:5d} B={r2_p[4]:5d} T={r2_p[5]:5d}'
            )
        )

        for mc, b in self.m_btns.items():
            if mc == self.node.conveyor_mode:
                b.config(bg='#fcc419', fg='black')
            else:
                b.config(bg='#495057', fg='white')

        self.root.after(80, self._update_ui)

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = DualConveyorMP3300Node()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        gui = MP3300DualConveyorGUI(node)
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
