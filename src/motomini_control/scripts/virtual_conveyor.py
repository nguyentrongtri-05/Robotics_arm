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
from std_msgs.msg import Bool, Float64MultiArray
from geometry_msgs.msg import PointStamped, Point
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
        # Publisher Hệ tọa độ USER gắn trực tiếp vào Đối tượng (50Hz Zero-Lag): [x, y, z_top, yaw, vx, vy, wz]
        self.user_frame_pub = self.create_publisher(
            Float64MultiArray, '/conveyor/object_user_frame', 10
        )

        # Subscribers: Nhận tín hiệu từ Robot R1 (OUT#1, OUT#3) và R2 (OUT#2)
        self.vac_r1_sub = self.create_subscription(
            Bool, '/yaskawa/dout1', self.vac_r1_cb, 10
        )
        self.vac_r2_sub = self.create_subscription(
            Bool, '/yaskawa/dout2', self.vac_r2_cb, 10
        )
        self.ot3_sub = self.create_subscription(
            Bool, '/yaskawa/dout3', self.ot3_cb, 10
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
        #   - Băng tải 1 (CONV1): Chạy dọc theo trục +Y tại X = 0.22m (từ Y = -0.42m đến Y = +0.14m)
        #   - Đĩa xoay Trục A1: Đặt độc lập ở khoảng trống góc vuông (X = 0.22m, Y = 0.26m)
        #   - Băng tải 2 (CONV2): Đặt VUÔNG GÓC với Băng tải 1, chạy dọc theo trục +X tại Y = 0.26m
        #     (từ X = 0.34m đến X = 0.94m) -> Hoàn toàn không chồng lấn lên đĩa xoay!
        # =================================================================
        self.conv1_x = 0.22           # Băng tải 1 (dọc trục +Y)
        self.conv2_y = 0.26           # Băng tải 2 (vuông góc 90°, dọc trục +X)
        self.rotary_x = 0.22          # Đĩa xoay trung gian Trục A tại góc giao 90° (X=0.22, Y=0.26)
        self.rotary_y = 0.26
        self.rotary_angle_deg = 0.0   # Góc đĩa xoay Trục A1 (deg)
        self.rotary_target_deg = 0.0
        self.rotary_zeroed = True

        self.spawn1_y = -0.38         # Điểm Auto-Drop phôi xuống đầu Băng tải 1
        self.s1_trig_y = -0.24        # Cảm biến vị trí S1 trên Băng tải 1 (chốt xung Encoder 1)
        self.s2_pick_y = 0.00         # Cảm biến DI_11_1 (Vị trí gắp của R1 trên Băng tải 1)

        self.s3_trig_x = 0.40         # Cảm biến vị trí S3 đầu Băng tải 2 (chốt xung Encoder 2 theo trục X)
        self.s4_exit_x = 0.88         # Cảm biến vị trí S4 cuối Băng tải 2 (DI_11_2 theo trục X)

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

        # Tọa độ vật thể do Code của Người dùng tính từ Encoder 1 / Encoder 2 & Cảm biến S1..S4
        self.est_x = self.conv1_x
        self.est_y = self.spawn1_y
        self.est_z = self.surface_z + self.part_size / 2.0
        self.active_enc_name = 'USER_CODE (Encoder + Sensor)'

        # Bộ nhớ lưu biến chốt xung (Latch) dành cho Code tính tọa độ của người dùng trên GUI
        self.user_mem = {
            'latch1': None,
            'latch2': None,
            'rot_start_deg': 0.0,
            'stage': 'CONV1',
        }
        self.s1_sensor = False
        self.s2_sensor = False
        self.s3_sensor = False
        self.s4_sensor = False
        self.user_code_error = ''
        self.user_coord_code = ''
        self.user_coord_compiled = None
        self._load_default_user_coord_code()

        self.vac_r1 = False
        self.vac_r2 = False
        self.coop_running = False
        self.status_msg = 'Hệ thống MP3300 + 2 Cánh tay (R1, R2) + 2 Băng tải đang hoạt động.'
        self.last_yerc_packet = (
            'TX [UDP:10040] "YERC" (0x43524559) | Hdr:0x20 | Data:0x04 | Cmd:0x83 (Servo ON) | Inst:0x02 | Attr:0x1001'
        )

        self.mpm002_step_deg = 90.0
        self.mpm002_vel_deg_s = 90.0
        self.compiled_ladder_rungs = []
        self.obj_yaw = 0.0
        self.obj_vx = 0.0
        self.obj_vy = 0.0
        self.obj_wz = 0.0

        # Tần số quét 50 Hz (dt = 0.02s) đảm bảo truyền tọa độ Hệ USER & vận tốc Feedforward không trễ
        self.dt = 0.020
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

    def ot3_cb(self, msg: Bool):
        if msg.data:
            self.trigger_rotary_90deg()

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

    def _load_default_user_coord_code(self):
        plc_dir = os.path.abspath(
            os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'plc')
        )
        fpath = os.path.join(plc_dir, 'user_object_coord_calc.py')
        if os.path.exists(fpath):
            with open(fpath, 'r', encoding='utf-8') as f:
                code_str = f.read()
        else:
            code_str = (
                "x_obj = 220.0\n"
                "y_obj = -240.0 + (enc1_pulses - (mem.get('latch1') or enc1_pulses)) / pulses_per_mm\n"
                "z_obj = 40.0\n"
                "yaw_obj = 0.0\n"
            )
        self.compile_user_coord_code(code_str)

    def compile_user_coord_code(self, code_str: str) -> bool:
        self.user_coord_code = code_str
        try:
            self.user_coord_compiled = compile(code_str, '<user_object_coord_calc.py>', 'exec')
            self.user_code_error = 'OK'
            return True
        except Exception as e:
            self.user_code_error = f'Lỗi cú pháp Python: {e}'
            return False

    def trigger_new_part_conv1(self):
        """Thả tự động (Auto-Drop) một vật mẫu mới xuống đầu Băng tải 1 (X=0.22, Y=-0.38)."""
        self.p1_shape = (self.p1_shape + 1) % len(self.shape_catalog)
        self.p1_x = self.conv1_x
        self.p1_y = self.spawn1_y
        self.p1_z = 0.18
        self.p1_vz = 0.0
        self.p1_state = 'DROPPING'
        self.enc1_latch = None
        self.enc2_latch = None
        self.do_02_in2 = False
        self.user_mem = {
            'latch1': None,
            'latch2': None,
            'rot_start_deg': 0.0,
            'stage': 'CONV1',
        }

    def trigger_rotary_90deg(self):
        """Bài TN 2 & 3 (Yêu cầu 1.2): Khi nhận xung OT#3 (DI_12) từ Robot 1 -> Mâm xoay A1 quay 90 độ."""
        if not self.mc_servo_on:
            self.mc_alarm = True
            self.status_msg = '! ALARM [CallMotion]: Chưa bật Servo_ON (DI_02) mà thực hiện lệnh MOV[A1]900!'
            return
        self.di_12_ot3 = True
        self.rotary_target_deg += self.mpm002_step_deg
        self.status_msg = (
            f'[MP3300 Motion MPM002] Nhận xung OT#3 (DI_12) -> Mâm xoay A1 xoay +{self.mpm002_step_deg:.0f}° (đến {self.rotary_target_deg:.0f}°)!'
        )

    def home_rotary_axis_a1(self):
        """Bài TN 3 (Yêu cầu 1.1 - ZRN Method 19): Về điểm Zero cho Mâm xoay A1 theo cảm biến DI_10."""
        self.rotary_target_deg = 0.0
        self.rotary_zeroed = True
        self.mc_alarm = False
        self.status_msg = ' [ZRN[A1]0 - Method 19] Đã dò cảm biến DI_10 và đưa Mâm xoay A1 về gốc 0.0°!'

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
            step_deg = self.mpm002_vel_deg_s * self.dt
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
        self.di_11_conv1 = (self.p1_state in ('CONV1_MOVING', 'CONV1_READY') and abs(self.p1_y - self.s2_pick_y) < 0.022)
        self.di_11_conv2 = (self.p1_state in ('CONV2_MOVING', 'CONV2_DONE') and abs(self.p1_x - self.s4_exit_x) < 0.025)

        if not self.mc_start or self.conveyor_mode == 'MANUAL_PAUSE':
            self.belt1_running = False
            self.belt2_running = False
            self.do_01_in1 = False
        elif self.conveyor_mode == 'STOP_AT_SENSOR':
            if self.compiled_ladder_rungs:
                plc_mem = {
                    'IB0C400': self.mc_start,
                    'IB0C401': not self.mc_start,
                    'IB0C402': self.mc_servo_on,
                    'IB0C40A': abs(self.rotary_angle_deg % 360.0) < 2.0,
                    'IB0C40B': self.di_11_conv1,
                    'IB0C40C': self.di_12_ot3,
                    'IB0C40D': self.di_11_conv2,
                    'DB000000': self.mc_start,
                    'MB001002': (self.p1_state == 'ON_ROTARY'),
                    'OB0C400': False,
                    'OB0C401': False,
                    'OB0C402': False,
                    'OB0C403': False,
                    'MB001010': False,
                }
                for rg in self.compiled_ladder_rungs:
                    if rg.get('type') == 'MSEE':
                        continue
                    c1_v = plc_mem.get(rg.get('c1_addr', 'IB0C400'), False)
                    c1_on = (not c1_v) if rg.get('c1_type') == 'NC' else c1_v
                    if rg.get('type') == 'BRANCH_OR':
                        b1_on = plc_mem.get(rg.get('b1_addr', 'DB000000'), False)
                        left_on = c1_on or b1_on
                    else:
                        left_on = c1_on
                    c2_v = plc_mem.get(rg.get('c2_addr', 'IB0C40B'), False)
                    c2_on = (not c2_v) if rg.get('c2_type') == 'NC' else c2_v
                    coil_addr = rg.get('coil_addr', '')
                    if coil_addr:
                        plc_mem[coil_addr] = bool(left_on and c2_on)
                self.belt1_running = bool(plc_mem.get('OB0C400', False))
                self.do_01_in1 = bool(plc_mem.get('OB0C401', False))
                self.do_02_in2 = bool(plc_mem.get('OB0C402', False))
                self.belt2_running = bool(plc_mem.get('OB0C403', False))
            else:
                self.belt1_running = (self.p1_state != 'CONV1_READY')
                self.belt2_running = (self.p1_state != 'CONV2_DONE')
                self.do_01_in1 = self.di_11_conv1
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
                self.status_msg = ' Phôi thành phẩm đã tới Cảm biến S4 cuối Băng tải 2 vuông góc (X=920mm, Y=380mm)!'
            self._check_pickup(tcp1, tcp2)

        elif self.p1_state == 'CONV2_DONE':
            self._check_pickup(tcp1, tcp2)
            if self.auto_drop_enabled and not self.coop_running:
                self.p1_done_timer += self.dt
                if self.p1_done_timer >= 2.2:
                    self.trigger_new_part_conv1()

        # -----------------------------------------------------------------
        # 4. CẬP NHẬT TÍN HIỆU CẢM BIẾN TIỆM CẬN S1..S4 & CHẠY CODE TẠO TỌA ĐỘ CỦA NGƯỜI DÙNG
        # -----------------------------------------------------------------
        self.di_11_conv1 = (self.p1_state in ('CONV1_MOVING', 'CONV1_READY') and abs(self.p1_y - self.s2_pick_y) < 0.022)
        self.di_11_conv2 = (self.p1_state in ('CONV2_MOVING', 'CONV2_DONE') and abs(self.p1_x - self.s4_exit_x) < 0.025)
        if not self.compiled_ladder_rungs or self.conveyor_mode != 'STOP_AT_SENSOR':
            self.do_01_in1 = self.di_11_conv1

        self.s1_sensor = bool(
            self.p1_state in ('CONV1_MOVING', 'CONV1_READY')
            and (self.s1_trig_y <= self.p1_y <= self.s1_trig_y + 0.025)
        )
        self.s2_sensor = bool(self.di_11_conv1)
        self.s3_sensor = bool(
            self.p1_state in ('CONV2_MOVING', 'CONV2_DONE')
            and (self.s3_trig_x <= self.p1_x <= self.s3_trig_x + 0.025)
        )
        self.s4_sensor = bool(self.di_11_conv2)

        prev_est_x, prev_est_y, prev_yaw = self.est_x, self.est_y, self.obj_yaw
        if self.user_coord_compiled is not None:
            loc = {
                'math': math,
                'enc1_pulses': int(self.enc1_pulses),
                'enc2_pulses': int(self.enc2_pulses),
                's1_sensor': self.s1_sensor,
                's2_sensor': self.s2_sensor,
                's3_sensor': self.s3_sensor,
                's4_sensor': self.s4_sensor,
                'on_rotary': bool(self.p1_state == 'ON_ROTARY'),
                'rotary_deg': float(self.rotary_angle_deg),
                'pulses_per_mm': float(self.pulses_per_mm),
                'mem': self.user_mem,
                'x_obj': float(self.est_x * 1000.0),
                'y_obj': float(self.est_y * 1000.0),
                'z_obj': float(self.est_z * 1000.0),
                'yaw_obj': float(math.degrees(self.obj_yaw)),
            }
            try:
                exec(self.user_coord_compiled, {'math': math, '__builtins__': __builtins__}, loc)
                self.est_x = float(loc.get('x_obj', 220.0)) / 1000.0
                self.est_y = float(loc.get('y_obj', -240.0)) / 1000.0
                self.est_z = float(loc.get('z_obj', 40.0)) / 1000.0
                self.obj_yaw = math.radians(float(loc.get('yaw_obj', 0.0)))
                self.user_code_error = 'OK'
                self.active_enc_name = f"USER_CODE (stage={self.user_mem.get('stage', 'CONV1')})"
            except Exception as e:
                self.user_code_error = f'Lỗi Runtime: {e}'

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

        # 5. TÍNH VẬN TỐC FEEDFORWARD TỪ TỌA ĐỘ CODE CỦA NGƯỜI DÙNG & XUẤT HỆ USER [OBJECT] (50Hz ZERO-LAG)
        vx_est = (self.est_x - prev_est_x) / self.dt
        vy_est = (self.est_y - prev_est_y) / self.dt
        wz_est = (self.obj_yaw - prev_yaw) / self.dt
        self.obj_vx = vx_est if abs(vx_est) < 0.5 else 0.0
        self.obj_vy = vy_est if abs(vy_est) < 0.5 else 0.0
        self.obj_wz = wz_est if abs(wz_est) < 4.0 else 0.0

        uf_msg = Float64MultiArray()
        uf_msg.data = [
            float(self.est_x),
            float(self.est_y),
            float(self.est_z),
            float(self.obj_yaw),
            float(self.obj_vx),
            float(self.obj_vy),
            float(self.obj_wz),
        ]
        self.user_frame_pub.publish(uf_msg)

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
            # Tư thế R1 đặt lên Đĩa xoay Trục A tại góc giao 90° (X=0.22, Y=0.215)
            r1_p004_rot_above = [0.775, 0.540, 0.220, 0.000, 0.320, 0.000]
            r1_p005_rot_place = [0.775, 0.720, 0.050, 0.000, 0.670, 0.000]

            # Tư thế R2 (gốc X=0.58, Y=0.14, xoay +90° hướng +Y) gắp từ Đĩa xoay sau khi xoay 90° (X=0.265, Y=0.26)
            r2_rot_above = [1.200, 0.540, 0.220, 0.000, 0.320, 0.000]
            r2_rot_pick  = [1.200, 0.720, 0.050, 0.000, 0.670, 0.000]
            # Tư thế R2 đặt sang đầu Băng tải 2 vuông góc (X=0.40, Y=0.26)
            r2_conv2_above = [0.980, 0.460, 0.170, 0.000, 0.290, 0.000]
            r2_conv2_place = [0.980, 0.660, -0.010, 0.000, 0.670, 0.000]

            try:
                self.mc_start = True
                if self.p1_state not in ('CONV1_MOVING', 'CONV1_READY', 'DROPPING'):
                    self.trigger_new_part_conv1()

                self.status_msg = '● [Bài TN 2 - Bước 1] Đang chờ phôi trên Băng tải 1 chạm cảm biến DI_11 -> Bật DO_01 (IN#1)...'
                t_wait = 0.0
                while self.p1_state != 'CONV1_READY' and t_wait < 8.0:
                    time.sleep(0.05)
                    t_wait += 0.05

                self.last_yerc_packet = (
                    'TX [UDP:10040 -> R1] "YERC" | Cmd:0x86 (Start JOB:LAB2-R1) | IN#1=ON, IN#2=OFF'
                )
                self.status_msg = ' [Bài TN 2 - Bước 2] IN#1=ON & IN#2=OFF -> Robot 1 (R1) đến P001 -> P002 gắp vật trên Băng tải 1...'
                self.send_traj(self.r1_client, r1_j, r1_p001_above, 1.0)
                self.send_traj(self.r1_client, r1_j, r1_p002_pick, 0.85)
                self.set_vac_r1(True)
                time.sleep(0.3)
                self.send_traj(self.r1_client, r1_j, r1_p001_above, 0.9)

                self.status_msg = ' [Bài TN 2 - Bước 3 & 4] R1 đưa phôi qua điểm trung gian P003 -> P004 -> Đặt xuống Đĩa xoay P005...'
                self.send_traj(self.r1_client, r1_j, r1_p004_rot_above, 1.1)
                self.send_traj(self.r1_client, r1_j, r1_p005_rot_place, 0.85)
                # Gán tọa độ phôi tại mép dưới Đĩa xoay (-90° so với tâm đĩa xoay X=0.22, Y=0.26)
                self.p1_x, self.p1_y = 0.22, 0.215
                self.set_vac_r1(False)
                self.p1_state = 'ON_ROTARY'
                self.p1_rotary_offset_angle = (-math.pi / 2.0) - math.radians(self.rotary_angle_deg)
                time.sleep(0.3)
                self.send_traj(self.r1_client, r1_j, r1_p004_rot_above, 0.85)
                self.send_traj(self.r1_client, r1_j, home_q, 1.0)

                # Bước 4 (Trang 8 Bài TN 2): R1 phát xung OT#3 (100ms) báo cho Machine Controller xoay mâm 90 độ
                self.status_msg = ' [Bài TN 2 & 3] R1 phát xung PULSE OT#(3) T=0.10s -> MP3300 nhận DI_12 kích Mâm xoay A1 quay 90°!'
                self.trigger_rotary_90deg()
                time.sleep(1.3)

                # Mâm xoay đã quay 90° đưa phôi sang hướng +X (X=0.265, Y=0.28) -> Robot 2 gắp đặt sang Băng tải 2 vuông góc
                self.last_yerc_packet = (
                    'TX [UDP:10040 -> R2] "YERC" | Cmd:0x86 (Start JOB:LAB3-R2) | DO_02 (IN#2)=ON'
                )
                self.status_msg = ' [Phối hợp R2] Mâm xoay đã quay 90° -> Robot 2 (R2) gắp phôi đặt sang Băng tải 2 vuông góc...'
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

                self.status_msg = ' [Băng tải 2 Vuông góc + Encoder 2] Phôi đang chạy dọc trục +X về cuối Băng tải 2...'
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

        # 3. Vạch chữ thập đen (+) chia 4 ô trên Mâm xoay Trục A (Giống Hình 3 Bài TN 2 & 3) quay theo góc rotary_angle_deg
        for cid, yaw_offset in [(8, 0.0), (9, 90.0)]:
            cross = Marker()
            cross.header.frame_id = 'world'
            cross.header.stamp = now
            cross.ns = 'workcell'
            cross.id = cid
            cross.type = Marker.CUBE
            cross.action = Marker.ADD
            cross.pose.position.x = self.rotary_x
            cross.pose.position.y = self.rotary_y
            cross.pose.position.z = 0.003
            half_yaw = math.radians(self.rotary_angle_deg + yaw_offset) * 0.5
            cross.pose.orientation.z = math.sin(half_yaw)
            cross.pose.orientation.w = math.cos(half_yaw)
            cross.scale.x = 0.16
            cross.scale.y = 0.018
            cross.scale.z = 0.003
            cross.color.r, cross.color.g, cross.color.b, cross.color.a = 0.12, 0.12, 0.15, 1.0
            ma.markers.append(cross)

        # 4. Vẽ Hệ Trục Tọa Độ USER [OBJECT] (X-Đỏ, Y-Xanh lá, Z-Xanh dương) theo TỌA ĐỘ DO CODE CỦA NGƯỜI DÙNG TÍNH (est_x, est_y, est_z)
        ox = float(self.est_x)
        oy = float(self.est_y)
        oz = float(max(0.02, self.est_z) + 0.002)
        cy = math.cos(self.obj_yaw)
        sy = math.sin(self.obj_yaw)
        axis_len = 0.055
        for aid, dx, dy, dz, rgb in [
            (10, axis_len * cy, axis_len * sy, 0.0, (1.0, 0.15, 0.15)),   # X_user (Đỏ)
            (11, -axis_len * sy, axis_len * cy, 0.0, (0.15, 1.0, 0.25)),  # Y_user (Xanh lá)
            (12, 0.0, 0.0, axis_len, (0.20, 0.55, 1.0)),                  # Z_user (Xanh dương)
        ]:
            arr = Marker()
            arr.header.frame_id = 'world'
            arr.header.stamp = now
            arr.ns = 'workcell'
            arr.id = aid
            arr.type = Marker.ARROW
            arr.action = Marker.ADD
            p_start = Point(x=ox, y=oy, z=oz)
            p_end = Point(x=ox + dx, y=oy + dy, z=oz + dz)
            arr.points = [p_start, p_end]
            arr.scale.x = 0.004
            arr.scale.y = 0.008
            arr.scale.z = 0.010
            arr.color.r, arr.color.g, arr.color.b, arr.color.a = rgb[0], rgb[1], rgb[2], 1.0
            ma.markers.append(arr)

        # Nhãn chữ 3D "USER [UF#1]" bay theo Đối tượng
        txt_m = Marker()
        txt_m.header.frame_id = 'world'
        txt_m.header.stamp = now
        txt_m.ns = 'workcell'
        txt_m.id = 13
        txt_m.type = Marker.TEXT_VIEW_FACING
        txt_m.action = Marker.ADD
        txt_m.pose.position.x = ox
        txt_m.pose.position.y = oy
        txt_m.pose.position.z = oz + 0.068
        txt_m.scale.z = 0.018
        txt_m.color.r, txt_m.color.g, txt_m.color.b, txt_m.color.a = 1.0, 0.92, 0.23, 1.0
        txt_m.text = 'USER [OBJECT]'
        ma.markers.append(txt_m)

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
        win_w = min(980, max(860, scr_w // 2))
        win_h = min(740, max(620, scr_h - 80))
        pos_y = max(20, scr_h - win_h - 45)
        self.root.geometry(f'{win_w}x{win_h}+0+{pos_y}')
        self.root.configure(bg='#1e2229')

        # Nạp nội dung 3 file PLC / Motion từ thư mục src/motomini_control/plc/
        self.plc_dir = os.path.abspath(
            os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'plc')
        )
        self.plc_files = {
            'USER_COORD': os.path.join(self.plc_dir, 'user_object_coord_calc.py'),
            'H01_LADDER': os.path.join(self.plc_dir, 'H01_ladder_dual_conveyor.plc'),
            'MPM001_ZRN': os.path.join(self.plc_dir, 'MPM001_servo_zrn.mpm'),
            'MPM002_ROT90': os.path.join(self.plc_dir, 'MPM002_rotary_90deg.mpm'),
        }
        self.plc_codes = {}
        for k, fpath in self.plc_files.items():
            if os.path.exists(fpath):
                with open(fpath, 'r', encoding='utf-8') as f:
                    self.plc_codes[k] = f.read()
            else:
                self.plc_codes[k] = f'// File {fpath}'
        self.active_plc_tab = 'USER_COORD'

        self._build_ui()
        self.root.after(80, self._update_ui)

    def _build_ui(self):
        hdr = tk.Frame(self.root, bg='#0b7285', padx=8, pady=4)
        hdr.pack(fill=tk.X)
        tk.Label(
            hdr,
            text='MP3300 MACHINE CONTROLLER + 2 BĂNG TẢI ENCODER + 2 ROBOT MOTOMINI (BÀI TN 2, 3, 4)',
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
            text='1. ĐO LƯỜNG 2 ENCODER BĂNG TẢI & TỌA ĐỘ VẬT THỂ ',
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
            text='2. GIAO TIẾP I/O MP3300 & SERVO MÂM XOAY A1 (BÀI TN 2 & 3) ',
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
            text='■ DI_01: STOP',
            bg='#c92a2a',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            command=lambda: setattr(self.node, 'mc_start', False),
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        tk.Button(
            mp_btns,
            text='ZRN[A1]0 (Về Zero)',
            bg='#1864ab',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            command=self.node.home_rotary_axis_a1,
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        tk.Button(
            mp_btns,
            text='Xoay A1 +90° (DI_12)',
            bg='#e67700',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            command=self.node.trigger_rotary_90deg,
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        # --- KHUNG 3 (TRÁI DƯỚI): BÀI THÍ NGHIỆM 4 - HIGH SPEED ETHERNET (YERC UDP:10040) & XUNG KHỚP ---
        f_eth = tk.LabelFrame(
            left_col,
            text='3. BÀI TN 4: HIGH-SPEED ETHERNET (UDP:10040) & XUNG KHỚP (PULSE) ',
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
            text='4. CHẾ ĐỘ CHẠY 2 BĂNG TẢI & AUTO-DROP PHÔI ',
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
            text='AUTO-DROP: BẬT',
            bg='#2b8a3e',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            command=self._toggle_autodrop,
        )
        self.btn_ad.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        tk.Button(
            drop_row,
            text='Thả Phôi Xuống Băng Tải 1',
            bg='#e67700',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            command=self.node.trigger_new_part_conv1,
        ).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=2)

        # --- KHUNG 5 (PHẢI DƯỚI): PHẦN MỀM YASKAWA MPE720 Ver.7 - SƠ ĐỒ LADDER ĐỒ HỌA & MOTION ---
        f_coop = tk.LabelFrame(
            right_col,
            text='5. PHẦN MỀM YASKAWA MPE720 Ver.7: SƠ ĐỒ LADDER (H:Main) & MOTION (MPM001/002) ',
            bg='#282c34',
            fg='#ffa94d',
            font=('Ubuntu', 8, 'bold'),
            padx=6,
            pady=4,
        )
        f_coop.pack(fill=tk.BOTH, expand=True)

        # Dữ liệu cấu trúc các Rung của Sơ đồ Ladder H:Main Program (chuẩn Trang 3, 6, 7, 8 Phụ lục MPE720)
        self.selected_rung_idx = 0
        self.ladder_rungs = self._default_ladder_rungs()
        self.node.compiled_ladder_rungs = [dict(r) for r in self.ladder_rungs]

        # Thanh chọn Tab: [USER_COORD] | [WIRING_DIAGRAM] | [H01_LADDER] | [MPM001_ZRN] | [MPM002_ROT90]
        tab_bar = tk.Frame(f_coop, bg='#282c34')
        tab_bar.pack(fill=tk.X, pady=(0, 2))
        self.plc_tab_btns = {}
        for tid, tlabel in [
            ('USER_COORD', 'Code Tạo Tọa Độ Vật (.py)'),
            ('WIRING_DIAGRAM', 'Sơ Đồ Đấu Dây I/O'),
            ('H01_LADDER', 'H: Main Ladder'),
            ('MPM001_ZRN', 'MPM001: ZRN (.mpm)'),
            ('MPM002_ROT90', 'MPM002: +90° (.mpm)'),
        ]:
            b = tk.Button(
                tab_bar,
                text=tlabel,
                font=('Ubuntu', 8, 'bold'),
                command=lambda k=tid: self._switch_plc_tab(k),
            )
            b.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
            self.plc_tab_btns[tid] = b

        # =================================================================
        # PANEL 0: SƠ ĐỒ ĐẤU DÂY PHẦN CỨNG & BẢNG ĐỊA CHỈ I/O MP3300 (HÌNH 1 & BẢNG 1 BÀI TN 2)
        # =================================================================
        self.wiring_panel = tk.Frame(f_coop, bg='#282c34')
        w_hdr = tk.Label(
            self.wiring_panel,
            text='SƠ ĐỒ ĐẤU NỐI CHÂN I/O GIỮA CONTROL PANEL, 2 BĂNG TẢI, ĐĨA XOAY, MP3300 VÀ 2 TỦ ROBOT YRC1000micro (Dây sáng XANH khi ON)',
            bg='#1864ab',
            fg='white',
            font=('Ubuntu', 7, 'bold'),
            pady=2,
        )
        w_hdr.pack(fill=tk.X)
        self.wiring_canvas = tk.Canvas(self.wiring_panel, bg='#14171c', height=235, highlightthickness=0)
        self.wiring_canvas.pack(fill=tk.BOTH, expand=True)

        # =================================================================
        # PANEL A: SƠ ĐỒ LADDER ĐỒ HỌA MPE720 + THANH CÔNG CỤ KÝ HIỆU (TRANG 2, 3, 7, 8 PHỤ LỤC)
        # =================================================================
        self.ladder_panel = tk.Frame(f_coop, bg='#282c34')

        # Thanh công cụ các lệnh Ladder chuẩn MPE720 (Trang 2 Phụ lục: NO, NC, Cạnh lên, Coil, Branch, MSEE)
        lad_tb = tk.Frame(self.ladder_panel, bg='#dee2e6', bd=1, relief=tk.RAISED, padx=2, pady=2)
        lad_tb.pack(fill=tk.X, pady=(0, 2))

        for sym_txt, sym_cmd in [
            ('|--| |-- NO', lambda: self._edit_selected_rung('NO')),
            ('|--|/|-- NC', lambda: self._edit_selected_rung('NC')),
            ('|--|^|-- Cạnh lên', lambda: self._edit_selected_rung('PULSE_UP')),
            ('|--( )-- Coil', lambda: self._edit_selected_rung('COIL')),
            ('[|| Nhánh Tự Giữ]', lambda: self._edit_selected_rung('BRANCH')),
            ('[MSEE Gọi Motion]', lambda: self._edit_selected_rung('MSEE')),
            ('+ Thêm Rung', self._insert_new_rung),
            ('- Xóa Rung', self._delete_selected_rung),
        ]:
            tk.Button(
                lad_tb,
                text=sym_txt,
                bg='#f8f9fa',
                fg='#212529',
                font=('Ubuntu Mono', 7, 'bold'),
                padx=3,
                pady=1,
                command=sym_cmd,
            ).pack(side=tk.LEFT, padx=1)

        # Hàng chọn nhanh Địa chỉ I/O & Thanh ghi gán vào Rung đang chọn
        addr_bar = tk.Frame(self.ladder_panel, bg='#343a40', padx=4, pady=2)
        addr_bar.pack(fill=tk.X, pady=(0, 2))
        tk.Label(
            addr_bar, text='Gán địa chỉ I/O vào Rung:', bg='#343a40', fg='#ffd43b', font=('Ubuntu', 7, 'bold')
        ).pack(side=tk.LEFT)

        self.io_addr_options = [
            'IB0C400 (DI_00: Start)',
            'IB0C401 (DI_01: Stop)',
            'IB0C402 (DI_02: Servo_ON)',
            'IB0C40B (DI_11: Cảm biến Đai 1)',
            'IB0C40C (DI_12: Xung R1 OT#3)',
            'IB0C40D (DI_13: Cảm biến Đai 2)',
            'DB000000 (VAR_Start: Cờ chạy)',
            'OB0C400 (DO_00: Biến tần Đai 1)',
            'OB0C401 (DO_01: Báo R1 IN#1)',
            'OB0C402 (DO_02: Báo R2 IN#1)',
            'OB0C403 (DO_03: Biến tần Đai 2)',
            'MB001010 (MOTION_002.Run)',
        ]
        self.selected_io_var = tk.StringVar(value=self.io_addr_options[0])
        io_menu = ttk.Combobox(
            addr_bar,
            textvariable=self.selected_io_var,
            values=self.io_addr_options,
            width=28,
            state='readonly',
            font=('Ubuntu Mono', 7),
        )
        io_menu.pack(side=tk.LEFT, padx=4)

        tk.Button(
            addr_bar,
            text='Gán vào Tiếp Điểm',
            bg='#1864ab',
            fg='white',
            font=('Ubuntu', 7, 'bold'),
            padx=4,
            command=lambda: self._assign_io_to_rung(target='contact'),
        ).pack(side=tk.LEFT, padx=2)

        tk.Button(
            addr_bar,
            text='Gán vào Coil ( )',
            bg='#d9480f',
            fg='white',
            font=('Ubuntu', 7, 'bold'),
            padx=4,
            command=lambda: self._assign_io_to_rung(target='coil'),
        ).pack(side=tk.LEFT, padx=2)

        tk.Button(
            addr_bar,
            text='Khôi phục mẫu Bài 2&3',
            bg='#495057',
            fg='white',
            font=('Ubuntu', 7, 'bold'),
            padx=4,
            command=self._reset_default_ladder,
        ).pack(side=tk.RIGHT, padx=2)

        # Canvas vẽ Sơ đồ Ladder Đồ Họa chuẩn MPE720 (Nền trắng, có thanh cuộn dọc)
        canv_wrap = tk.Frame(self.ladder_panel, bg='white', bd=2, relief=tk.SUNKEN)
        canv_wrap.pack(fill=tk.BOTH, expand=True)
        self.ladder_canvas = tk.Canvas(canv_wrap, bg='#ffffff', height=195, highlightthickness=0)
        lad_sb = tk.Scrollbar(canv_wrap, orient=tk.VERTICAL, command=self.ladder_canvas.yview)
        self.ladder_canvas.configure(yscrollcommand=lad_sb.set)
        lad_sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.ladder_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.ladder_canvas.bind('<Button-1>', self._on_canvas_click)

        # =================================================================
        # PANEL B: TRÌNH SOẠN THẢO CODE TẠO TỌA ĐỘ VẬT (.py) & MOTION (MPM001 / MPM002)
        # =================================================================
        self.motion_panel = tk.Frame(f_coop, bg='#282c34')

        # Thanh công cụ cho Code Tạo Tọa Độ Vật Thể (Encoder + Sensor) & Lệnh Motion
        mot_tb = tk.Frame(self.motion_panel, bg='#343a40', padx=2, pady=2)
        mot_tb.pack(fill=tk.X, pady=(0, 2))
        tk.Button(
            mot_tb,
            text='[1. Bài Tập Tự Code (Khung Sườn)]',
            bg='#d9480f',
            fg='white',
            font=('Ubuntu', 7, 'bold'),
            padx=4,
            pady=1,
            command=self._load_user_coord_skeleton,
        ).pack(side=tk.LEFT, padx=2)
        tk.Button(
            mot_tb,
            text='[2. Lời Giải Mẫu Chuẩn]',
            bg='#0ca678',
            fg='white',
            font=('Ubuntu', 7, 'bold'),
            padx=4,
            pady=1,
            command=self._load_user_coord_solution,
        ).pack(side=tk.LEFT, padx=2)

        for m_lbl, m_snip in [
            ('+ ZRN[A1]0;', '    ZRN[A1]0;    // Về vị trí Home 0.0 deg cho trục [A1]\n'),
            ('+ MOV[A1]900;', '    MOV[A1]900;  // Quay trục [A1] thêm +90.0 độ (900 * 0.1 deg)\n'),
            ('+ IOW IB0C40C==1;', '    IOW IB0C40C==1; // Chờ tín hiệu xung DI_12 lên 1\n'),
        ]:
            tk.Button(
                mot_tb,
                text=m_lbl,
                bg='#495057',
                fg='white',
                font=('Ubuntu Mono', 7, 'bold'),
                padx=3,
                pady=1,
                command=lambda s=m_snip: self._insert_motion_snippet(s),
            ).pack(side=tk.LEFT, padx=1)

        editor_wrap = tk.Frame(self.motion_panel, bg='#1b1f24')
        editor_wrap.pack(fill=tk.BOTH, expand=True, pady=1)
        self.txt_plc = tk.Text(
            editor_wrap,
            height=9,
            bg='#ffffff',
            fg='#1864ab',
            insertbackground='black',
            font=('Ubuntu Mono', 9, 'bold'),
            wrap=tk.NONE,
        )
        sb_y = tk.Scrollbar(editor_wrap, orient=tk.VERTICAL, command=self.txt_plc.yview)
        self.txt_plc.configure(yscrollcommand=sb_y.set)
        sb_y.pack(side=tk.RIGHT, fill=tk.Y)
        self.txt_plc.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Hàng nút F4: Compile & Download + Chạy Phối Hợp Bài TN 2 & 3
        act_row = tk.Frame(f_coop, bg='#282c34')
        act_row.pack(fill=tk.X, pady=(3, 0))
        tk.Button(
            act_row,
            text='▶ [F4: COMPILE & DOWNLOAD] Nạp Code Tọa Độ Vật / PLC xuống Hệ Thống',
            bg='#2b8a3e',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            pady=4,
            command=self._compile_and_download_plc,
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

        tk.Button(
            act_row,
            text='▶ CHẠY CHU TRÌNH BÀI TN 2 & 3 (PLC + R1 + R2)',
            bg='#1864ab',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            pady=4,
            command=self.node.run_full_lab2_lab3_dual_cycle,
        ).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=1)

        self.active_plc_tab = None
        self._switch_plc_tab('USER_COORD')

    def _default_ladder_rungs(self):
        return [
            {
                'header': 'Conveyor Control and Trigger Robot (Start/Stop Self-Hold - Trang 3 Phụ lục)',
                'type': 'BRANCH_OR',
                'c1_type': 'NO', 'c1_addr': 'IB0C400', 'c1_var': 'LIO_DI_00', 'c1_cmt': 'Input0 (Start)',
                'b1_addr': 'DB000000', 'b1_var': 'VAR_Start',
                'c2_type': 'NC', 'c2_addr': 'IB0C401', 'c2_var': 'LIO_DI_01', 'c2_cmt': 'Input1 (Stop)',
                'coil_addr': 'DB000000', 'coil_var': 'VAR_Start', 'coil_cmt': 'System Run',
            },
            {
                'header': 'Băng tải 1 chạy khi Hệ thống RUN và Cảm biến DI_11 chưa có phôi (Bài TN 2 Mục 1.1)',
                'type': 'SERIES',
                'c1_type': 'NO', 'c1_addr': 'DB000000', 'c1_var': 'VAR_Start', 'c1_cmt': 'System Run',
                'c2_type': 'NC', 'c2_addr': 'IB0C40B', 'c2_var': 'DI_11_Sens1', 'c2_cmt': 'Sensor Đai 1',
                'coil_addr': 'OB0C400', 'coil_var': 'DO_00_Conv1', 'coil_cmt': 'Inverter Đai 1',
            },
            {
                'header': 'Khi phôi chạm Cảm biến DI_11 -> Dừng Đai 1 & Bật DO_01 báo Robot 1 (IN#1=ON)',
                'type': 'SERIES',
                'c1_type': 'NO', 'c1_addr': 'DB000000', 'c1_var': 'VAR_Start', 'c1_cmt': 'System Run',
                'c2_type': 'NO', 'c2_addr': 'IB0C40B', 'c2_var': 'DI_11_Sens1', 'c2_cmt': 'Sensor Đai 1',
                'coil_addr': 'OB0C401', 'coil_var': 'DO_01_R1_IN1', 'coil_cmt': 'Trigger R1 Pick',
            },
            {
                'header': 'INPUT PROCESSING: Nhận xung OT#3 (DI_12 cạnh lên) từ Robot 1 (Trang 7 Phụ lục)',
                'type': 'SERIES',
                'c1_type': 'NO', 'c1_addr': 'IB0C402', 'c1_var': 'DI_02_SvON', 'c1_cmt': 'Servo ON',
                'c2_type': 'PULSE_UP', 'c2_addr': 'IB0C40C', 'c2_var': 'DI_12_OT3', 'c2_cmt': 'R1 Pulse OT#3',
                'coil_addr': 'MB001010', 'coil_var': 'MOTION_002.Run', 'coil_cmt': 'Start request',
            },
            {
                'header': 'CALL MOTION PROGRAM: Gọi MPM002 quay Đĩa xoay Trục A1 +90° (Trang 6 & 8 Phụ lục)',
                'type': 'MSEE',
                'prog_no': '00002', 'struct_addr': 'MA00100', 'struct_name': 'MOTION_002',
            },
            {
                'header': 'OUTPUT PROCESSING: Đĩa xoay đã đưa phôi sang hướng +X -> Bật DO_02 báo Robot 2 (IN#1)',
                'type': 'SERIES',
                'c1_type': 'NO', 'c1_addr': 'DB000000', 'c1_var': 'VAR_Start', 'c1_cmt': 'System Run',
                'c2_type': 'NO', 'c2_addr': 'MB001002', 'c2_var': 'ROT90_Done', 'c2_cmt': 'Phôi tại Đĩa xoay',
                'coil_addr': 'OB0C402', 'coil_var': 'DO_02_R2_IN1', 'coil_cmt': 'Trigger R2 Pick',
            },
            {
                'header': 'Chạy Băng tải 2 vuông góc (+X) đưa phôi thành phẩm về Cảm biến S4 (DI_13)',
                'type': 'SERIES',
                'c1_type': 'NO', 'c1_addr': 'DB000000', 'c1_var': 'VAR_Start', 'c1_cmt': 'System Run',
                'c2_type': 'NC', 'c2_addr': 'IB0C40D', 'c2_var': 'DI_13_Sens4', 'c2_cmt': 'Cuối Đai 2',
                'coil_addr': 'OB0C403', 'coil_var': 'DO_03_Conv2', 'coil_cmt': 'Inverter Đai 2',
            },
        ]

    def _eval_addr_state(self, addr: str) -> bool:
        table = {
            'IB0C400': self.node.mc_start,
            'IB0C401': not self.node.mc_start,
            'IB0C402': self.node.mc_servo_on,
            'IB0C40B': self.node.di_11_conv1,
            'IB0C40C': self.node.di_12_ot3,
            'IB0C40D': self.node.di_11_conv2,
            'DB000000': self.node.mc_start,
            'OB0C400': self.node.belt1_running,
            'OB0C401': self.node.do_01_in1,
            'OB0C402': self.node.do_02_in2,
            'OB0C403': self.node.belt2_running,
            'MB001010': self.node.di_12_ot3,
            'MB001002': self.node.do_02_in2,
        }
        return table.get(addr, False)

    def _eval_contact(self, ctype: str, addr: str) -> bool:
        val = self._eval_addr_state(addr)
        if ctype == 'NC':
            return not val
        return val

    def _draw_ladder_canvas(self):
        if not hasattr(self, 'ladder_canvas'):
            return
        c = self.ladder_canvas
        c.delete('all')
        w = max(460, c.winfo_width())
        rung_h = 74
        total_h = (len(self.ladder_rungs) + 1) * rung_h + 20
        c.configure(scrollregion=(0, 0, w, total_h))

        left_rail_x = 34
        right_rail_x = w - 18

        for idx, rg in enumerate(self.ladder_rungs):
            y0 = idx * rung_h + 4
            y_mid = y0 + 40
            is_sel = (idx == self.selected_rung_idx)

            # Nền Rung đang chọn (viền xanh dương nhạt chuẩn MPE720)
            if is_sel:
                c.create_rectangle(2, y0, w - 4, y0 + rung_h - 2, fill='#eef6ff', outline='#339af0', width=1)

            # Thanh tiêu đề xanh xám phía trên mỗi Rung (Chuẩn Hình trang 3, 7, 8 Phụ lục)
            c.create_rectangle(left_rail_x, y0 + 2, right_rail_x, y0 + 16, fill='#6c757d', outline='#495057')
            c.create_text(
                (left_rail_x + right_rail_x) // 2,
                y0 + 9,
                text=rg.get('header', f'Rung {idx}'),
                fill='white',
                font=('Ubuntu', 7, 'bold'),
            )

            # Cột số thứ tự Rung bên trái (Ô đen chữ trắng chuẩn MPE720)
            c.create_rectangle(4, y0 + 22, left_rail_x - 2, y0 + 38, fill='#212529', outline='black')
            c.create_text(18, y0 + 30, text=str(idx), fill='white', font=('Ubuntu Mono', 8, 'bold'))

            # Đường nguồn dọc trái & phải
            c.create_line(left_rail_x, y0 + 16, left_rail_x, y0 + rung_h, fill='#212529', width=2)
            c.create_line(right_rail_x, y0 + 16, right_rail_x, y0 + rung_h, fill='#212529', width=2)

            rtype = rg.get('type', 'SERIES')
            if rtype == 'MSEE':
                # Khối hàm MSEE gọi chương trình Motion (Hình trang 6 & 8 Phụ lục)
                active = self.node.di_12_ot3 or abs(self.node.rotary_target_deg - self.node.rotary_angle_deg) > 0.5
                wire_col = '#00b4d8' if active else '#495057'
                bx1 = left_rail_x + 140
                bx2 = bx1 + 64
                c.create_line(left_rail_x, y_mid, bx1, y_mid, fill=wire_col, width=2)
                c.create_rectangle(bx1, y0 + 20, bx2, y0 + 66, fill='#c3fae8', outline='#0ca678', width=2)
                c.create_text((bx1 + bx2) // 2, y_mid, text='MSEE', fill='#087f5b', font=('Ubuntu Mono', 9, 'bold'))
                c.create_text(
                    bx2 + 52,
                    y0 + 28,
                    text=f"[W]Program  [A]Data\n  {rg.get('prog_no','00002')}     {rg.get('struct_name','MOTION_002')}\n            {rg.get('struct_addr','MA00100')}",
                    fill='#1864ab',
                    font=('Ubuntu Mono', 7, 'bold'),
                    justify=tk.LEFT,
                )
                c.create_line(bx2, y_mid, right_rail_x, y_mid, fill=wire_col, width=2)
            else:
                c1_on = self._eval_contact(rg.get('c1_type', 'NO'), rg.get('c1_addr', 'IB0C400'))
                if rtype == 'BRANCH_OR':
                    b1_on = self._eval_contact('NO', rg.get('b1_addr', 'DB000000'))
                    left_on = c1_on or b1_on
                else:
                    b1_on = False
                    left_on = c1_on
                c2_on = self._eval_contact(rg.get('c2_type', 'NO'), rg.get('c2_addr', 'IB0C40B'))
                rung_on = left_on and c2_on

                x_c1 = left_rail_x + 65
                x_c2 = left_rail_x + 185
                x_coil = right_rail_x - 55

                # Dây ngang chính
                c.create_line(left_rail_x, y_mid, x_c1 - 12, y_mid, fill='#00c853' if c1_on else '#495057', width=2)
                self._draw_contact_symbol(
                    c, x_c1, y_mid, rg.get('c1_type', 'NO'),
                    rg.get('c1_var', ''), rg.get('c1_addr', ''), rg.get('c1_cmt', ''), c1_on
                )

                # Nhánh song song tự giữ (nếu là BRANCH_OR - Hình trang 3 Phụ lục)
                if rtype == 'BRANCH_OR':
                    y_br = y_mid + 22
                    br_col = '#00c853' if b1_on else '#495057'
                    c.create_line(left_rail_x + 18, y_mid, left_rail_x + 18, y_br, fill=br_col, width=2)
                    c.create_line(left_rail_x + 18, y_br, x_c1 - 12, y_br, fill=br_col, width=2)
                    self._draw_contact_symbol(
                        c, x_c1, y_br, 'NO', '', rg.get('b1_addr', 'DB000000'), '', b1_on, compact=True
                    )
                    c.create_line(x_c1 + 12, y_br, x_c1 + 36, y_br, fill=br_col, width=2)
                    c.create_line(x_c1 + 36, y_br, x_c1 + 36, y_mid, fill=br_col, width=2)

                c.create_line(x_c1 + 12, y_mid, x_c2 - 12, y_mid, fill='#00c853' if left_on else '#495057', width=2)
                self._draw_contact_symbol(
                    c, x_c2, y_mid, rg.get('c2_type', 'NO'),
                    rg.get('c2_var', ''), rg.get('c2_addr', ''), rg.get('c2_cmt', ''), c2_on
                )
                c.create_line(x_c2 + 12, y_mid, x_coil - 14, y_mid, fill='#00c853' if rung_on else '#495057', width=2)

                # Cuộn dây ngõ ra (Coil) bên phải
                self._draw_coil_symbol(
                    c, x_coil, y_mid, rg.get('coil_var', ''), rg.get('coil_addr', ''), rg.get('coil_cmt', ''), rung_on
                )
                c.create_line(x_coil + 14, y_mid, right_rail_x, y_mid, fill='#00c853' if rung_on else '#495057', width=2)

        # Rung cuối cùng: Khối [END] chuẩn MPE720 (Hình trang 2 & 8 Phụ lục)
        y_end = len(self.ladder_rungs) * rung_h + 18
        c.create_rectangle(4, y_end - 8, left_rail_x - 2, y_end + 8, fill='#495057', outline='black')
        c.create_text(18, y_end, text=str(len(self.ladder_rungs)), fill='white', font=('Ubuntu Mono', 8, 'bold'))
        c.create_line(left_rail_x, y_end, right_rail_x, y_end, fill='#495057', width=2)
        mid_x = (left_rail_x + right_rail_x) // 2
        c.create_rectangle(mid_x - 45, y_end - 8, mid_x + 45, y_end + 8, fill='#dee2e6', outline='#495057')
        c.create_text(mid_x, y_end, text='END', fill='#212529', font=('Ubuntu Mono', 8, 'bold'))

    def _draw_contact_symbol(self, c, x, y, ctype, var_name, addr, comment, is_on, compact=False):
        col = '#00a843' if is_on else '#212529'
        if is_on:
            c.create_rectangle(x - 11, y - 9, x + 11, y + 9, fill='#b2f2bb', outline='')
        # Hai thanh dọc của tiếp điểm | |
        c.create_line(x - 10, y - 8, x - 10, y + 8, fill=col, width=2)
        c.create_line(x + 10, y - 8, x + 10, y + 8, fill=col, width=2)
        if ctype == 'NC':
            c.create_line(x - 7, y + 7, x + 7, y - 7, fill=col, width=2)
        elif ctype == 'PULSE_UP':
            c.create_line(x, y + 7, x, y - 7, fill=col, width=2)
            c.create_line(x - 4, y - 2, x, y - 7, fill=col, width=2)
            c.create_line(x + 4, y - 2, x, y - 7, fill=col, width=2)
        if not compact:
            c.create_text(x, y - 19, text=var_name, fill='#d9480f', font=('Ubuntu Mono', 7, 'bold'))
            c.create_text(x, y - 11, text=addr, fill='#000000', font=('Ubuntu Mono', 7, 'bold'))
            c.create_text(x, y + 14, text=comment, fill='#2b8a3e', font=('Ubuntu', 7))
        else:
            c.create_text(x, y - 11, text=addr, fill='#000000', font=('Ubuntu Mono', 7, 'bold'))

    def _draw_coil_symbol(self, c, x, y, var_name, addr, comment, is_on):
        col = '#00a843' if is_on else '#212529'
        fill_c = '#b2f2bb' if is_on else '#ffffff'
        c.create_oval(x - 10, y - 8, x + 10, y + 8, fill=fill_c, outline=col, width=2)
        c.create_text(x, y - 19, text=var_name, fill='#d9480f', font=('Ubuntu Mono', 7, 'bold'))
        c.create_text(x, y - 11, text=addr, fill='#000000', font=('Ubuntu Mono', 7, 'bold'))
        c.create_text(x, y + 14, text=comment, fill='#2b8a3e', font=('Ubuntu', 7))

    def _on_canvas_click(self, event):
        y_canvas = self.ladder_canvas.canvasy(event.y)
        idx = int(y_canvas // 74)
        if 0 <= idx < len(self.ladder_rungs):
            self.selected_rung_idx = idx
            self._draw_ladder_canvas()

    def _edit_selected_rung(self, action: str):
        if not (0 <= self.selected_rung_idx < len(self.ladder_rungs)):
            return
        rg = self.ladder_rungs[self.selected_rung_idx]
        if action in ('NO', 'NC', 'PULSE_UP'):
            if rg.get('type') == 'MSEE':
                rg['type'] = 'SERIES'
                rg['c1_type'] = 'NO'
                rg['c1_addr'] = 'DB000000'
                rg['c1_var'] = 'VAR_Start'
                rg['c1_cmt'] = 'System Run'
                rg['coil_addr'] = 'OB0C400'
                rg['coil_var'] = 'DO_00'
                rg['coil_cmt'] = 'Output'
            rg['c2_type'] = action
            self.node.status_msg = f'[MPE720 Ladder] Đã đổi tiếp điểm Rung {self.selected_rung_idx} thành kiểu [{action}].'
        elif action == 'BRANCH':
            rg['type'] = 'BRANCH_OR' if rg.get('type') != 'BRANCH_OR' else 'SERIES'
            rg.setdefault('b1_addr', rg.get('coil_addr', 'DB000000'))
            rg.setdefault('b1_var', rg.get('coil_var', 'VAR_Hold'))
            self.node.status_msg = f'[MPE720 Ladder] Đã bật/tắt nhánh song song tự giữ tại Rung {self.selected_rung_idx}.'
        elif action == 'MSEE':
            rg['type'] = 'MSEE'
            rg['prog_no'] = '00002'
            rg['struct_addr'] = 'MA00100'
            rg['struct_name'] = 'MOTION_002'
            rg['header'] = 'CALL MOTION PROGRAM: MSEE MPM002 (MA00100)'
            self.node.status_msg = f'[MPE720 Ladder] Đã đặt hàm MSEE gọi chương trình Motion MPM002 tại Rung {self.selected_rung_idx}.'
        self._draw_ladder_canvas()

    def _assign_io_to_rung(self, target: str = 'contact'):
        if not (0 <= self.selected_rung_idx < len(self.ladder_rungs)):
            return
        raw = self.selected_io_var.get()
        addr = raw.split(' ')[0].strip()
        desc = raw.split('(')[1].rstrip(')') if '(' in raw else addr
        rg = self.ladder_rungs[self.selected_rung_idx]
        if rg.get('type') == 'MSEE':
            rg['type'] = 'SERIES'
            rg.setdefault('c1_type', 'NO')
            rg.setdefault('c1_addr', 'DB000000')
            rg.setdefault('c1_var', 'VAR_Start')
            rg.setdefault('c1_cmt', 'System Run')
        if target == 'contact':
            rg['c2_addr'] = addr
            rg['c2_var'] = desc.split(':')[0].strip()
            rg['c2_cmt'] = desc.split(':')[-1].strip()
        else:
            rg['coil_addr'] = addr
            rg['coil_var'] = desc.split(':')[0].strip()
            rg['coil_cmt'] = desc.split(':')[-1].strip()
        self.node.status_msg = f'[MPE720 Ladder] Đã gán địa chỉ {addr} ({desc}) vào Rung {self.selected_rung_idx}.'
        self._draw_ladder_canvas()

    def _insert_new_rung(self):
        new_r = {
            'header': f'New Rung {len(self.ladder_rungs)} (Chọn địa chỉ I/O và kiểu tiếp điểm trên thanh công cụ)',
            'type': 'SERIES',
            'c1_type': 'NO', 'c1_addr': 'DB000000', 'c1_var': 'VAR_Start', 'c1_cmt': 'System Run',
            'c2_type': 'NO', 'c2_addr': 'IB0C40B', 'c2_var': 'DI_11_Sens1', 'c2_cmt': 'Sensor Đai 1',
            'coil_addr': 'OB0C401', 'coil_var': 'DO_01_R1_IN1', 'coil_cmt': 'Output',
        }
        ins_pos = min(len(self.ladder_rungs), self.selected_rung_idx + 1)
        self.ladder_rungs.insert(ins_pos, new_r)
        self.selected_rung_idx = ins_pos
        self._draw_ladder_canvas()

    def _delete_selected_rung(self):
        if len(self.ladder_rungs) > 1 and 0 <= self.selected_rung_idx < len(self.ladder_rungs):
            self.ladder_rungs.pop(self.selected_rung_idx)
            self.selected_rung_idx = max(0, self.selected_rung_idx - 1)
            self._draw_ladder_canvas()

    def _reset_default_ladder(self):
        self.ladder_rungs = self._default_ladder_rungs()
        self.selected_rung_idx = 0
        self.node.status_msg = '[MPE720 Ladder] Đã khôi phục Sơ đồ Ladder chuẩn Bài Thí Nghiệm 2 & 3!'
        self._draw_ladder_canvas()

    def _insert_motion_snippet(self, snippet: str):
        if hasattr(self, 'txt_plc'):
            self.txt_plc.insert(tk.INSERT, snippet)

    def _draw_wiring_canvas(self):
        if not hasattr(self, 'wiring_canvas'):
            return
        c = self.wiring_canvas
        c.delete('all')
        w = max(480, c.winfo_width())
        h = max(225, c.winfo_height())

        # Ba khối thiết bị chính: [1. NGÕ VÀO (SENSORS & PANEL)] ---> [2. MP3300 (LIO-04 / SVC)] ---> [3. NGÕ RA (2 ROBOT & 2 BĂNG TẢI)]
        x_in1, x_in2 = 8, int(w * 0.28)
        x_mc1, x_mc2 = int(w * 0.36), int(w * 0.64)
        x_out1, x_out2 = int(w * 0.72), w - 8

        # Khối 1: Thiết bị Ngõ vào (Control Panel + Sensors + Robot 1 OT#3)
        c.create_rectangle(x_in1, 8, x_in2, h - 8, fill='#1e242b', outline='#4dabf7', width=2)
        c.create_rectangle(x_in1, 8, x_in2, 26, fill='#1864ab', outline='#4dabf7')
        c.create_text((x_in1 + x_in2) // 2, 17, text='1. THIẾT BỊ NGÕ VÀO (INPUTS)', fill='white', font=('Ubuntu', 7, 'bold'))

        # Khối 2: Bộ điều khiển Machine Controller Yaskawa MP3300 (Module LIO-04 & SVC)
        c.create_rectangle(x_mc1, 8, x_mc2, h - 8, fill='#212529', outline='#fcc419', width=2)
        c.create_rectangle(x_mc1, 8, x_mc2, 26, fill='#e67700', outline='#fcc419')
        c.create_text((x_mc1 + x_mc2) // 2, 17, text='2. YASKAWA MP3300 (LIO-04)', fill='white', font=('Ubuntu', 7, 'bold'))

        # Khối 3: Thiết bị Chấp hành & 2 Robot YRC1000micro (Outputs)
        c.create_rectangle(x_out1, 8, x_out2, h - 8, fill='#1e242b', outline='#69db7c', width=2)
        c.create_rectangle(x_out1, 8, x_out2, 26, fill='#2b8a3e', outline='#69db7c')
        c.create_text((x_out1 + x_out2) // 2, 17, text='3. CHẤP HÀNH & 2 ROBOT (OUTPUTS)', fill='white', font=('Ubuntu', 7, 'bold'))

        # Danh sách 7 đường dây Ngõ vào (Trái -> Giữa: MC DI / IB0C4xx)
        inputs = [
            ('Nút START (Panel)', 'DI_0 : IB0C400', self.node.mc_start),
            ('Nút STOP (Panel)', 'DI_1 : IB0C401', not self.node.mc_start),
            ('Công tắc Servo_ON', 'DI_2 : IB0C402', self.node.mc_servo_on),
            ('Sensor HOME_A (Đĩa)', 'DI_10: IB0C40A', abs(self.node.rotary_angle_deg % 360.0) < 2.0),
            ('Sensor Băng tải 1', 'DI_11: IB0C40B', self.node.di_11_conv1),
            ('Robot 1 Xung OT#3', 'DI_12: IB0C40C', self.node.di_12_ot3),
            ('Sensor Băng tải 2', 'DI_13: IB0C40D', self.node.di_11_conv2),
        ]
        y_step_in = (h - 42) / len(inputs)
        for i, (dev_txt, pin_txt, active) in enumerate(inputs):
            y = int(36 + i * y_step_in)
            w_col = '#00e676' if active else '#495057'
            t_col = '#8ce99a' if active else '#ced4da'
            c.create_text(x_in1 + 6, y, text=dev_txt, fill=t_col, font=('Ubuntu', 7, 'bold'), anchor='w')
            c.create_line(x_in2, y, x_mc1, y, fill=w_col, width=2)
            c.create_oval(x_mc1 - 3, y - 3, x_mc1 + 3, y + 3, fill=w_col, outline='')
            c.create_text(x_mc1 + 5, y, text=pin_txt, fill='#ffd43b' if active else '#adb5bd', font=('Ubuntu Mono', 7, 'bold'), anchor='w')

        # Danh sách 6 đường dây Ngõ ra & Truyền thông (Giữa -> Phải: MC DO / OB0C4xx & Ethernet)
        outputs = [
            ('DO_0 : OB0C400', '-> Biến tần J1000 (Đai 1)', self.node.belt1_running),
            ('DO_1 : OB0C401', '-> Robot 1 (Chân IN#1)', self.node.do_01_in1),
            ('DO_2 : OB0C402', '-> Robot 2 (Chân IN#1)', self.node.do_02_in2),
            ('DO_3 : OB0C403', '-> Biến tần 2 (Đai 2 +X)', self.node.belt2_running),
            ('DO_16: OB0C410', '-> Đèn Q0 (Báo Hệ thống)', self.node.mc_start),
            ('SVC  : %OL8010', f'-> Servo Đĩa A1 ({self.node.rotary_angle_deg:.0f}°)', abs(self.node.rotary_target_deg - self.node.rotary_angle_deg) > 0.5),
            ('218IFD: UDP10040', '-> R1 (.11) & R2 (.12)', True),
        ]
        y_step_out = (h - 42) / len(outputs)
        for i, (pin_txt, target_txt, active) in enumerate(outputs):
            y = int(36 + i * y_step_out)
            w_col = '#00e676' if active else '#495057'
            t_col = '#8ce99a' if active else '#ced4da'
            c.create_text(x_mc2 - 5, y, text=pin_txt, fill='#ffd43b' if active else '#adb5bd', font=('Ubuntu Mono', 7, 'bold'), anchor='e')
            c.create_oval(x_mc2 - 3, y - 3, x_mc2 + 3, y + 3, fill=w_col, outline='')
            c.create_line(x_mc2, y, x_out1, y, fill=w_col, width=2)
            c.create_text(x_out1 + 6, y, text=target_txt, fill=t_col, font=('Ubuntu', 7, 'bold'), anchor='w')

    def _load_user_coord_skeleton(self):
        skel = (
            "# === BÀI TẬP TỰ CODE: TÍNH TỌA ĐỘ VẬT THỂ TỪ ENCODER & CẢM BIẾN TIỆM CẬN ===\n"
            "# Đầu vào có sẵn (50 Hz): enc1_pulses, enc2_pulses, s1_sensor, s2_sensor,\n"
            "#   s3_sensor, s4_sensor, on_rotary, rotary_deg, pulses_per_mm (=20.0 xung/mm), mem\n"
            "# Đầu ra cần tính (mm, deg): x_obj, y_obj, z_obj, yaw_obj\n\n"
            "# 1. Chốt xung Encoder 1 khi vật vừa chạm Cảm biến S1 (tại X=220mm, Y=-240mm):\n"
            "if s1_sensor and mem.get('latch1') is None:\n"
            "    mem['latch1'] = enc1_pulses\n"
            "    mem['stage'] = 'CONV1'\n\n"
            "# 2. Hãy viết công thức tính (x_obj, y_obj, z_obj, yaw_obj) của bạn ở dưới đây:\n"
            "if mem.get('latch1') is not None:\n"
            "    delta_p1 = enc1_pulses - mem['latch1']\n"
            "    # TODO: Sửa công thức tính y_obj từ delta_p1 và pulses_per_mm:\n"
            "    x_obj = 220.0\n"
            "    y_obj = -240.0 + (delta_p1 / pulses_per_mm)\n"
            "    z_obj = 40.0\n"
            "    yaw_obj = 0.0\n"
            "else:\n"
            "    x_obj, y_obj, z_obj, yaw_obj = 220.0, -380.0, 40.0, 0.0\n"
        )
        self._switch_plc_tab('USER_COORD')
        self.txt_plc.delete('1.0', tk.END)
        self.txt_plc.insert('1.0', skel)
        self.plc_codes['USER_COORD'] = skel
        self.node.status_msg = '[USER_COORD] Đã nạp Khung Sườn Bài Tập! Hãy sửa công thức rồi bấm [F4: COMPILE & DOWNLOAD].'

    def _load_user_coord_solution(self):
        fpath = self.plc_files.get('USER_COORD')
        if fpath and os.path.exists(fpath):
            with open(fpath, 'r', encoding='utf-8') as f:
                sol = f.read()
            self._switch_plc_tab('USER_COORD')
            self.txt_plc.delete('1.0', tk.END)
            self.txt_plc.insert('1.0', sol)
            self.plc_codes['USER_COORD'] = sol
            self.node.compile_user_coord_code(sol)
            self.node.status_msg = '[USER_COORD] Đã nạp & biên dịch Lời Giải Mẫu Chuẩn cho cả 2 Băng tải & Đĩa xoay!'

    def _switch_plc_tab(self, tab_key: str):
        # Lưu lại nội dung tab Code/Motion trước khi chuyển tab (không bao giờ ghi đè khi khởi tạo)
        if self.active_plc_tab in ('USER_COORD', 'MPM001_ZRN', 'MPM002_ROT90') and hasattr(self, 'txt_plc'):
            cur_txt = self.txt_plc.get('1.0', tk.END).strip()
            if cur_txt:
                self.plc_codes[self.active_plc_tab] = cur_txt + '\n'

        self.active_plc_tab = tab_key
        self.wiring_panel.pack_forget()
        self.ladder_panel.pack_forget()
        self.motion_panel.pack_forget()

        if tab_key == 'WIRING_DIAGRAM':
            self.wiring_panel.pack(fill=tk.BOTH, expand=True, pady=2)
            self.root.after(20, self._draw_wiring_canvas)
        elif tab_key == 'H01_LADDER':
            self.ladder_panel.pack(fill=tk.BOTH, expand=True, pady=2)
            self.root.after(20, self._draw_ladder_canvas)
        else:
            self.motion_panel.pack(fill=tk.BOTH, expand=True, pady=2)
            self.txt_plc.delete('1.0', tk.END)
            self.txt_plc.insert('1.0', self.plc_codes.get(tab_key, ''))

        for k, b in self.plc_tab_btns.items():
            if k == tab_key:
                b.config(bg='#fcc419', fg='black')
            else:
                b.config(bg='#495057', fg='white')

    def _compile_and_download_plc(self):
        if self.active_plc_tab in ('USER_COORD', 'MPM001_ZRN', 'MPM002_ROT90') and hasattr(self, 'txt_plc'):
            cur_txt = self.txt_plc.get('1.0', tk.END).strip()
            if cur_txt:
                self.plc_codes[self.active_plc_tab] = cur_txt + '\n'
                fpath = self.plc_files.get(self.active_plc_tab)
                if fpath and self.active_plc_tab != 'USER_COORD':
                    try:
                        with open(fpath, 'w', encoding='utf-8') as f:
                            f.write(self.plc_codes[self.active_plc_tab])
                    except Exception:
                        pass

        # Biên dịch Code tạo tọa độ vật thể (USER_COORD) & Sơ đồ Ladder (H01_LADDER) của người dùng
        self.node.compiled_ladder_rungs = [dict(r) for r in self.ladder_rungs]
        ucode = self.plc_codes.get('USER_COORD', '')
        ok_user = self.node.compile_user_coord_code(ucode) if ucode else True

        # Phân tích lệnh OL8010 / VEL [A1] và MOV[A1] trong MPM002_ROT90 (Đơn vị 0.1 độ chuẩn Trang 5 Phụ lục)
        mpm2 = self.plc_codes.get('MPM002_ROT90', '')
        for line in mpm2.splitlines():
            s = line.split('//')[0].strip().rstrip(';')
            if s.startswith('MOV[A1]') or s.startswith('MOV [A1]'):
                val_str = s.replace('MOV[A1]', '').replace('MOV [A1]', '').strip()
                try:
                    raw_val = float(val_str)
                    # Trong MPE720 (Trang 5 Phụ lục): MOV[A1]900 tương ứng 90.0 độ (đơn vị 0.1 deg)
                    self.node.mpm002_step_deg = raw_val / 10.0 if abs(raw_val) > 360.0 else raw_val
                except ValueError:
                    pass
            elif s.startswith('OL8010=') or s.startswith('VEL [A1]'):
                val_str = s.replace('OL8010=', '').replace('VEL [A1]', '').strip()
                try:
                    self.node.mpm002_vel_deg_s = max(10.0, float(val_str) / 10.0)
                except ValueError:
                    pass
        if ok_user:
            self.node.status_msg = (
                f'[F4 COMPILE OK] Đã nạp Code Tọa Độ Vật (USER [OBJECT]) & PLC '
                f'(MPM002: +{self.node.mpm002_step_deg:.1f}°, {self.node.mpm002_vel_deg_s:.1f}°/s)!'
            )
        else:
            self.node.status_msg = f'! [COMPILE ERROR] {self.node.user_code_error}'

    def _set_mode(self, m: str):
        if m == 'INDEXING_STEP':
            self.node.start_index_step(100.0)
        else:
            self.node.conveyor_mode = m

    def _toggle_autodrop(self):
        self.node.auto_drop_enabled = not self.node.auto_drop_enabled
        if self.node.auto_drop_enabled:
            self.btn_ad.config(text='AUTO-DROP: BẬT', bg='#2b8a3e')
        else:
            self.btn_ad.config(text='AUTO-DROP: TẮT', bg='#495057')

    def _update_ui(self):
        self.lbl_status.config(text=f'{self.node.status_msg}')

        # Vẽ cập nhật trực tiếp trạng thái dẫn điện (màu xanh lá) trên Sơ đồ Đấu Dây hoặc Sơ đồ Ladder Đồ Họa MPE720
        if self.active_plc_tab == 'WIRING_DIAGRAM':
            self._draw_wiring_canvas()
        elif self.active_plc_tab == 'H01_LADDER':
            self._draw_ladder_canvas()

        l1 = str(self.node.user_mem.get('latch1')) if self.node.user_mem.get('latch1') is not None else '---'
        l2 = str(self.node.user_mem.get('latch2')) if self.node.user_mem.get('latch2') is not None else '---'
        s_flags = (
            f"S1={'ON' if self.node.s1_sensor else '0'} "
            f"S2={'ON' if self.node.s2_sensor else '0'} "
            f"S3={'ON' if self.node.s3_sensor else '0'} "
            f"S4={'ON' if self.node.s4_sensor else '0'}"
        )
        self.lbl_enc12.config(
            text=(
                f'enc1_pulses (Đai 1 +Y): {self.node.enc1_pulses:7d} (latch1: {l1}) | {s_flags}\n'
                f'enc2_pulses (Đai 2 +X): {self.node.enc2_pulses:7d} (latch2: {l2}) | pulses_per_mm={self.node.pulses_per_mm:.0f}'
            )
        )

        sname = self.node.shape_catalog[self.node.p1_shape][0]
        err_x = (self.node.est_x - self.node.p1_x) * 1000.0
        err_y = (self.node.est_y - self.node.p1_y) * 1000.0
        self.lbl_pose.config(
            text=(
                f'Phôi: {sname} | TT: {self.node.p1_state} | Code: {self.node.user_code_error}\n'
                f'Tọa độ Code (USER): X={self.node.est_x*1000:5.1f}  Y={self.node.est_y*1000:6.1f}  Z={self.node.est_z*1000:4.1f} mm\n'
                f'Tọa độ Thực tế    : X={self.node.p1_x*1000:5.1f}  Y={self.node.p1_y*1000:6.1f}  (Sai số: dX={err_x:+4.1f}, dY={err_y:+4.1f} mm)'
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
