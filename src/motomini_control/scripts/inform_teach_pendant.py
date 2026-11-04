#!/usr/bin/env python3
"""
Mo phong CHUAN 100% Bo Lap Trinh YASKAWA YRC1000micro Programming Pendant
theo Tai lieu Thi nghiem Ky thuat Robot - Bai 1 (Bo mon Dieu khien Tu Dong - DHBK TPHCM):

1. Bo cuc vat ly chuan Hinh 5 & Bang 2 (Trang 4, 5, 6):
   - Cum tren dinh: Num xoay Mode Switch (REMOTE / PLAY / TEACH), Nut [START] xanh, Nut [HOLD] vang,
     Nut [EMERGENCY STOP] do tren nen vang.
   - Man hinh cam ung 5.7 inch chuan Hinh 6 & Bang 3 (Trang 7):
     + Menu area (JOB, EDIT, DISPLAY, UTILITY) + Status display area (Icon He toa do: Joint/Cartesian/Tool/User,
       Icon Toc do: INCH/LOW/MED/HIGH, Icon Trang thai, Tool #00).
     + Main menu area ben trai (JOB, GENERAL, VARIABLE, IN/OUT, ROBOT, SYSTEM INFO).
     + General purpose display area giua (JOB CONTENT co cot Address 0000.. + Instruction + Buffer line mau xam,
       hoac man hinh POSITION VARIABLE #P000..#P014, hoac GENERAL PURPOSE OUTPUT/INPUT).
     + Human interface display area duoi day.
   - Ban phim cung phia duoi man hinh:
     + Hang dieu huong: [MAIN MENU], [PAGE], [AREA], [SELECT], [CANCEL], Cum 4 phim mui ten Cursor Key (UP/DOWN/LEFT/RIGHT).
     + Cum van hanh tay & Truc: [SERVO ON READY], [Enable Switch], [COORD], [HIGH SPEED], [FAST ▲] / [SLOW ▼],
       12 phim Axis Key [X-/S-], [X+/S+], [Y-/L-], [Y+/L+], [Z-/U-], [Z+/U+], [X-/R-], [X+/R+], [Y-/B-], [Y+/B+], [Z-/T-], [Z+/T+].
     + Cum lap trinh INFORM: [SHIFT], [INTERLOCK], [INFORM LIST], [DIRECT OPEN], [TEST START], [FWD], [BWD],
       [DELETE], [INSERT], [MODIFY], [ENTER] + Bang dieu khien I/O Control Panel (I4..I7 -> IN#5..8, Q4..Q7 -> OUT#5..8).
"""

import os
import sys
import math
import time
import threading
import numpy as np

# Lam sach bien moi truong Snap de Tkinter / GUI chay muot tu VS Code Terminal
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
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


# =====================================================================
# DONG HOC THUAN & NGHICH VI PHAN (JACOBIAN 6-DOF) MOTOMINI
# =====================================================================
def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]], dtype=float)


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]], dtype=float)


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)


def _trans(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = x, y, z
    return T


def fk_motomini(q):
    j1, j2, j3, j4, j5, j6 = q
    T = np.eye(4)
    T = T @ _trans(0, 0, 0.103) @ _rot_z(j1)          # Trục S
    T = T @ _trans(0.020, 0, 0) @ _rot_y(j2)          # Trục L
    T = T @ _trans(0, 0, 0.165) @ _rot_y(-j3)         # Trục U
    T_wrist = T @ _trans(0.165, 0, 0) @ _rot_x(-j4)   # Trục R
    T5 = T_wrist @ _rot_y(-j5)                        # Trục B
    T6 = T5 @ _rot_z(j6)                              # Trục T
    T_tcp = T6 @ _trans(0, 0, -0.040) @ _rot_x(math.pi) @ _trans(0, 0, 0.050)
    return T_tcp, T_wrist


def rot_to_rpy_deg(R: np.ndarray):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.atan2(-R[2, 0], sy)
        rz = 0.0
    return math.degrees(rx), math.degrees(ry), math.degrees(rz)


def compute_jacobian_world(q, eps=1e-5):
    T0, _ = fk_motomini(q)
    p0 = T0[:3, 3]
    R0 = T0[:3, :3]
    J = np.zeros((6, 6))
    for i in range(6):
        dq = list(q)
        dq[i] += eps
        Ti, _ = fk_motomini(dq)
        pi = Ti[:3, 3]
        Ri = Ti[:3, :3]
        J[:3, i] = (pi - p0) / eps
        dR = Ri @ R0.T
        J[3:, i] = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / (2.0 * eps)
    return J, T0


T_WORLD_USER1 = _trans(0.25, 0.0, 0.0) @ _rot_z(math.radians(30.0))
T_USER1_WORLD = np.linalg.inv(T_WORLD_USER1)


# =====================================================================
# NODE ROS 2 DIEU KHIEN & GIAI TAP LENH INFORM (BANG 4 - TRANG 15, 16, 17)
# =====================================================================
class InformTeachPendantNode(Node):
    # Thu tu 4 he toa do theo đúng Trang 5 & Trang 9: JOINT -> CARTESIAN -> TOOL -> USER
    COORD_MODES = ['JOINT', 'CARTESIAN', 'TOOL', 'USER']

    def __init__(self, robot_id: str = 'R1'):
        robot_id = robot_id.upper() if robot_id.upper() in ('R1', 'R2') else 'R1'
        super().__init__(f'inform_teach_pendant_{robot_id.lower()}')

        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/manipulator_controller/follow_joint_trajectory',
        )
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/manipulator_controller/joint_trajectory',
            10,
        )

        # ActionClient & Publisher cho Canh tay Robot 2 (R2 - CONTROL GROUP: R2)
        self.traj_r2_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/manipulator_r2_controller/follow_joint_trajectory',
        )
        self.traj_r2_pub = self.create_publisher(
            JointTrajectory,
            '/manipulator_r2_controller/joint_trajectory',
            10,
        )

        # Digital Inputs IN#1..IN#8 & Outputs OUT#1..OUT#8 (Trang 9, 17)
        self.din = {i: False for i in range(1, 9)}
        self.dout = {i: False for i in range(1, 9)}

        self.dout1_pub = self.create_publisher(Bool, '/yaskawa/dout1', 10)
        self.dout2_pub = self.create_publisher(Bool, '/yaskawa/dout2', 10)
        self.dout3_pub = self.create_publisher(Bool, '/yaskawa/dout3', 10)
        self.din1_sub = self.create_subscription(
            Bool, '/yaskawa/din1', self.din1_callback, 10
        )
        self.din2_sub = self.create_subscription(
            Bool, '/yaskawa/din2', self.din2_callback, 10
        )
        # Nhận Hệ tọa độ USER gắn trực tiếp vào Đối tượng đang di chuyển + Vận tốc Feedforward (50Hz Không Trễ)
        self.obj_uf_sub = self.create_subscription(
            Float64MultiArray, '/conveyor/object_user_frame', self.obj_uf_callback, 10
        )
        self.obj_world_pose = [0.22, 0.0, 0.04, 0.0]   # [x, y, z_top, yaw]
        self.obj_world_vel = [0.0, 0.0, 0.0]           # [vx, vy, wz]
        self.user_object_lock = False                  # Chế độ khóa bám theo Hệ USER của đối tượng không trễ
        self.user_target_offset = [0.0, 0.0, 0.045]    # Tọa độ mục tiêu trong Hệ USER của đối tượng (mặc định: tâm vật, cao +45mm)

        # Bien Byte B000..B019, Integer I000..I009 (Trang 15, 16)
        self.b_vars = {f'B{i:03d}': 0 for i in range(20)}
        self.i_vars = {f'I{i:03d}': 0 for i in range(10)}

        # Gioi han khop chuan Bang 1 (Trang 2):
        # S: -170..+170, L: -85..+90, U: -50..+90, R: -140..+140, B: -30..+210, T: -360..+360
        self.active_robot = robot_id     # 'R1' (Robot 1 - Băng tải 1) hoặc 'R2' (Robot 2 - Băng tải 2)
        self.r1_joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'
        ]
        self.r2_joint_names = [
            'r2_joint_1', 'r2_joint_2', 'r2_joint_3', 'r2_joint_4', 'r2_joint_5', 'r2_joint_6'
        ]
        self.joint_limits = [
            (math.radians(-170), math.radians(170)),
            (math.radians(-85), math.radians(90)),
            (math.radians(-50), math.radians(90)),
            (math.radians(-140), math.radians(140)),
            (math.radians(-30), math.radians(210)),
            (math.radians(-360), math.radians(360)),
        ]
        self.r1_joints = [0.0] * 6
        self.r2_joints = [0.0] * 6
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Trang thai van hanh theo Bang 2 (Trang 4, 5, 9)
        self.mode_switch = 'TEACH'       # 'REMOTE', 'PLAY', 'TEACH'
        self.servo_ready = True          # [SERVO ON READY]
        self.enable_switch_on = True     # Enable Switch (Deadman switch)
        self.estop_active = False        # [EMERGENCY STOP]

        self.coord_mode = 'JOINT'        # JOINT -> CARTESIAN -> TOOL -> USER
        # 4 muc toc do theo Bang 2 (Trang 5, 7): INCH -> LOW -> MED -> HIGH
        self.speed_icons = ['[I] INCH', '[L] LOW', '[M] MED', '[H] HIGH']
        self.speed_values = [5.0, 20.0, 50.0, 100.0]
        self.speed_idx = 2               # Mac dinh MED (50%)
        self.high_speed_held = False
        self.shift_held = False
        self.interlock_held = False

        # Cac diem P000 -> P012 rieng biet cho tung Robot R1 & R2
        if self.active_robot == 'R1':
            self.points = {
                'P000': [0.000, 0.000, 0.000, 0.000, 0.000, 0.000],   # Home Pos R1 (Hinh 11)
                'P001': [0.000, 0.420, 0.160, 0.000, 0.260, 0.000],   # R1: Tren Bang tai 1 (S2) (+50mm)
                'P002': [0.000, 0.640, -0.020, 0.000, 0.660, 0.000],  # R1: Gap vat tai Bang tai 1 (S2)
                'P003': [-0.250, 0.436, 0.175, 0.000, 0.262, 0.000],  # Tren diem A2 (+50mm)
                'P004': [-0.250, 0.655, -0.020, 0.000, 0.675, 0.000], # Tiep xuc vat tai A2
                'P005': [0.250, 0.436, 0.175, 0.000, 0.262, 0.000],   # Tren diem A3 (+50mm)
                'P006': [0.250, 0.655, -0.020, 0.000, 0.675, 0.000],  # Tiep xuc vat tai A3
                'P007': [0.775, 0.540, 0.220, 0.000, 0.320, 0.000],   # R1: Tren Đĩa xoay Trục A (+50mm)
                'P008': [0.775, 0.720, 0.050, 0.000, 0.670, 0.000],   # R1: Đặt vật lên Đĩa xoay Trục A
                'P009': [0.775, 0.680, 0.090, 0.000, 0.590, 0.000],   # Xếp chồng tại Đĩa xoay (Tầng 2)
                'P010': [0.775, 0.620, 0.140, 0.000, 0.480, 0.000],   # Xếp chồng tại Đĩa xoay (Tầng 3)
                'P011': [1.200, 0.436, 0.175, 0.000, 0.262, 0.000],   # Tren o tha C (+50mm)
                'P012': [1.200, 0.630, 0.000, 0.000, 0.630, 0.000],   # Tha vat tai o C
            }
        else:
            # Diem P000..P012 danh rieng cho Robot 2 (R2: Gap tu Đĩa xoay Trục A -> Đặt sang Băng tải 2 vuông góc)
            self.points = {
                'P000': [0.000, 0.000, 0.000, 0.000, 0.000, 0.000],   # Home Pos R2
                'P001': [1.200, 0.540, 0.220, 0.000, 0.320, 0.000],   # R2: Tren Đĩa xoay Trục A (+50mm)
                'P002': [1.200, 0.720, 0.050, 0.000, 0.670, 0.000],   # R2: Gắp vật từ Đĩa xoay Trục A
                'P003': [1.100, 0.540, 0.220, 0.000, 0.320, 0.000],   # R2: Điểm A2 trên Đĩa xoay
                'P004': [1.100, 0.720, 0.050, 0.000, 0.670, 0.000],   # R2: Chạm điểm A2
                'P005': [1.300, 0.540, 0.220, 0.000, 0.320, 0.000],   # R2: Điểm A3 trên Đĩa xoay
                'P006': [1.300, 0.720, 0.050, 0.000, 0.670, 0.000],   # R2: Chạm điểm A3
                'P007': [0.980, 0.460, 0.170, 0.000, 0.290, 0.000],   # R2: Tren đầu Băng tải 2 vuông góc (+50mm)
                'P008': [0.980, 0.660, -0.010, 0.000, 0.670, 0.000],  # R2: Đặt vật lên Băng tải 2 vuông góc
                'P009': [0.980, 0.600, 0.050, 0.000, 0.550, 0.000],   # R2: Xếp chồng Băng tải 2 (Tầng 2)
                'P010': [0.980, 0.540, 0.110, 0.000, 0.430, 0.000],   # R2: Xếp chồng Băng tải 2 (Tầng 3)
                'P011': [0.000, 0.460, 0.170, 0.000, 0.290, 0.000],   # R2: Tren giữa Băng tải 2 (+50mm)
                'P012': [0.000, 0.660, -0.010, 0.000, 0.670, 0.000],  # R2: Đặt vật giữa Băng tải 2
            }

        # Thu vien cac JOB theo đúng Trang 16 & 17 (MAIN, PICK-A1, PLACE-B, CONVEYOR-AUTO)
        self.jobs = {
            'MAIN': [
                'NOP',
                'WAIT IN#(1)=ON',
                'CALL JOB:PICK-A1',
                'CALL JOB:PLACE-B',
                'PULSE OT#(5) T=1.00',
                'END',
            ],
            'PICK-A1': [
                'NOP',
                'MOVJ P000 VJ=50.00',
                'MOVJ P001 VJ=50.00',
                'MOVL P002 V=25.00 PL=0',
                'DOUT OT#(1) ON',
                'TIMER T=0.50',
                'MOVL P001 V=30.00',
                'END',
            ],
            'PLACE-B': [
                'NOP',
                'MOVJ P007 VJ=50.00',
                'MOVL P008 V=25.00 PL=0',
                'DOUT OT#(1) OFF',
                'TIMER T=0.50',
                'MOVL P007 V=40.00',
                'MOVJ P000 VJ=50.00',
                'END',
            ],
            'PICK-USER-UF1': [
                'NOP',
                'WAIT IN#(1)=ON',
                'MOVL UF#(1) (0,0,45) V=60.0',
                'MOVL UF#(1) (0,0,0) V=25.0',
                'DOUT OT#(1) ON',
                'TIMER T=0.35',
                'MOVL UF#(1) (0,0,50) V=50.0',
                'CALL JOB:PLACE-B',
                'PULSE OT#(3) T=0.10',
                'END',
            ],
        }
        self.current_job_name = 'MAIN'

        self.is_running_job = False
        self.stop_requested = False
        self.status_callback = None
        self.highlight_callback = None

        # JOG & Object USER-Frame Tracking Liên Tục 50 Hz (dt = 0.02s - Không trễ)
        self.jog_active_axis = None
        self.jog_direction = 0.0
        self.jog_target_joints = None
        self.jog_dt = 0.020
        self.jog_timer = self.create_timer(self.jog_dt, self._continuous_jog_loop)

    @property
    def joint_names(self):
        return self.r1_joint_names if self.active_robot == 'R1' else self.r2_joint_names

    @property
    def current_joints(self):
        return self.r1_joints if self.active_robot == 'R1' else self.r2_joints

    @property
    def active_traj_pub(self):
        return self.traj_pub if self.active_robot == 'R1' else self.traj_r2_pub

    @property
    def active_traj_client(self):
        return self.traj_client if self.active_robot == 'R1' else self.traj_r2_client

    @property
    def is_servo_powered(self) -> bool:
        if self.estop_active:
            return False
        if self.mode_switch == 'PLAY':
            return self.servo_ready
        return self.servo_ready and self.enable_switch_on

    def din1_callback(self, msg: Bool):
        if self.active_robot == 'R1':
            self.din[1] = msg.data
        else:
            self.din[2] = msg.data

    def din2_callback(self, msg: Bool):
        if self.active_robot == 'R2':
            self.din[1] = msg.data
        else:
            self.din[2] = msg.data

    def obj_uf_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 7:
            self.obj_world_pose = [msg.data[0], msg.data[1], msg.data[2], msg.data[3]]
            self.obj_world_vel = [msg.data[4], msg.data[5], msg.data[6]]

    def get_object_user_frame_in_base(self):
        """
        Chuyển đổi Hệ tọa độ USER gắn trên Đối tượng (từ World frame sang Base frame của R1 hoặc R2)
        cùng вектор vận tốc Feedforward của đối tượng để điều khiển bám không có độ trễ!
        """
        xw, yw, zw, yaw_w = self.obj_world_pose
        vx_w, vy_w, wz_w = self.obj_world_vel
        T_w_obj = _trans(xw, yw, zw) @ _rot_z(yaw_w)

        if self.active_robot == 'R1':
            T_base_obj = T_w_obj
            v_base = np.array([vx_w, vy_w, 0.0, 0.0, 0.0, wz_w], dtype=float)
        else:
            # Gốc Robot 2 (R2) đặt tại World (0.58, 0.14, 0.0) và xoay +90 độ (+pi/2)
            T_w_r2 = _trans(0.58, 0.14, 0.0) @ _rot_z(math.pi / 2.0)
            T_r2_w = np.linalg.inv(T_w_r2)
            T_base_obj = T_r2_w @ T_w_obj
            R_r2_w = T_r2_w[:3, :3]
            v_lin_base = R_r2_w @ np.array([vx_w, vy_w, 0.0], dtype=float)
            v_base = np.array([v_lin_base[0], v_lin_base[1], v_lin_base[2], 0.0, 0.0, wz_w], dtype=float)

        T_obj_base = np.linalg.inv(T_base_obj)
        return T_base_obj, T_obj_base, v_base

    def joint_state_callback(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        if all(j in name_to_pos for j in self.r1_joint_names):
            self.r1_joints = [name_to_pos[j] for j in self.r1_joint_names]
        if all(j in name_to_pos for j in self.r2_joint_names):
            self.r2_joints = [name_to_pos[j] for j in self.r2_joint_names]

    def set_dout(self, ch: int, state_on: bool):
        self.dout[ch] = state_on
        msg = Bool()
        msg.data = state_on
        if ch == 1:
            if self.active_robot == 'R1':
                self.dout1_pub.publish(msg)
            else:
                self.dout2_pub.publish(msg)
        elif ch == 2:
            self.dout2_pub.publish(msg)
        elif ch in (3, 5):
            self.dout3_pub.publish(msg)

    def pulse_dout(self, ch: int, duration_sec: float = 1.0):
        def _p():
            self.set_dout(ch, True)
            time.sleep(duration_sec)
            self.set_dout(ch, False)
        threading.Thread(target=_p, daemon=True).start()

    def get_effective_speed(self) -> float:
        if self.high_speed_held:
            return 100.0
        return self.speed_values[self.speed_idx]

    def get_current_pose_in_coord(self):
        T_tcp, _ = fk_motomini(self.current_joints)
        if self.coord_mode == 'USER':
            _, T_obj_base, _ = self.get_object_user_frame_in_base()
            T_rel = T_obj_base @ T_tcp
        else:
            T_rel = T_tcp
        x_mm = T_rel[0, 3] * 1000.0
        y_mm = T_rel[1, 3] * 1000.0
        z_mm = T_rel[2, 3] * 1000.0
        rx_deg, ry_deg, rz_deg = rot_to_rpy_deg(T_rel[:3, :3])
        return (x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)

    def check_safety_collision(self, target_joints):
        for i, val in enumerate(target_joints):
            low, high = self.joint_limits[i]
            if val < low or val > high:
                axis_n = ['S', 'L', 'U', 'R', 'B', 'T'][i]
                return False, f'! ALARM: Trục {axis_n} chạm giới hạn hành trình (Bảng 1)!'

        T_tcp, T_wrist = fk_motomini(target_joints)
        x_tcp, y_tcp, z_tcp = T_tcp[0, 3], T_tcp[1, 3], T_tcp[2, 3]
        z_wrist = T_wrist[2, 3]

        if z_wrist < 0.055:
            return False, f'! Chặn va chạm: Cổ tay robot quá thấp (Z={z_wrist*1000:.0f}mm)!'

        if self.dout[1] and z_tcp < 0.039:
            return False, f'! Chặn xuyên thấu: Đáy vật mẫu chạm mặt bàn (Z={z_tcp*1000:.1f}mm < 40mm)!'

        if self.din[1] and abs(x_tcp - 0.25) < 0.035 and abs(y_tcp - 0.0) < 0.035:
            if z_tcp < 0.039:
                return False, '! Chặn xuyên thấu: Đầu hút đã chạm mặt trên vật mẫu (Z=40mm)!'

        if z_tcp < 0.002:
            return False, f'! Chặn va chạm: Đầu hút chạm mặt băng tải/bàn (Z={z_tcp*1000:.1f}mm)!'

        return True, 'OK'

    def start_continuous_jog(self, axis_idx: int, direction: float):
        if self.mode_switch != 'TEACH':
            self.log_status('! Chỉ điều khiển bằng tay (JOG) khi Mode Switch ở vị trí [TEACH].')
            return
        if not self.is_servo_powered:
            self.log_status('! SERVO đang OFF! Hãy bật [SERVO ON READY] và giữ [Enable Switch].')
            return
        if self.is_running_job:
            return

        self.jog_target_joints = list(self.current_joints)
        self.jog_active_axis = axis_idx
        self.jog_direction = direction
        lbl = (
            ['S', 'L', 'U', 'R', 'B', 'T'][axis_idx]
            if self.coord_mode == 'JOINT'
            else ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'][axis_idx]
        )
        sign = '+' if direction > 0 else '-'
        self.log_status(f'Manual Jog [{self.coord_mode}]: Đang giữ trục {lbl}{sign} ({self.get_effective_speed():.0f}%)')

    def stop_continuous_jog(self):
        if self.jog_active_axis is not None:
            self.jog_active_axis = None
            self.jog_direction = 0.0
            msg = JointTrajectory()
            msg.joint_names = self.joint_names
            pt = JointTrajectoryPoint()
            pt.positions = list(self.current_joints)
            pt.velocities = [0.0] * 6
            pt.time_from_start = Duration(seconds=0, nanoseconds=50_000_000).to_msg()
            msg.points = [pt]
            self.active_traj_pub.publish(msg)

    def _continuous_jog_loop(self):
        if self.is_running_job or not self.is_servo_powered:
            return
        if self.jog_active_axis is None and not self.user_object_lock:
            return

        spd_scale = self.get_effective_speed() / 100.0
        axis = self.jog_active_axis
        d = self.jog_direction

        if self.jog_target_joints is None:
            self.jog_target_joints = list(self.current_joints)
        q_curr = list(self.jog_target_joints)

        T_base_obj, T_obj_base, v_obj_base = self.get_object_user_frame_in_base()
        R_user = T_base_obj[:3, :3]

        if self.coord_mode == 'JOINT' and not self.user_object_lock:
            omega = 1.6 * spd_scale * d
            candidate = list(q_curr)
            low, high = self.joint_limits[axis]
            candidate[axis] = max(low, min(high, candidate[axis] + omega * self.jog_dt))
            q_dot_cmd = [omega if i == axis else 0.0 for i in range(6)]
        else:
            v_lin_max = 0.16 * spd_scale
            v_ang_max = 1.40 * spd_scale
            twist_local = np.zeros(6)
            if axis is not None:
                if axis < 3:
                    twist_local[axis] = v_lin_max * d
                else:
                    twist_local[axis] = v_ang_max * d

            J_world, T_tcp = compute_jacobian_world(q_curr)
            R_tcp = T_tcp[:3, :3]
            p_tcp = T_tcp[:3, 3]

            twist_world = np.zeros(6)
            if self.user_object_lock:
                # 1. Cập nhật offset mục tiêu (x_u, y_u, z_u) ngay trong Hệ USER của Đối tượng khi bấm phím JOG
                if axis is not None and axis < 3:
                    self.user_target_offset[axis] += twist_local[axis] * self.jog_dt
                    self.user_target_offset[2] = max(0.0, min(0.16, self.user_target_offset[2]))
                # 2. Tọa độ mục tiêu tức thời trong Base frame = T_base_obj * p_offset_user
                p_des_base = T_base_obj[:3, 3] + R_user @ np.array(self.user_target_offset, dtype=float)
                e_pos = p_des_base - p_tcp
                # 3. Luật điều khiển Bù Vận Tốc Feedforward (v_obj_base) + Khóa vị trí Kp*e_pos -> KHÔNG ĐỘ TRỄ!
                kp_lock = 14.0
                v_cmd_lin = v_obj_base[:3] + R_user @ twist_local[:3] + kp_lock * e_pos
                v_np = np.linalg.norm(v_cmd_lin)
                if v_np > 0.45:
                    v_cmd_lin = v_cmd_lin * (0.45 / v_np)
                twist_world[:3] = v_cmd_lin
                # Giữ đầu hút luôn hướng thẳng đứng xuống đối tượng
                z_des = np.array([0.0, 0.0, -1.0])
                z_cur = R_tcp[:, 2]
                e_rot = np.cross(z_cur, z_des)
                twist_world[3:] = 8.0 * e_rot
            elif self.coord_mode == 'CARTESIAN':
                twist_world = twist_local
            elif self.coord_mode == 'TOOL':
                twist_world[:3] = R_tcp @ twist_local[:3]
                twist_world[3:] = R_tcp @ twist_local[3:]
            elif self.coord_mode == 'USER':
                # Trong Hệ USER: Trục X, Y, Z xoay theo hướng Đối tượng + Cộng bù vận tốc Feedforward của Đối tượng
                twist_world[:3] = v_obj_base[:3] + R_user @ twist_local[:3]
                twist_world[3:] = v_obj_base[3:] + R_user @ twist_local[3:]

            lam = 0.025
            JJT = J_world @ J_world.T + (lam ** 2) * np.eye(6)
            q_dot_vec = J_world.T @ np.linalg.solve(JJT, twist_world)
            q_dot_cmd = [float(q_dot_vec[i]) for i in range(6)]
            candidate = [
                max(self.joint_limits[i][0], min(self.joint_limits[i][1], q_curr[i] + q_dot_cmd[i] * self.jog_dt))
                for i in range(6)
            ]

        is_safe, reason = self.check_safety_collision(candidate)
        if not is_safe:
            self.log_status(reason)
            return

        self.jog_target_joints = candidate
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = list(self.jog_target_joints)
        # Truyền trực tiếp vận tốc khớp tức thời q_dot_cmd & time_from_start=22ms để bộ nội suy Spline chạy 0ms trễ!
        pt.velocities = q_dot_cmd
        pt.time_from_start = Duration(seconds=0, nanoseconds=22_000_000).to_msg()
        msg.points = [pt]
        self.active_traj_pub.publish(msg)

    def log_status(self, text: str):
        if self.status_callback:
            self.status_callback(text)

    def move_to_joints(self, target_joints, speed_percent=50.0):
        if not self.is_servo_powered:
            self.log_status('! Không thể di chuyển: Nguồn SERVO đang tắt (OFF).')
            return False

        is_safe, reason = self.check_safety_collision(target_joints)
        if not is_safe:
            self.log_status(reason)
            return False

        if not self.active_traj_client.wait_for_server(timeout_sec=2.0):
            return False

        speed_clamped = max(5.0, min(100.0, float(speed_percent)))
        duration_sec = max(0.5, 2.0 * (50.0 / speed_clamped))

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in target_joints]
        point.velocities = [0.0] * 6
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start = Duration(seconds=sec, nanoseconds=nanosec).to_msg()
        goal_msg.trajectory.points = [point]

        send_future = self.active_traj_client.send_goal_async(goal_msg)
        while not send_future.done():
            if self.stop_requested:
                return False
            time.sleep(0.02)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if self.stop_requested:
                goal_handle.cancel_goal_async()
                return False
            time.sleep(0.02)
        return True

    def move_to_user_frame_offset(self, dx_mm=0.0, dy_mm=0.0, dz_mm=0.0, speed_percent=50.0):
        """
        Di chuyển đầu hút TCP tới điểm (dx_mm, dy_mm, dz_mm) trong Hệ tọa độ USER [UF#(1)]
        do Code của Người Dùng tính trên GUI MP3300 (có bù vận tốc Feedforward 50 Hz để gắp cả khi băng tải đang chạy).
        """
        if not self.is_servo_powered:
            self.log_status('! Không thể di chuyển UF#(1): Nguồn SERVO đang tắt (OFF).')
            return False

        offset_m = np.array([dx_mm / 1000.0, dy_mm / 1000.0, dz_mm / 1000.0], dtype=float)
        spd_scale = max(0.2, min(1.5, float(speed_percent) / 50.0))
        q_curr = list(self.current_joints)
        t_start = time.time()
        settled_count = 0

        while rclpy.ok() and not self.stop_requested and (time.time() - t_start) < 3.0:
            T_base_obj, _, v_obj_base = self.get_object_user_frame_in_base()
            R_user = T_base_obj[:3, :3]
            p_des = T_base_obj[:3, 3] + R_user @ offset_m

            J_world, T_tcp = compute_jacobian_world(q_curr)
            R_tcp = T_tcp[:3, :3]
            p_tcp = T_tcp[:3, 3]

            e_pos = p_des - p_tcp
            dist_err = float(np.linalg.norm(e_pos))
            if dist_err < 0.004:
                settled_count += 1
                if settled_count >= 3:
                    break
            else:
                settled_count = 0

            v_cmd_lin = v_obj_base[:3] + (10.0 * spd_scale) * e_pos
            v_np = np.linalg.norm(v_cmd_lin)
            v_max = 0.35 * spd_scale
            if v_np > v_max:
                v_cmd_lin = v_cmd_lin * (v_max / v_np)

            z_des = np.array([0.0, 0.0, -1.0])
            z_cur = R_tcp[:, 2]
            e_rot = np.cross(z_cur, z_des)

            twist_world = np.zeros(6)
            twist_world[:3] = v_cmd_lin
            twist_world[3:] = 7.0 * e_rot

            lam = 0.025
            JJT = J_world @ J_world.T + (lam ** 2) * np.eye(6)
            q_dot_vec = J_world.T @ np.linalg.solve(JJT, twist_world)
            q_dot_cmd = [float(q_dot_vec[i]) for i in range(6)]
            candidate = [
                max(self.joint_limits[i][0], min(self.joint_limits[i][1], q_curr[i] + q_dot_cmd[i] * 0.020))
                for i in range(6)
            ]
            is_safe, reason = self.check_safety_collision(candidate)
            if not is_safe:
                self.log_status(reason)
                return False

            q_curr = candidate
            self.jog_target_joints = list(q_curr)
            msg = JointTrajectory()
            msg.joint_names = self.joint_names
            pt = JointTrajectoryPoint()
            pt.positions = list(q_curr)
            pt.velocities = q_dot_cmd
            pt.time_from_start = Duration(seconds=0, nanoseconds=22_000_000).to_msg()
            msg.points = [pt]
            self.active_traj_pub.publish(msg)
            time.sleep(0.020)

        return True

    def execute_inform_line(self, raw_line: str):
        """Thuc thi day du tap lenh INFORM theo Bang 4 (Trang 15, 16, 17) & Lenh UF#(1) do nguoi dung code."""
        line = raw_line.strip().upper()
        if not line or line.startswith("'") or line.startswith('*') or line in ('NOP', 'END', 'RET'):
            return

        # Kiem tra dieu kien IF o cuoi cau lenh (Vi du: CALL JOB:PICK-A1 IF IN#(5)=ON)
        if ' IF ' in line:
            main_part, cond_part = line.split(' IF ', 1)
            if not self._eval_condition(cond_part.strip()):
                return
            line = main_part.strip()

        tokens = line.split()
        cmd = tokens[0]

        if cmd in ('MOVJ', 'MOVL', 'MOVC', 'MOVS'):
            if len(tokens) < 2:
                return
            speed = 50.0
            for t in tokens[1:]:
                if '=' in t and (t.startswith('VJ=') or t.startswith('V=') or t.startswith('VL=')):
                    try:
                        speed = float(t.split('=')[1])
                    except ValueError:
                        pass

            # Hỗ trợ đi theo Hệ tọa độ USER [UF#(1)] do Code của người dùng tạo trên GUI:
            # Cú pháp: MOVL UF#(1) (0,0,45) V=60.0  hoặc  MOVL UF#(1) Z=45.0 V=60.0
            if 'UF#(' in line or 'UF#' in line:
                dx_mm, dy_mm, dz_mm = 0.0, 0.0, 0.0
                for t in tokens[1:]:
                    if t.startswith('(') and t.endswith(')'):
                        parts = t.strip('()').split(',')
                        if len(parts) == 3:
                            try:
                                dx_mm, dy_mm, dz_mm = float(parts[0]), float(parts[1]), float(parts[2])
                            except ValueError:
                                pass
                    elif t.startswith('X='):
                        try:
                            dx_mm = float(t.split('=')[1])
                        except ValueError:
                            pass
                    elif t.startswith('Y='):
                        try:
                            dy_mm = float(t.split('=')[1])
                        except ValueError:
                            pass
                    elif t.startswith('Z='):
                        try:
                            dz_mm = float(t.split('=')[1])
                        except ValueError:
                            pass
                self.log_status(
                    f'Thực thi: {cmd} UF#(1) ({dx_mm:.0f},{dy_mm:.0f},{dz_mm:.0f} mm) theo tọa độ Code của bạn!'
                )
                self.move_to_user_frame_offset(dx_mm, dy_mm, dz_mm, speed)
                return

            p_name = tokens[1]
            if p_name not in self.points:
                self.log_status(f'! Lỗi: Chưa có dữ liệu điểm {p_name}!')
                return
            self.log_status(f'Thực thi: {cmd} {p_name} (Speed={speed}%)')
            self.move_to_joints(self.points[p_name], speed)

        elif cmd == 'DOUT':
            # Cu phap: DOUT OT#(1) ON
            ch = self._extract_io_num(line, default=1)
            state_on = ('ON' in tokens)
            self.set_dout(ch, state_on)
            self.log_status(f'Thực thi: DOUT OT#({ch}) {"ON" if state_on else "OFF"}')
            time.sleep(0.12)

        elif cmd == 'PULSE':
            # Cu phap: PULSE OT#(5) T=1.00
            ch = self._extract_io_num(line, default=5)
            t_sec = 1.0
            for t in tokens[1:]:
                if t.startswith('T='):
                    try:
                        t_sec = float(t.split('=')[1])
                    except ValueError:
                        pass
            self.log_status(f'Thực thi: PULSE OT#({ch}) T={t_sec:.2f}s')
            self.pulse_dout(ch, t_sec)
            time.sleep(0.1)

        elif cmd == 'WAIT':
            # Cu phap: WAIT IN#(1)=ON
            ch = self._extract_io_num(line, default=1)
            target_on = ('=OFF' not in line)
            self.log_status(f'Đang chờ tín hiệu: WAIT IN#({ch})={"ON" if target_on else "OFF"}...')
            while rclpy.ok() and not self.stop_requested and (self.din.get(ch, False) != target_on):
                time.sleep(0.05)

        elif cmd == 'TIMER':
            wait_sec = 1.0
            for t in tokens[1:]:
                if '=' in t:
                    try:
                        wait_sec = float(t.split('=')[1])
                    except ValueError:
                        pass
            self.log_status(f'Thực thi: TIMER T={wait_sec:.2f}s')
            elapsed = 0.0
            while elapsed < wait_sec and not self.stop_requested:
                time.sleep(0.05)
                elapsed += 0.05

        elif cmd == 'CALL':
            # Cu phap: CALL JOB:PICK-A1
            sub_name = line.split('JOB:')[-1].strip() if 'JOB:' in line else tokens[-1]
            if sub_name in self.jobs:
                self.log_status(f'CALL JOB:{sub_name}...')
                for sub_line in self.jobs[sub_name]:
                    if self.stop_requested:
                        break
                    self.execute_inform_line(sub_line)

        elif cmd == 'SET':
            if len(tokens) >= 3:
                var_name, val_str = tokens[1], tokens[2]
                if var_name in self.b_vars:
                    self.b_vars[var_name] = int(val_str)

    def _extract_io_num(self, text: str, default: int = 1) -> int:
        if '(' in text and ')' in text:
            inside = text.split('(')[1].split(')')[0]
            try:
                return int(inside)
            except ValueError:
                return default
        return default

    def _eval_condition(self, cond: str) -> bool:
        ch = self._extract_io_num(cond, default=1)
        target_on = ('=ON' in cond)
        return self.din.get(ch, False) == target_on


# =====================================================================
# GIAO DIEN BO LAP TRINH YASKAWA YRC1000micro CHUAN HINH 5, 6, 10, 13, 14
# =====================================================================
class YaskawaAuthenticPendantGUI:
    def __init__(self, node: InformTeachPendantNode):
        self.node = node
        self.node.status_callback = self.set_human_interface_msg

        self.root = tk.Tk()
        if self.node.active_robot == 'R1':
            self.root.title('YASKAWA YRC1000micro Programming Pendant #1 - [ROBOT 1 (R1): Băng tải 1 -> Đĩa xoay]')
        else:
            self.root.title('YASKAWA YRC1000micro Programming Pendant #2 - [ROBOT 2 (R2): Đĩa xoay -> Băng tải 2]')
        # Tu dong lay do phan giai man hinh laptop va dat 2 cua so Teach Pendant
        scr_w = self.root.winfo_screenwidth()
        scr_h = self.root.winfo_screenheight()
        half_w = max(860, scr_w // 2)
        win_h = max(760, scr_h - 65)
        if self.node.active_robot == 'R1':
            pos_x = 0
            pos_y = 0
        else:
            pos_x = scr_w - half_w
            pos_y = 0
        self.root.geometry(f'{half_w}x{win_h}+{pos_x}+{pos_y}')
        self.root.configure(bg='#3a3d42')

        # Con tro dong lenh tren JOB CONTENT (Address Cursor)
        self.cursor_idx = 0
        self.current_screen = 'JOB'    # 'JOB', 'POS_VAR', 'IO', 'WORK_HOME'
        self.selected_p_var = 'P001'
        self.inform_list_open = False
        self.test_start_held = False

        self._build_layout()
        self._refresh_screen()
        self.root.after(80, self._periodic_update)

    def _build_layout(self):
        # Bo cuc doc chuan nua man hinh Laptop 16-inch:
        # Phan tren: Bo lap trinh Yaskawa Programming Pendant
        # Phan duoi: Bang Yaskawa Control Panel I/O (Hinh 14) nam ngang gon gang, khong bao gio bi cat chu!
        main_wrap = tk.Frame(self.root, bg='#3a3d42', padx=6, pady=4)
        main_wrap.pack(fill=tk.BOTH, expand=True)

        # =================================================================
        # PHAN TREN: THÂN BỘ LẬP TRÌNH YASKAWA (PROGRAMMING PENDANT - HÌNH 5)
        # =================================================================
        pendant = tk.Frame(main_wrap, bg='#4a4e54', bd=3, relief=tk.RAISED, padx=8, pady=6)
        pendant.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=(0, 6))

        # --- 1. TOP HOUSING: Mode Switch + [START] + [HOLD] + [EMERGENCY STOP] (Bảng 2 - Trang 4) ---
        top_housing = tk.Frame(pendant, bg='#2c3036', bd=2, relief=tk.GROOVE, padx=10, pady=6)
        top_housing.pack(fill=tk.X, pady=(0, 6))

        # Mode Switch (REMOTE / PLAY / TEACH)
        ms_frame = tk.LabelFrame(
            top_housing, text='Mode Switch ', fg='#d0d7de', bg='#2c3036', font=('Ubuntu', 8, 'bold'), padx=6, pady=2
        )
        ms_frame.pack(side=tk.LEFT, padx=4)
        self.mode_btns = {}
        for m in ('REMOTE', 'PLAY', 'TEACH'):
            b = tk.Button(
                ms_frame,
                text=m,
                width=7,
                font=('Ubuntu', 8, 'bold'),
                command=lambda mode=m: self._on_mode_switch(mode),
            )
            b.pack(side=tk.LEFT, padx=2)
            self.mode_btns[m] = b

        # Nút [START] (Tròn xanh lá - Trang 4)
        self.btn_start = tk.Button(
            top_housing,
            text='● [START]\nBắt đầu chạy',
            bg='#00c853',
            fg='black',
            font=('Ubuntu', 9, 'bold'),
            width=13,
            height=2,
            command=self._on_press_start,
        )
        self.btn_start.pack(side=tk.LEFT, padx=12)

        # Nút [HOLD] (Tròn vàng/nâu - Trang 4)
        self.btn_hold = tk.Button(
            top_housing,
            text='● [HOLD]\nTạm dừng',
            bg='#ffab00',
            fg='black',
            font=('Ubuntu', 9, 'bold'),
            width=13,
            height=2,
            command=self._on_press_hold,
        )
        self.btn_hold.pack(side=tk.LEFT, padx=8)

        # Nút [EMERGENCY STOP] (Đỏ trên nền vàng - Trang 4)
        estop_ring = tk.Frame(top_housing, bg='#ffd600', padx=3, pady=3)
        estop_ring.pack(side=tk.RIGHT, padx=6)
        self.btn_estop = tk.Button(
            estop_ring,
            text='E-STOP\n(Nhấn/Xoay)',
            bg='#d50000',
            fg='white',
            font=('Ubuntu', 9, 'bold'),
            width=12,
            command=self._on_press_estop,
        )
        self.btn_estop.pack()

        # Logo YASKAWA giữa màn hình và đỉnh
        tk.Label(
            pendant, text='Y A S K A W A', fg='white', bg='#4a4e54', font=('Ubuntu', 11, 'bold')
        ).pack()

        # --- 2. MÀN HÌNH CẢM ỨNG 5.7 INCH (5 KHU VỰC CHUẨN HÌNH 6 - TRANG 7) ---
        screen_bezel = tk.Frame(pendant, bg='#1b1f24', bd=4, relief=tk.SUNKEN)
        screen_bezel.pack(fill=tk.BOTH, expand=True, pady=(2, 8))

        # (A) KHU VỰC 1 & 2: Menu area (trên trái) + Status display area (trên phải) (Hình 6 & Bảng 3)
        top_bar_lcd = tk.Frame(screen_bezel, bg='#ced4da', height=28)
        top_bar_lcd.pack(fill=tk.X)

        for menu_txt in ('JOB', 'EDIT', 'DISPLAY', 'UTILITY'):
            tk.Button(
                top_bar_lcd,
                text=menu_txt,
                bg='#adb5bd',
                fg='black',
                font=('Ubuntu', 8, 'bold'),
                relief=tk.RAISED,
                bd=1,
                padx=8,
                command=lambda t=menu_txt: self._on_top_menu_click(t),
            ).pack(side=tk.LEFT, padx=1, pady=1)

        # Status display area icons (Bảng 3 - Trang 7)
        status_icons = tk.Frame(top_bar_lcd, bg='#495057', padx=4)
        status_icons.pack(side=tk.RIGHT, fill=tk.Y)

        self.icon_coord = tk.Label(
            status_icons, text='[Joint]', bg='#1864ab', fg='white', font=('Ubuntu Mono', 9, 'bold'), padx=4
        )
        self.icon_coord.pack(side=tk.LEFT, padx=2, pady=2)

        self.icon_speed = tk.Label(
            status_icons, text='[M] MED', bg='#2b8a3e', fg='white', font=('Ubuntu Mono', 9, 'bold'), padx=4
        )
        self.icon_speed.pack(side=tk.LEFT, padx=2, pady=2)

        self.icon_state = tk.Label(
            status_icons, text='[STOP]', bg='#e67700', fg='white', font=('Ubuntu Mono', 9, 'bold'), padx=4
        )
        self.icon_state.pack(side=tk.LEFT, padx=2, pady=2)

        self.icon_mode = tk.Label(
            status_icons, text='[TEACH]', bg='#364fc7', fg='white', font=('Ubuntu Mono', 9, 'bold'), padx=4
        )
        self.icon_mode.pack(side=tk.LEFT, padx=2, pady=2)

        self.btn_robot_sel = tk.Button(
            status_icons,
            text=f'GROUP: {self.node.active_robot}',
            bg='#d9480f' if self.node.active_robot == 'R1' else '#5f3dc4',
            fg='white',
            font=('Ubuntu Mono', 8, 'bold'),
            padx=4,
            command=self._toggle_active_robot,
        )
        self.btn_robot_sel.pack(side=tk.LEFT, padx=2, pady=1)

        tk.Label(
            status_icons, text='TOOL:00', bg='#212529', fg='#ffd43b', font=('Ubuntu Mono', 9, 'bold'), padx=4
        ).pack(side=tk.LEFT, padx=2, pady=2)

        # Thân giữa màn hình: Main menu area (Cột trái) + General purpose display area (Khu vực chung ở giữa)
        mid_lcd = tk.Frame(screen_bezel, bg='#e9ecef')
        mid_lcd.pack(fill=tk.BOTH, expand=True)

        # (B) KHU VỰC 3: Main menu area (Cột nút dọc bên trái chuẩn Hình 6, 10, 11, 13)
        self.main_menu_col = tk.Frame(mid_lcd, bg='#adb5bd', width=96, bd=1, relief=tk.RAISED)
        self.main_menu_col.pack(side=tk.LEFT, fill=tk.Y)

        self.menu_nav_btns = {}
        for nav_id, nav_label in [
            ('JOB', ' JOB\n(Chương trình)'),
            ('VARIABLE', ' VARIABLE\n(Điểm P000..)'),
            ('IN/OUT', ' IN/OUT\n(Đọc/Ghi I/O)'),
            ('ROBOT', ' ROBOT\n(Work Home)'),
            ('POS_MON', ' POSITION\n(Tọa độ thực)'),
        ]:
            b = tk.Button(
                self.main_menu_col,
                text=nav_label,
                bg='#dee2e6',
                fg='black',
                font=('Ubuntu', 8, 'bold'),
                height=2,
                relief=tk.RAISED,
                command=lambda nid=nav_id: self._switch_screen(nid),
            )
            b.pack(fill=tk.X, padx=3, pady=3)
            self.menu_nav_btns[nav_id] = b

        # Chọn nhanh JOB: MAIN / PICK-A1 / PLACE-B (Trang 16, 17)
        tk.Label(
            self.main_menu_col, text='SELECT JOB:', bg='#adb5bd', fg='#1864ab', font=('Ubuntu', 8, 'bold')
        ).pack(pady=(6, 1))
        for jname in ('MAIN', 'PICK-A1', 'PLACE-B'):
            tk.Button(
                self.main_menu_col,
                text=jname,
                bg='#364fc7' if jname == 'MAIN' else '#ced4da',
                fg='white' if jname == 'MAIN' else 'black',
                font=('Ubuntu Mono', 8, 'bold'),
                command=lambda j=jname: self._select_job(j),
            ).pack(fill=tk.X, padx=4, pady=1)

        # (C) KHU VỰC 4: General purpose display area (Hiển thị JOB CONTENT / POSITION VARIABLE / IN-OUT)
        self.gp_area = tk.Frame(mid_lcd, bg='white')
        self.gp_area.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Header xanh dương đậm của General Purpose Area (Giống hệt Hình 6, 10, 13)
        self.gp_header = tk.Label(
            self.gp_area,
            text='JOB CONTENT    J:MAIN    CONTROL GROUP: R1    S:0000  TOOL: 00',
            bg='#1d3587',
            fg='white',
            font=('Ubuntu Mono', 10, 'bold'),
            anchor='w',
            padx=6,
            pady=4,
        )
        self.gp_header.pack(fill=tk.X)

        # Khung nội dung chính bên trong General Purpose Area
        self.gp_body = tk.Frame(self.gp_area, bg='white')
        self.gp_body.pack(fill=tk.BOTH, expand=True)

        # Listbox hiển thị các dòng lệnh Address + Instruction (Hình 13 - Trang 13)
        self.job_listbox = tk.Listbox(
            self.gp_body,
            bg='#f8f9fa',
            fg='#1d3587',
            selectbackground='#1d3587',
            selectforeground='white',
            font=('Ubuntu Mono', 11, 'bold'),
            activestyle='none',
            relief=tk.FLAT,
        )
        self.job_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4, pady=2)
        self.job_listbox.bind('<<ListboxSelect>>', self._on_listbox_click)

        # Cửa sổ phụ bên phải màn hình LCD: Hiển thị liên tục tọa độ Robot & Hộp thoại INFORM LIST (khi mở)
        self.lcd_right_sub = tk.Frame(self.gp_body, bg='#e9ecef', width=265, bd=1, relief=tk.SOLID)
        self.lcd_right_sub.pack(side=tk.RIGHT, fill=tk.Y, padx=2, pady=2)

        self.live_pos_lbl = tk.Label(
            self.lcd_right_sub,
            text='',
            bg='#f1f3f5',
            fg='#0b7285',
            font=('Ubuntu Mono', 9, 'bold'),
            justify=tk.LEFT,
            anchor='nw',
            padx=6,
            pady=4,
        )
        self.live_pos_lbl.pack(fill=tk.X)

        # Khung INFORM LIST (Ẩn/Hiện khi bấm nút [INFORM LIST] - Trang 13)
        self.inform_popup = tk.LabelFrame(
            self.lcd_right_sub,
            text='INFORM LIST (Chọn lệnh -> Buffer) ',
            bg='#dee2e6',
            fg='#1d3587',
            font=('Ubuntu', 8, 'bold'),
        )
        for grp_txt, sample_cmd in [
            ('USER: MOVL UF#(1) Z=45', 'MOVL UF#(1) (0,0,45) V=60.0'),
            ('USER: MOVL UF#(1) Z=0', 'MOVL UF#(1) (0,0,0) V=25.0'),
            ('MOTION: MOVJ', 'MOVJ P001 VJ=50.00'),
            ('MOTION: MOVL', 'MOVL P002 V=25.00 PL=0'),
            ('IN/OUT: DOUT ON', 'DOUT OT#(1) ON'),
            ('IN/OUT: DOUT OFF', 'DOUT OT#(1) OFF'),
            ('IN/OUT: WAIT IN#1', 'WAIT IN#(1)=ON'),
            ('IN/OUT: PULSE OT#3', 'PULSE OT#(3) T=0.10'),
            ('CONTROL: TIMER', 'TIMER T=0.50'),
            ('CONTROL: CALL JOB', 'CALL JOB:PICK-USER-UF1'),
        ]:
            tk.Button(
                self.inform_popup,
                text=grp_txt,
                bg='white',
                fg='#1d3587',
                font=('Ubuntu Mono', 8, 'bold'),
                anchor='w',
                command=lambda c=sample_cmd: self._set_buffer_text(c),
            ).pack(fill=tk.X, padx=3, pady=1)

        # Dòng đệm Buffer Line (Trang 13 - Hình 6: Ô nhập/sửa lệnh nằm ngay dưới vùng lệnh)
        buf_frame = tk.Frame(self.gp_area, bg='#ced4da', bd=1, relief=tk.SOLID, pady=3)
        buf_frame.pack(fill=tk.X, padx=4, pady=2)
        tk.Label(
            buf_frame, text='Buffer Line:', bg='#ced4da', fg='black', font=('Ubuntu Mono', 9, 'bold')
        ).pack(side=tk.LEFT, padx=4)
        self.buffer_entry = tk.Entry(
            buf_frame,
            bg='white',
            fg='#1d3587',
            font=('Ubuntu Mono', 11, 'bold'),
        )
        self.buffer_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.buffer_entry.insert(0, 'MOVJ P001 VJ=50.00')

        # (D) KHU VỰC 5: Human interface display area (Dưới đáy màn hình LCD - Hình 6 Trang 7)
        bottom_bar_lcd = tk.Frame(screen_bezel, bg='#adb5bd', bd=1, relief=tk.RAISED)
        bottom_bar_lcd.pack(fill=tk.X, side=tk.BOTTOM)
        tk.Button(
            bottom_bar_lcd,
            text='Main Menu',
            bg='#1d3587',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            command=self._toggle_main_menu,
        ).pack(side=tk.LEFT, padx=3, pady=2)

        self.human_msg_lbl = tk.Label(
            bottom_bar_lcd,
            text='! Sẵn sàng vận hành. Chọn hệ tọa độ [COORD] và nhấn giữ các phím [Axis Key] để điều khiển.',
            bg='#e9ecef',
            fg='#c92a2a',
            font=('Ubuntu', 9, 'bold'),
            anchor='w',
            padx=8,
        )
        self.human_msg_lbl.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3, pady=2)

        # --- 3. BÀN PHÍM CỨNG BỘ LẬP TRÌNH (CÁC NÚT NHẤN CHUẨN BẢNG 2 - TRANG 4, 5, 6) ---
        hw_keys = tk.Frame(pendant, bg='#343a40', bd=2, relief=tk.GROOVE, padx=4, pady=4)
        hw_keys.pack(fill=tk.X)

        # Chia 3 cụm bàn phím cứng: Trái (Điều hướng & Servo), Giữa (12 phím Axis Key + Speed/Coord), Phải (Lập trình INFORM)
        col_nav = tk.Frame(hw_keys, bg='#343a40')
        col_nav.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 4))

        col_axis = tk.Frame(hw_keys, bg='#343a40')
        col_axis.pack(side=tk.LEFT, fill=tk.Y, padx=4)

        col_prog = tk.Frame(hw_keys, bg='#343a40')
        col_prog.pack(side=tk.RIGHT, fill=tk.Y, padx=(4, 0))

        # === CỤM 1 (TRÁI): [SERVO ON READY], [Enable Switch], Cursor Keys, [SELECT], [CANCEL] ===
        tk.Label(
            col_nav, text='NGUỒN SERVO & ĐIỀU HƯỚNG', fg='#ffd43b', bg='#343a40', font=('Ubuntu', 8, 'bold')
        ).pack(pady=(0, 2))

        srv_row = tk.Frame(col_nav, bg='#343a40')
        srv_row.pack(fill=tk.X, pady=2)
        self.btn_servo_ready = tk.Button(
            srv_row,
            text='SERVO ON\nREADY',
            bg='#00e676',
            fg='black',
            font=('Ubuntu', 8, 'bold'),
            width=9,
            command=self._toggle_servo_ready,
        )
        self.btn_servo_ready.pack(side=tk.LEFT, padx=1)

        self.btn_enable_sw = tk.Button(
            srv_row,
            text='Enable Switch\n(Đè nhẹ: ON)',
            bg='#1864ab',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            width=12,
            command=self._toggle_enable_switch,
        )
        self.btn_enable_sw.pack(side=tk.LEFT, padx=1)

        # Cụm phím điều hướng: [MAIN MENU], [AREA], [PAGE], [SELECT], [CANCEL] + Cursor UP/DOWN
        nav_grid = tk.Frame(col_nav, bg='#343a40', pady=2)
        nav_grid.pack()

        tk.Button(
            nav_grid, text='MAIN\nMENU', width=5, bg='#495057', fg='white', font=('Ubuntu', 7, 'bold'),
            command=self._toggle_main_menu
        ).grid(row=0, column=0, padx=1, pady=1)
        tk.Button(
            nav_grid, text='▲\nUP', width=5, bg='#212529', fg='#ffd43b', font=('Ubuntu', 8, 'bold'),
            command=lambda: self._move_cursor(-1)
        ).grid(row=0, column=1, padx=1, pady=1)
        tk.Button(
            nav_grid, text='PAGE\n', width=5, bg='#495057', fg='white', font=('Ubuntu', 7, 'bold'),
            command=self._cycle_screens
        ).grid(row=0, column=2, padx=1, pady=1)

        tk.Button(
            nav_grid, text='CANCEL', width=5, bg='#495057', fg='white', font=('Ubuntu', 7, 'bold'),
            command=lambda: self.inform_popup.pack_forget()
        ).grid(row=1, column=0, padx=1, pady=1)
        tk.Button(
            nav_grid, text='▼\nDOWN', width=5, bg='#212529', fg='#ffd43b', font=('Ubuntu', 8, 'bold'),
            command=lambda: self._move_cursor(+1)
        ).grid(row=1, column=1, padx=1, pady=1)
        tk.Button(
            nav_grid, text='SELECT\n', width=5, bg='#1864ab', fg='white', font=('Ubuntu', 7, 'bold'),
            command=self._on_press_select
        ).grid(row=1, column=2, padx=1, pady=1)

        # === CỤM 2 (GIỮA): [COORD], [MANUAL SPEED: FAST/SLOW], [HIGH SPEED], 12 PHÍM [Axis Key] ===
        tk.Label(
            col_axis, text='HỆ TỌA ĐỘ & AXIS KEYS (NHẤN GIỮ)', fg='#74c0fc', bg='#343a40', font=('Ubuntu', 8, 'bold')
        ).pack(pady=(0, 2))

        top_axis_row = tk.Frame(col_axis, bg='#343a40')
        top_axis_row.pack(fill=tk.X, pady=2)

        tk.Button(
            top_axis_row,
            text='[COORD]\nJnt/Cart/Tl/Usr',
            bg='#fcc419',
            fg='black',
            font=('Ubuntu', 8, 'bold'),
            command=self._on_press_coord,
        ).pack(side=tk.LEFT, padx=1)

        tk.Button(
            top_axis_row,
            text='▲FAST',
            bg='#495057',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            command=lambda: self._change_manual_speed(+1),
        ).pack(side=tk.LEFT, padx=1)

        tk.Button(
            top_axis_row,
            text='▼SLOW',
            bg='#495057',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            command=lambda: self._change_manual_speed(-1),
        ).pack(side=tk.LEFT, padx=1)

        btn_hspd = tk.Button(
            top_axis_row,
            text='[HIGH SPD]',
            bg='#212529',
            fg='#fcc419',
            font=('Ubuntu', 8, 'bold'),
        )
        btn_hspd.pack(side=tk.LEFT, padx=1)
        btn_hspd.bind('<ButtonPress-1>', lambda e: self._set_high_speed(True))
        btn_hspd.bind('<ButtonRelease-1>', lambda e: self._set_high_speed(False))
        btn_hspd.bind('<Leave>', lambda e: self._set_high_speed(False))

        # Lưới 12 phím [Axis Key] chuẩn Hình trang 5:
        axis_grid = tk.Frame(col_axis, bg='#343a40', pady=2)
        axis_grid.pack()

        axis_pairs = [
            (0, 0, 'X-\nS-', 'X+\nS+', 3, 'Rx-\nR-', 'Rx+\nR+'),
            (1, 1, 'Y-\nL-', 'Y+\nL+', 4, 'Ry-\nB-', 'Ry+\nB+'),
            (2, 2, 'Z-\nU-', 'Z+\nU+', 5, 'Rz-\nT-', 'Rz+\nT+'),
        ]

        for r, l_idx, ln_txt, lp_txt, r_idx, rn_txt, rp_txt in axis_pairs:
            for col_offset, ax_idx, neg_label, pos_label in [
                (0, l_idx, ln_txt, lp_txt),
                (2, r_idx, rn_txt, rp_txt),
            ]:
                b_neg = tk.Button(
                    axis_grid,
                    text=neg_label,
                    width=5,
                    bg='#212529',
                    fg='white',
                    activebackground='#fcc419',
                    activeforeground='black',
                    font=('Ubuntu Mono', 8, 'bold'),
                    bd=2,
                )
                b_neg.grid(row=r, column=col_offset, padx=1, pady=1)
                b_neg.bind('<ButtonPress-1>', lambda e, i=ax_idx: self.node.start_continuous_jog(i, -1.0))
                b_neg.bind('<ButtonRelease-1>', lambda e: self.node.stop_continuous_jog())
                b_neg.bind('<Leave>', lambda e: self.node.stop_continuous_jog())

                b_pos = tk.Button(
                    axis_grid,
                    text=pos_label,
                    width=5,
                    bg='#212529',
                    fg='white',
                    activebackground='#fcc419',
                    activeforeground='black',
                    font=('Ubuntu Mono', 8, 'bold'),
                    bd=2,
                )
                b_pos.grid(row=r, column=col_offset + 1, padx=1, pady=1)
                b_pos.bind('<ButtonPress-1>', lambda e, i=ax_idx: self.node.start_continuous_jog(i, +1.0))
                b_pos.bind('<ButtonRelease-1>', lambda e: self.node.stop_continuous_jog())
                b_pos.bind('<Leave>', lambda e: self.node.stop_continuous_jog())

        # === CỤM 3 (PHẢI): NHÓM NÚT LẬP TRÌNH CHUẨN TRANG 6 ([INFORM LIST], [FWD], [BWD], [TEST START], [INSERT], [MODIFY], [DELETE], [ENTER]) ===
        tk.Label(
            col_prog, text='NHÓM NÚT LẬP TRÌNH (TRANG 6)', fg='#69db7c', bg='#343a40', font=('Ubuntu', 8, 'bold')
        ).pack(pady=(0, 2))

        prog_grid = tk.Frame(col_prog, bg='#343a40')
        prog_grid.pack()

        # Hàng 0: [SHIFT], [INTERLOCK], [INFORM LIST]
        self.btn_shift = tk.Button(
            prog_grid, text='[SHIFT]', width=7, bg='#495057', fg='white', font=('Ubuntu', 8, 'bold'),
            command=self._toggle_shift
        )
        self.btn_shift.grid(row=0, column=0, padx=1, pady=1)

        self.btn_interlock = tk.Button(
            prog_grid, text='[INTERLOCK]', width=9, bg='#495057', fg='white', font=('Ubuntu', 8, 'bold'),
            command=self._toggle_interlock
        )
        self.btn_interlock.grid(row=0, column=1, padx=1, pady=1)

        tk.Button(
            prog_grid, text='[INFORM LIST]', width=10, bg='#1864ab', fg='white', font=('Ubuntu', 8, 'bold'),
            command=self._toggle_inform_list
        ).grid(row=0, column=2, padx=1, pady=1)

        # Hàng 1: [FWD], [BWD], [TEST START] (Trang 6)
        tk.Button(
            prog_grid, text='[FWD]\nTiến 1 bước', width=7, bg='#2b8a3e', fg='white', font=('Ubuntu', 8, 'bold'),
            command=self._on_press_fwd
        ).grid(row=1, column=0, padx=1, pady=1)

        tk.Button(
            prog_grid, text='[BWD]\nLùi 1 bước', width=9, bg='#495057', fg='white', font=('Ubuntu', 8, 'bold'),
            command=self._on_press_bwd
        ).grid(row=1, column=1, padx=1, pady=1)

        btn_test = tk.Button(
            prog_grid, text='[TEST START]\n(Giữ chạy thử)', width=10, bg='#e67700', fg='white', font=('Ubuntu', 8, 'bold'),
        )
        btn_test.grid(row=1, column=2, padx=1, pady=1)
        btn_test.bind('<ButtonPress-1>', lambda e: self._start_test_run())
        btn_test.bind('<ButtonRelease-1>', lambda e: self._stop_test_run())

        # Hàng 2: [INSERT], [MODIFY], [DELETE], [ENTER] (Trang 6 & Trang 14)
        tk.Button(
            prog_grid, text='[INSERT]\nChèn dòng', width=7, bg='#364fc7', fg='white', font=('Ubuntu', 8, 'bold'),
            command=self._on_press_insert
        ).grid(row=2, column=0, padx=1, pady=1)

        tk.Button(
            prog_grid, text='[MODIFY]\nSửa/Dạy điểm', width=9, bg='#fcc419', fg='black', font=('Ubuntu', 8, 'bold'),
            command=self._on_press_modify
        ).grid(row=2, column=1, padx=1, pady=1)

        tk.Button(
            prog_grid, text='[DELETE]\nXóa dòng', width=10, bg='#c92a2a', fg='white', font=('Ubuntu', 8, 'bold'),
            command=self._on_press_delete
        ).grid(row=2, column=2, padx=1, pady=1)

        tk.Button(
            prog_grid, text='⏎ [ENTER] (Xác nhận ghi / Dạy điểm P)', bg='#00c853', fg='black',
            font=('Ubuntu', 8, 'bold'), command=self._on_press_modify
        ).grid(row=3, column=0, columnspan=3, sticky='we', padx=1, pady=2)

        # =================================================================
        # PHẦN DƯỚI: YASKAWA CONTROL PANEL I/O NẰM NGANG GỌN GÀNG (HÌNH 14 - TRANG 17)
        # Vừa khít 100% trong nửa màn hình Laptop 16-inch mà không bị cắt bất kỳ chữ nào!
        # =================================================================
        panel = tk.Frame(main_wrap, bg='#d0d7de', bd=2, relief=tk.RAISED, padx=8, pady=4)
        panel.pack(side=tk.BOTTOM, fill=tk.X)

        top_cp_hdr = tk.Frame(panel, bg='#d0d7de')
        top_cp_hdr.pack(fill=tk.X, pady=(0, 3))
        tk.Label(
            top_cp_hdr,
            text='YASKAWA CONTROL PANEL (Hình 14 - Trang 17: Bảng Công Tắc I4..I7 & Đèn Báo Q4..Q7)',
            fg='#1d3587',
            bg='#d0d7de',
            font=('Ubuntu', 9, 'bold'),
        ).pack(side=tk.LEFT)

        self.btn_user_lock = tk.Button(
            top_cp_hdr,
            text='BÁM HỆ USER ĐỐI TƯỢNG (0ms TRỄ): OFF',
            bg='#e67700',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            padx=6,
            command=self._toggle_user_object_lock,
        )
        self.btn_user_lock.pack(side=tk.RIGHT, padx=4)

        tk.Button(
            top_cp_hdr,
            text='Về Work Home (0°)',
            bg='#2b8a3e',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            padx=6,
            command=lambda: threading.Thread(
                target=self.node.move_to_joints,
                args=(self.node.points['P000'], self.node.get_effective_speed()),
                daemon=True,
            ).start(),
        ).pack(side=tk.RIGHT, padx=4)

        tk.Button(
            top_cp_hdr,
            text='Bật/Tắt OUT#1 (Hút)',
            bg='#1864ab',
            fg='white',
            font=('Ubuntu', 8, 'bold'),
            padx=6,
            command=lambda: self.node.set_dout(1, not self.node.dout[1]),
        ).pack(side=tk.RIGHT, padx=4)

        cp_cols = tk.Frame(panel, bg='#d0d7de')
        cp_cols.pack(fill=tk.X)

        # Cột Trái Control Panel: INPUT SWITCHES (IN#1 + Công tắc I4..I7)
        in_box = tk.LabelFrame(
            cp_cols, text='INPUT SWITCHES (I4..I7 -> IN#5..8) ', bg='#d0d7de', fg='black', font=('Ubuntu', 8, 'bold'), padx=6, pady=3
        )
        in_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 4))

        self.in1_panel_lbl = tk.Label(
            in_box, text='● IN#1 (Cảm biến băng tải): OFF', bg='#212529', fg='#ff6b6b', font=('Ubuntu Mono', 8, 'bold'), pady=3
        )
        self.in1_panel_lbl.pack(fill=tk.X, pady=(0, 3))

        sw_grid = tk.Frame(in_box, bg='#d0d7de')
        sw_grid.pack(fill=tk.X)

        self.sw_btns = {}
        for idx_sw, (sw_label, in_ch, desc) in enumerate([
            ('Công tắc I4 (IN#5)', 5, 'Gắp A1'),
            ('Công tắc I5 (IN#6)', 6, 'Gắp A2'),
            ('Công tắc I6 (IN#7)', 7, 'Gắp A3'),
            ('Công tắc I7 (IN#8)', 8, 'OFF:B / ON:C'),
        ]):
            r_sw = idx_sw // 2
            c_sw = idx_sw % 2
            cell = tk.Frame(sw_grid, bg='#d0d7de')
            cell.grid(row=r_sw, column=c_sw, sticky='w', padx=3, pady=1)
            b = tk.Button(
                cell,
                text=f'{sw_label}: OFF',
                bg='#495057',
                fg='white',
                font=('Ubuntu', 8, 'bold'),
                width=16,
                command=lambda c=in_ch: self._toggle_input_switch(c),
            )
            b.pack(side=tk.LEFT)
            tk.Label(cell, text=desc, bg='#d0d7de', fg='#212529', font=('Ubuntu', 8, 'bold')).pack(side=tk.LEFT, padx=3)
            self.sw_btns[in_ch] = b

        # Cột Phải Control Panel: OUTPUT LAMPS (OUT#1 + Q4..Q7)
        out_box = tk.LabelFrame(
            cp_cols, text='OUTPUT LAMPS (Q4..Q7 -> OUT#5..8) ', bg='#d0d7de', fg='black', font=('Ubuntu', 8, 'bold'), padx=6, pady=3
        )
        out_box.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(4, 0))

        lamp_grid = tk.Frame(out_box, bg='#d0d7de')
        lamp_grid.pack(fill=tk.X)

        self.out_lamps = {}
        for idx_l, (out_ch, lamp_name) in enumerate([
            (1, 'OUT#1 (Giác hút / Magnet)'),
            (5, 'Đèn Q4 (OUT#5 - Báo thả B)'),
            (6, 'Đèn Q5 (OUT#6 - Báo thả C)'),
            (7, 'Đèn Q6 (OUT#7)'),
            (8, 'Đèn Q7 (OUT#8)'),
        ]):
            r_l = idx_l // 2
            c_l = idx_l % 2
            col_span = 2 if idx_l == 0 else 1
            if idx_l > 0:
                r_l = (idx_l + 1) // 2
                c_l = (idx_l + 1) % 2
            lbl = tk.Label(
                lamp_grid,
                text=f'○ {lamp_name}: OFF',
                bg='#212529',
                fg='#adb5bd',
                font=('Ubuntu Mono', 8, 'bold'),
                anchor='w',
                padx=6,
                pady=3,
            )
            lbl.grid(row=r_l, column=c_l, columnspan=col_span, sticky='we', padx=2, pady=1)
            self.out_lamps[out_ch] = lbl
        lamp_grid.columnconfigure(0, weight=1)
        lamp_grid.columnconfigure(1, weight=1)

        self._on_mode_switch('TEACH')

    # =================================================================
    # CÁC HÀM XỬ LÝ MÀN HÌNH & NÚT NHẤN CHUẨN TÀI LIỆU ĐHBK TPHCM
    # =================================================================
    def _on_mode_switch(self, mode: str):
        self.node.mode_switch = mode
        for m, b in self.mode_btns.items():
            if m == mode:
                b.config(bg='#fcc419' if m == 'TEACH' else '#00c853', fg='black')
            else:
                b.config(bg='#495057', fg='white')
        self.icon_mode.config(text=f'[{mode}]')
        if mode == 'TEACH':
            self.set_human_interface_msg('Chế độ [TEACH]: Điều khiển bằng tay (Axis Key), dạy điểm [MODIFY]+[ENTER], chạy thử [TEST START].')
        elif mode == 'PLAY':
            self.set_human_interface_msg('Chế độ [PLAY]: Nhấn [SERVO ON READY] rồi nhấn nút xanh [START] để chạy tự động JOB.')
        else:
            self.set_human_interface_msg('Chế độ [REMOTE]: Vận hành bằng tín hiệu ngoài.')

    def _toggle_servo_ready(self):
        if self.node.estop_active:
            self.set_human_interface_msg('! Đang dừng khẩn cấp [E-STOP]! Hãy nhấn vào nút đỏ E-STOP để xoay mở khóa trước.')
            return
        self.node.servo_ready = not self.node.servo_ready
        self._update_servo_buttons()

    def _toggle_enable_switch(self):
        self.node.enable_switch_on = not self.node.enable_switch_on
        self._update_servo_buttons()

    def _update_servo_buttons(self):
        if self.node.is_servo_powered:
            self.btn_servo_ready.config(text='SERVO ON\nREADY', bg='#00e676', fg='black')
        else:
            self.btn_servo_ready.config(text='SERVO OFF\n(Bấm Bật)', bg='#495057', fg='white')

        if self.node.enable_switch_on:
            self.btn_enable_sw.config(text='Enable Switch\n(Đè nhẹ: ON)', bg='#1864ab', fg='white')
        else:
            self.btn_enable_sw.config(text='Enable Switch\n(Buông: OFF)', bg='#495057', fg='white')

    def _on_press_estop(self):
        self.node.estop_active = not self.node.estop_active
        if self.node.estop_active:
            self.node.stop_requested = True
            self.node.is_running_job = False
            self.node.stop_continuous_jog()
            self.btn_estop.config(text='ĐANG E-STOP!\n(Bấm để Xoay mở)', bg='#ff1744')
            self.icon_state.config(text='[E-STOP]', bg='#d50000')
            self.set_human_interface_msg('! EMERGENCY STOP: Đã ngắt khẩn cấp nguồn động cơ servo! Bấm lại E-STOP để xoay thoát.')
        else:
            self.btn_estop.config(text='E-STOP\n(Nhấn/Xoay)', bg='#d50000')
            self.icon_state.config(text='[STOP]', bg='#e67700')
            self.set_human_interface_msg('Đã xoay mở khóa E-STOP. Nhấn [SERVO ON READY] để bật lại nguồn servo.')
        self._update_servo_buttons()

    def _toggle_active_robot(self):
        self.node.stop_continuous_jog()
        self.node.active_robot = 'R2' if self.node.active_robot == 'R1' else 'R1'
        self.node.jog_target_joints = list(self.node.current_joints)
        if self.node.active_robot == 'R1':
            self.btn_robot_sel.config(text='GROUP: R1', bg='#d9480f')
        else:
            self.btn_robot_sel.config(text='GROUP: R2', bg='#5f3dc4')
        self._refresh_screen()
        self.set_human_interface_msg(
            f'Đã chuyển quyền điều khiển Teach Pendant sang CONTROL GROUP: {self.node.active_robot}!'
        )

    def _on_press_coord(self):
        modes = self.node.COORD_MODES
        idx = (modes.index(self.node.coord_mode) + 1) % len(modes)
        self.node.coord_mode = modes[idx]
        short_map = {
            'JOINT': '[Joint]',
            'CARTESIAN': '[Cart]',
            'TOOL': '[Tool]',
            'USER': '[User]',
        }
        self.icon_coord.config(text=short_map[self.node.coord_mode])
        self.set_human_interface_msg(
            f'Đã chuyển hệ tọa độ [COORD] sang: {self.node.coord_mode} (Joint -> Cartesian -> Tool -> User).'
        )

    def _change_manual_speed(self, delta: int):
        self.node.speed_idx = max(0, min(3, self.node.speed_idx + delta))
        self.icon_speed.config(text=self.node.speed_icons[self.node.speed_idx])
        self.set_human_interface_msg(
            f'Tốc độ điều khiển bằng tay [MANUAL SPEED]: {self.node.speed_icons[self.node.speed_idx]} ({self.node.speed_values[self.node.speed_idx]:.0f}%)'
        )

    def _set_high_speed(self, active: bool):
        self.node.high_speed_held = active
        if active:
            self.icon_speed.config(text='[H] HIGH!', bg='#d9480f')
        else:
            self.icon_speed.config(text=self.node.speed_icons[self.node.speed_idx], bg='#2b8a3e')

    def _toggle_main_menu(self):
        if self.main_menu_col.winfo_ismapped():
            self.main_menu_col.pack_forget()
        else:
            self.main_menu_col.pack(side=tk.LEFT, fill=tk.Y, before=self.gp_area)

    def _switch_screen(self, screen_id: str):
        self.current_screen = screen_id
        self.cursor_idx = 0
        self._refresh_screen()

    def _cycle_screens(self):
        order = ['JOB', 'VARIABLE', 'IN/OUT', 'ROBOT']
        idx = (order.index(self.current_screen) + 1) % len(order) if self.current_screen in order else 0
        self._switch_screen(order[idx])

    def _select_job(self, job_name: str):
        self.node.current_job_name = job_name
        self.current_screen = 'JOB'
        self.cursor_idx = 0
        self._refresh_screen()
        self.set_human_interface_msg(f'Đã mở chương trình JOB: {job_name} (Trang 16, 17).')

    def _refresh_screen(self):
        self.job_listbox.delete(0, tk.END)

        if self.current_screen == 'JOB':
            jname = self.node.current_job_name
            self.gp_header.config(
                text=f'JOB CONTENT    J:{jname}    CONTROL GROUP: {self.node.active_robot}    S:{self.cursor_idx:04d}  TOOL: 00'
            )
            lines = self.node.jobs[jname]
            for i, line in enumerate(lines):
                self.job_listbox.insert(tk.END, f' {i:04d}   {line}')
            if lines:
                self.cursor_idx = min(self.cursor_idx, len(lines) - 1)
                self.job_listbox.selection_set(self.cursor_idx)
                self.job_listbox.see(self.cursor_idx)

        elif self.current_screen == 'VARIABLE':
            # Màn hình POSITION VARIABLE chuẩn Hình trang 11 (#P000..#P012)
            self.gp_header.config(
                text='POSITION VARIABLE          TOOL: 00      (Chọn điểm -> Nhấn [MODIFY]+[ENTER] để lưu)'
            )
            for idx, (p_key, q_vals) in enumerate(sorted(self.node.points.items())):
                T_p, _ = fk_motomini(q_vals)
                x_m, y_m, z_m = T_p[0, 3] * 1000.0, T_p[1, 3] * 1000.0, T_p[2, 3] * 1000.0
                rx, ry, rz = rot_to_rpy_deg(T_p[:3, :3])
                self.job_listbox.insert(
                    tk.END,
                    f' #{p_key}  ROBOT | X:{x_m:6.1f}  Y:{y_m:6.1f}  Z:{z_m:5.1f} | Rx:{rx:5.1f}° Ry:{ry:5.1f}° Rz:{rz:5.1f}°',
                )
            self.job_listbox.selection_set(self.cursor_idx)

        elif self.current_screen == 'IN/OUT':
            # Màn hình GENERAL PURPOSE OUTPUT chuẩn Hình 10 (Trang 9)
            self.gp_header.config(
                text='GENERAL PURPOSE OUTPUT / INPUT     (Di chuyển con trỏ -> Nhấn [SELECT] để ON/OFF)'
            )
            out_names = {
                1: 'MAGNET / VACUUM',
                2: 'LASER',
                3: 'SPARE_3',
                4: 'SPARE_4',
                5: 'LAMP_Q4 (PLACE B)',
                6: 'LAMP_Q5 (PLACE C)',
                7: 'LAMP_Q6',
                8: 'LAMP_Q7',
            }
            for ch in range(1, 9):
                st = '● ON ' if self.node.dout[ch] else '○ OFF'
                self.job_listbox.insert(
                    tk.END, f' OUT#{ch:04d}   #1001{ch-1}   {st}   |  {out_names[ch]}'
                )
            self.job_listbox.selection_set(self.cursor_idx)

        elif self.current_screen in ('ROBOT', 'POS_MON'):
            # Màn hình WORK HOME POS chuẩn Trang 10 (Mục 7)
            self.gp_header.config(
                text='WORK HOME POSITION (Mục 7 - Trang 10: Nhấn [FWD] để đưa tất cả các khớp về 0°)'
            )
            axes = ['S', 'L', 'U', 'R', 'B', 'T']
            for i in range(6):
                curr_deg = math.degrees(self.node.current_joints[i])
                self.job_listbox.insert(
                    tk.END, f' Trục {axes[i]} : Hiện tại = {curr_deg:7.2f}°   --->   Work Home Pos =   0.00°'
                )

    def _on_listbox_click(self, event):
        sel = self.job_listbox.curselection()
        if not sel:
            return
        self.cursor_idx = sel[0]
        if self.current_screen == 'JOB':
            jname = self.node.current_job_name
            line = self.node.jobs[jname][self.cursor_idx]
            self.buffer_entry.delete(0, tk.END)
            self.buffer_entry.insert(0, line)
        elif self.current_screen == 'VARIABLE':
            p_keys = sorted(self.node.points.keys())
            if self.cursor_idx < len(p_keys):
                self.selected_p_var = p_keys[self.cursor_idx]

    def _move_cursor(self, delta: int):
        count = self.job_listbox.size()
        if count == 0:
            return
        self.cursor_idx = max(0, min(count - 1, self.cursor_idx + delta))
        self.job_listbox.selection_clear(0, tk.END)
        self.job_listbox.selection_set(self.cursor_idx)
        self.job_listbox.see(self.cursor_idx)
        self._on_listbox_click(None)

    def _on_press_select(self):
        if self.current_screen == 'IN/OUT':
            ch = self.cursor_idx + 1
            if 1 <= ch <= 8:
                self.node.set_dout(ch, not self.node.dout[ch])
                self._refresh_screen()
                self.set_human_interface_msg(
                    f'Đã đảo trạng thái OUT#{ch:04d} thành {"ON" if self.node.dout[ch] else "OFF"} (Hình 10 - Trang 9).'
                )
        else:
            self.buffer_entry.focus_set()

    def _toggle_inform_list(self):
        if self.inform_popup.winfo_ismapped():
            self.inform_popup.pack_forget()
        else:
            self.inform_popup.pack(fill=tk.X, pady=4)
            self.set_human_interface_msg('Đã mở [INFORM LIST] (Trang 13). Chọn lệnh để đưa vào dòng đệm Buffer Line.')

    def _set_buffer_text(self, cmd_text: str):
        self.buffer_entry.delete(0, tk.END)
        self.buffer_entry.insert(0, cmd_text)
        self.set_human_interface_msg(f'Đã đưa lệnh "{cmd_text}" vào Buffer Line. Nhấn [INSERT] hoặc [MODIFY] + [ENTER].')

    def _on_press_insert(self):
        """Trang 14: Nhấn [INSERT] + [ENTER] để chèn câu lệnh từ dòng buffer vào địa chỉ liền sau con trỏ."""
        if self.current_screen != 'JOB':
            self._switch_screen('JOB')
        new_cmd = self.buffer_entry.get().strip().upper()
        if not new_cmd:
            return
        jname = self.node.current_job_name
        lines = self.node.jobs[jname]
        insert_pos = min(len(lines), self.cursor_idx + 1)
        lines.insert(insert_pos, new_cmd)
        self.cursor_idx = insert_pos
        self._refresh_screen()
        self.set_human_interface_msg(f'Đã chèn (INSERT) lệnh "{new_cmd}" vào địa chỉ {insert_pos:04d} (Trang 14).')

    def _on_press_modify(self):
        """
        Trang 11 & Trang 14:
          - Nếu đang ở màn hình VARIABLE: Lưu tọa độ góc khớp hiện tại của robot vào điểm #Pxxx đang chọn.
          - Nếu đang ở màn hình JOB: Thay thế câu lệnh tại vị trí con trỏ bằng câu lệnh trên dòng Buffer Line,
            đồng thời nếu câu lệnh có chứa biến điểm Pxxx thì cập nhật vị trí hiện tại vào Pxxx!
        """
        if self.current_screen == 'VARIABLE':
            p_keys = sorted(self.node.points.keys())
            p_name = p_keys[self.cursor_idx]
            self.node.points[p_name] = list(self.node.current_joints)
            self._refresh_screen()
            self.set_human_interface_msg(f' [MODIFY + ENTER] Đã lưu vị trí hiện tại của robot vào điểm #{p_name} (Trang 11)!')
        elif self.current_screen == 'JOB':
            new_cmd = self.buffer_entry.get().strip().upper()
            jname = self.node.current_job_name
            lines = self.node.jobs[jname]
            if 0 <= self.cursor_idx < len(lines) and new_cmd:
                lines[self.cursor_idx] = new_cmd
                # Neu dong lenh chua P00x -> cap nhat luon vi tri day diem
                for tok in new_cmd.split():
                    if tok in self.node.points:
                        self.node.points[tok] = list(self.node.current_joints)
                self._refresh_screen()
                self.set_human_interface_msg(f' [MODIFY + ENTER] Đã cập nhật dòng {self.cursor_idx:04d}: "{new_cmd}" (Trang 14).')

    def _on_press_delete(self):
        """Trang 14: Nhấn [DELETE] + [ENTER] để xóa câu lệnh tại vị trí con trỏ."""
        if self.current_screen != 'JOB':
            return
        jname = self.node.current_job_name
        lines = self.node.jobs[jname]
        if len(lines) <= 2:
            return
        if 0 <= self.cursor_idx < len(lines):
            removed = lines.pop(self.cursor_idx)
            if removed in ('NOP', 'END'):
                lines.insert(self.cursor_idx, removed)
                self.set_human_interface_msg('! Không thể xóa lệnh NOP hoặc END của chương trình.')
                return
            self.cursor_idx = max(0, self.cursor_idx - 1)
            self._refresh_screen()
            self.set_human_interface_msg(f'Đã xóa (DELETE) câu lệnh "{removed}" (Trang 14).')

    def _on_press_fwd(self):
        """
        Trang 6, 10, 12:
          - Trên màn hình ROBOT (Work Home Pos): Đưa robot về vị trí Home (0°).
          - Trên màn hình VARIABLE: Di chuyển robot tới điểm #Pxxx đang chọn.
          - Trên màn hình JOB: Thực thi dòng lệnh tại con trỏ và tiến con trỏ xuống 1 bước.
        """
        if self.node.is_running_job:
            return
        self.node.stop_requested = False

        if self.current_screen == 'ROBOT':
            self.set_human_interface_msg('[FWD] Đang đưa robot về vị trí Work Home Pos (0°)...')
            threading.Thread(
                target=self.node.move_to_joints,
                args=(self.node.points['P000'], self.node.get_effective_speed()),
                daemon=True,
            ).start()
        elif self.current_screen == 'VARIABLE':
            p_keys = sorted(self.node.points.keys())
            p_name = p_keys[self.cursor_idx]
            self.set_human_interface_msg(f'[FWD] Đang di chuyển robot đến điểm #{p_name} (Trang 12)...')
            threading.Thread(
                target=self.node.move_to_joints,
                args=(self.node.points[p_name], self.node.get_effective_speed()),
                daemon=True,
            ).start()
        elif self.current_screen == 'JOB':
            jname = self.node.current_job_name
            lines = self.node.jobs[jname]
            if not lines:
                return
            line = lines[self.cursor_idx]
            self._move_cursor(+1)
            threading.Thread(
                target=self.node.execute_inform_line, args=(line,), daemon=True
            ).start()

    def _on_press_bwd(self):
        if self.node.is_running_job or self.current_screen != 'JOB':
            return
        self._move_cursor(-1)
        jname = self.node.current_job_name
        line = self.node.jobs[jname][self.cursor_idx]
        self.node.stop_requested = False
        threading.Thread(
            target=self.node.execute_inform_line, args=(line,), daemon=True
        ).start()

    def _start_test_run(self):
        """Bước 3 (Trang 14): Nhấn giữ [TEST START] trong chế độ TEACH để chạy thử chương trình."""
        if self.node.mode_switch != 'TEACH':
            self.set_human_interface_msg('! Chạy thử [TEST START] chỉ dùng trong chế độ TEACH (Trang 14).')
            return
        self._run_current_job(is_test=True)

    def _stop_test_run(self):
        """Buông nút [TEST START] -> Dừng ngay lập tức (Trang 14)."""
        if self.test_start_held:
            self.test_start_held = False
            self._on_press_hold()
            self.set_human_interface_msg('Đã nhả nút [TEST START] -> Tạm dừng chạy thử (Trang 14).')

    def _on_press_start(self):
        """Bước 4 (Trang 14, 15): Chạy tự động chương trình khi nhấn [START]."""
        if not self.node.is_servo_powered:
            self.set_human_interface_msg('! Hãy bật [SERVO ON READY] trước khi nhấn [START]!')
            return
        self._run_current_job(is_test=False)

    def _run_current_job(self, is_test: bool = False):
        if self.node.is_running_job:
            return
        self.test_start_held = is_test
        self.node.stop_requested = False
        self.node.is_running_job = True
        self.current_screen = 'JOB'
        self._refresh_screen()

        jname = self.node.current_job_name
        lines = list(self.node.jobs[jname])

        def _worker():
            self.icon_state.config(text='[RUN]', bg='#2b8a3e')
            mode_tag = 'TEST START' if is_test else 'PLAY'
            self.set_human_interface_msg(f'▶ [{mode_tag}] Đang thực thi JOB:{jname}...')
            for idx, line in enumerate(lines):
                if self.node.stop_requested:
                    break
                self.cursor_idx = idx
                self.root.after(0, self._highlight_active_job_line, idx)
                self.node.execute_inform_line(line)
            self.node.is_running_job = False
            self.test_start_held = False
            if not self.node.stop_requested:
                self.icon_state.config(text='[STOP]', bg='#e67700')
                self.set_human_interface_msg(f' Đã hoàn thành JOB:{jname}!')

        threading.Thread(target=_worker, daemon=True).start()

    def _highlight_active_job_line(self, idx: int):
        if self.current_screen == 'JOB' and idx < self.job_listbox.size():
            self.job_listbox.selection_clear(0, tk.END)
            self.job_listbox.selection_set(idx)
            self.job_listbox.see(idx)

    def _on_press_hold(self):
        self.node.stop_requested = True
        self.node.is_running_job = False
        self.node.stop_continuous_jog()
        self.icon_state.config(text='[HOLD]', bg='#e67700')
        self.set_human_interface_msg('⏸ [HOLD] Đã tạm dừng chuyển động của robot.')

    def _toggle_shift(self):
        self.node.shift_held = not self.node.shift_held
        self.btn_shift.config(bg='#fcc419' if self.node.shift_held else '#495057',
                              fg='black' if self.node.shift_held else 'white')

    def _toggle_interlock(self):
        self.node.interlock_held = not self.node.interlock_held
        self.btn_interlock.config(bg='#fcc419' if self.node.interlock_held else '#495057',
                                  fg='black' if self.node.interlock_held else 'white')

    def _toggle_input_switch(self, ch: int):
        """Gạt công tắc I4..I7 (IN#5..IN#8) trên Control Panel (Hình 14 - Trang 17)."""
        self.node.din[ch] = not self.node.din[ch]
        sw_names = {5: 'Công tắc I4 (IN#5)', 6: 'Công tắc I5 (IN#6)', 7: 'Công tắc I6 (IN#7)', 8: 'Công tắc I7 (IN#8)'}
        st_on = self.node.din[ch]
        self.sw_btns[ch].config(
            text=f'{sw_names[ch]}: {"ON" if st_on else "OFF"}',
            bg='#2b8a3e' if st_on else '#495057',
        )
        # Neu bat I4, I5, I6 -> Tu dong thuc hien Yeu cau 2.2 (Trang 17)
        if st_on and ch in (5, 6, 7) and not self.node.is_running_job:
            pick_map = {5: ('P001', 'P002', 'A1'), 6: ('P003', 'P004', 'A2'), 7: ('P005', 'P006', 'A3')}
            p_above, p_touch, slot_name = pick_map[ch]
            place_to_c = self.node.din[8]  # I7 ON -> Thả vào C, I7 OFF -> Thả vào B
            target_tray = 'C' if place_to_c else 'B'
            self.set_human_interface_msg(
                f'[Yêu cầu 2.2 - Trang 17] Đã bật {sw_names[ch]}: Gắp vật ở {slot_name} -> Thả vào {target_tray}!'
            )
            threading.Thread(
                target=self._run_lab_req_2_2,
                args=(p_above, p_touch, place_to_c),
                daemon=True,
            ).start()

    def _run_lab_req_2_2(self, p_above: str, p_touch: str, place_to_c: bool):
        """Thực thi tự động Yêu cầu 2.2 (Trang 17): Gắp từ A1/A2/A3 thả vào B (hoặc C theo I7) và phát xung 1s ra Q4/Q5."""
        self.node.is_running_job = True
        self.node.stop_requested = False
        seq = [
            f'MOVJ {p_above} VJ=50.00',
            f'MOVL {p_touch} V=25.00',
            'DOUT OT#(1) ON',
            'TIMER T=0.40',
            f'MOVL {p_above} V=30.00',
        ]
        if place_to_c:
            seq += [
                'MOVJ P011 VJ=50.00',
                'MOVL P012 V=25.00',
                'DOUT OT#(1) OFF',
                'TIMER T=0.40',
                'MOVL P011 V=40.00',
                'PULSE OT#(6) T=1.00',   # Xuất xung 1s ra đèn Q5 (OUT#6)
                'MOVJ P000 VJ=50.00',
            ]
        else:
            seq += [
                'MOVJ P007 VJ=50.00',
                'MOVL P008 V=25.00',
                'DOUT OT#(1) OFF',
                'TIMER T=0.40',
                'MOVL P007 V=40.00',
                'PULSE OT#(5) T=1.00',   # Xuất xung 1s ra đèn Q4 (OUT#5)
                'MOVJ P000 VJ=50.00',
            ]
        for cmd in seq:
            if self.node.stop_requested:
                break
            self.node.execute_inform_line(cmd)
        self.node.is_running_job = False

    def _toggle_user_object_lock(self):
        self.node.user_object_lock = not self.node.user_object_lock
        if self.node.user_object_lock:
            self.node.coord_mode = 'USER'
            self.node.user_target_offset = [0.0, 0.0, 0.045]
            self.node.jog_target_joints = list(self.node.current_joints)
            self.btn_user_lock.config(text='● BÁM HỆ USER ĐỐI TƯỢNG (0ms TRỄ): ĐANG BÁM', bg='#00c853', fg='black')
            self.set_human_interface_msg(
                '[USER OBJECT LOCK 50Hz]: Đang bám tâm đối tượng (0,0,+45mm) không trễ! Nhấn [Z-] để hạ xuống gắp vật đang chạy!'
            )
        else:
            self.btn_user_lock.config(text='BÁM HỆ USER ĐỐI TƯỢNG (0ms TRỄ): OFF', bg='#e67700', fg='white')
            self.set_human_interface_msg('Đã tắt chế độ khóa bám đối tượng (USER OBJECT LOCK = OFF).')
        self._refresh_screen()

    def _on_top_menu_click(self, menu_name: str):
        if menu_name == 'JOB':
            self._switch_screen('JOB')
        elif menu_name == 'DISPLAY':
            self._switch_screen('VARIABLE')
        elif menu_name == 'UTILITY':
            self._switch_screen('ROBOT')

    def set_human_interface_msg(self, text: str):
        self.root.after(0, lambda: self.human_msg_lbl.config(text=text))

    def _periodic_update(self):
        # Cập nhật bảng tọa độ thời gian thực bên phải LCD
        degs = [math.degrees(r) for r in self.node.current_joints]
        x_mm, y_mm, z_mm, rx, ry, rz = self.node.get_current_pose_in_coord()
        self.live_pos_lbl.config(
            text=(
                f'=== POS {self.node.active_robot} [{self.node.coord_mode}] ===\n'
                f' S: {degs[0]:6.1f}°   R: {degs[3]:6.1f}°\n'
                f' L: {degs[1]:6.1f}°   B: {degs[4]:6.1f}°\n'
                f' U: {degs[2]:6.1f}°   T: {degs[5]:6.1f}°\n'
                f'----------------------------\n'
                f' X : {x_mm:6.1f} mm  Rx: {rx:5.1f}°\n'
                f' Y : {y_mm:6.1f} mm  Ry: {ry:5.1f}°\n'
                f' Z : {z_mm:6.1f} mm  Rz: {rz:5.1f}°'
            )
        )

        # Cập nhật đèn cảm biến băng tải IN#1
        if self.node.din[1]:
            self.in1_panel_lbl.config(text='● IN#1 (Cảm biến băng tải): ON (CÓ PHÔI)', fg='#51cf66')
        else:
            self.in1_panel_lbl.config(text='○ IN#1 (Cảm biến băng tải): OFF', fg='#ff6b6b')

        # Cập nhật các đèn Output trên Control Panel (Hình 14)
        lamp_titles = {
            1: 'OUT#1 (Giác hút / Magnet)',
            5: 'Đèn Q4 (OUT#5 - Báo thả B)',
            6: 'Đèn Q5 (OUT#6 - Báo thả C)',
            7: 'Đèn Q6 (OUT#7)',
            8: 'Đèn Q7 (OUT#8)',
        }
        for ch, lbl in self.out_lamps.items():
            if self.node.dout[ch]:
                lbl.config(text=f'● {lamp_titles[ch]}: ON', bg='#1864ab', fg='#ffd43b')
            else:
                lbl.config(text=f'○ {lamp_titles[ch]}: OFF', bg='#212529', fg='#adb5bd')

        self.root.after(80, self._periodic_update)

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    robot_id = 'R1'
    for arg in sys.argv[1:]:
        if arg.upper() in ('R1', 'R2'):
            robot_id = arg.upper()
            break
    node = InformTeachPendantNode(robot_id=robot_id)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        gui = YaskawaAuthenticPendantGUI(node)
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
