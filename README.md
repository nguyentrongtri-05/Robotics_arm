# Yaskawa MotoMINI ROS 2 Humble Simulation (MoveIt 2 + Teach Pendant + Virtual Conveyor)

Dự án mô phỏng hệ thống cánh tay robot công nghiệp **Yaskawa MotoMINI (6-DOF)** trên nền tảng **ROS 2 Humble** & **MoveIt 2**, bám sát theo tài liệu *Bài Thí Nghiệm 1: Vận Hành Robot Công Nghiệp (Bộ môn Điều khiển Tự động - ĐHBK TP.HCM)*.

## 1. Cấu trúc thư mục

```text
kythuatRobot/
└── src/
    ├── motomini_description/          # Gói mô hình 3D (URDF/Xacro & STL Meshes)
    │   ├── meshes/motomini/
    │   └── urdf/
    │       ├── motomini_workcell.xacro
    │       └── moto.urdf
    └── motomini_bringup/              # Gói khởi chạy tích hợp (Bringup, MoveIt 2, Nodes)
        ├── config/                    # Cấu hình SRDF, Kinematics, Controllers, RViz2
        ├── launch/
        │   └── bringup.launch.py      # File launch duy nhất khởi chạy toàn bộ hệ thống
        └── scripts/
            ├── virtual_conveyor.py    # Node băng tải ảo, cảm biến IN#1 & vật mẫu đa hình dạng
            └── inform_teach_pendant.py# Giao diện Bộ lập trình Yaskawa YRC1000micro (INFORM III)
```

## 2. Tính năng nổi bật

- **4 Hệ tọa độ điều khiển bằng tay (`[COORD]`)**:
  - `JOINT`: Điều khiển độc lập 6 khớp xoay (`S, L, U, R, B, T`).
  - `CARTESIAN`: Nội suy đường thẳng Đề-các theo hệ tọa độ đế robot (`X, Y, Z, Rx, Ry, Rz`).
  - `TOOL`: Nội suy chuyển động dọc theo hướng của đầu công tác / giác hút (`tcp_link`).
  - `USER`: Nội suy theo hệ tọa độ người dùng đặt trên mặt phẳng băng tải (`user_frame_1`).
- **Bộ lập trình Yaskawa YRC1000micro (`JZRCR-YPP21-1`)**:
  - Đầy đủ các nút chức năng: `Mode Switch (REMOTE / PLAY / TEACH)`, `[START]`, `[HOLD]`, `[EMERGENCY STOP]`, `[SERVO ON READY]`, `Enable Switch`, `[MANUAL SPEED: INCH / LOW / MED / HIGH]`, `[HIGH SPEED]`, 12 phím `[Axis Key]` nhấn giữ liên tục.
  - Hỗ trợ lập trình và thực thi ngôn ngữ **INFORM**: `MOVJ`, `MOVL`, `DOUT`, `WAIT`, `PULSE`, `TIMER`, `CALL JOB`, cùng các thao tác `[INFORM LIST]`, `[INSERT]`, `[MODIFY]`, `[DELETE]`, `[FWD]`, `[BWD]`, `[TEST START]`.
  - Tích hợp bảng **Yaskawa Control Panel I/O** (`I4..I7` nối với `IN#5..IN#8`, đèn `Q4..Q7` nối với `OUT#5..OUT#8`).
- **Luật chống xuyên thấu & va chạm vật lý (Anti-Penetration Rules)**:
  - Ngăn đầu hút đâm xuyên qua vật mẫu, mặt băng tải hoặc mặt bàn.
  - Vật mẫu tự động khóa phẳng (`frame_locked`) vào mặt đáy giác hút khi được hút.

## 3. Hướng dẫn cài đặt và chạy mô phỏng

```bash
# 1. Biên dịch workspace
cd ~/kythuatRobot
source /opt/ros/humble/setup.bash
colcon build

# 2. Khởi chạy toàn bộ hệ thống bằng 1 lệnh duy nhất
source install/setup.bash
ros2 launch motomini_bringup bringup.launch.py
```
