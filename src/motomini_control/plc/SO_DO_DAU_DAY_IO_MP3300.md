# SƠ ĐỒ ĐẤU DÂY & BẢNG ĐỊA CHỈ I/O MACHINE CONTROLLER YASKAWA MP3300
*(Theo Bài Thí Nghiệm 2, 3, 4 & Phụ lục MPE720 — Bộ môn Điều khiển Tự Động - ĐHBK TPHCM, mở rộng cho 2 Cánh Tay MotoMINI & 2 Băng Tải Vuông Góc)*

---

## 1. Sơ Đồ Đấu Dây Tổng Quan Giữa MP3300, 2 Băng Tải, Đĩa Xoay & 2 Robot (Hình 1 & Mục 4.1)

```text
+---------------------------------------------------------------------------------------------------+
|                                   1. CONTROL PANEL & I/O BOX                                      |
|  [Nút START]       [Nút STOP]       [Nút Servo_ON]     [Nút Robot_ON]    [Công tắc I0..I7]        |
+-------+-----------------+------------------+------------------+------------------+----------------+
        |                 |                  |                  |                  |
        | DI_0 (IB0C400)  | DI_1 (IB0C401)   | DI_2 (IB0C402)   | DI_3 (IB0C403)   | DI_24..31
        v                 v                  v                  v                  v
+===================================================================================================+
|                    2. BỘ ĐIỀU KHIỂN TRUNG TÂM YASKAWA MP3300 (CPU-301 16-Axes)                    |
|  - Module 02 [218IFD] : Ethernet 192.168.1.1 (Port 10001 <-> Robot UDP Port 10040)                |
|  - Module 03 [SVC]    : MECHATROLINK-III điều khiển Servo Đĩa Xoay Trục A1 (%OL8010, %OW803C)      |
|  - Module 01 [LIO-04] : 32 Ngõ vào số (IB0C400..IB0C41F) & 32 Ngõ ra số (OB0C400..OB0C41F)        |
+=======+=================+==================+==================+==================+================+
        |                 ^                  |                  ^                  |
        | DO_0            | DI_11            | DO_1             | DI_12            | DO_2 & DO_3
        | (OB0C400)       | (IB0C40B)        | (OB0C401)        | (IB0C40C)        | (OB0C402 / OB0C403)
        v                 |                  v                  |                  v
+-------------------------+----+   +----------------------------+----+   +--------------------------+
| 3. BĂNG TẢI 1 & ĐĨA XOAY A1  |   | 4. TỦ ROBOT 1 (YRC1000micro #1) |   | 5. TỦ ROBOT 2 & ĐAI 2    |
| - Biến tần J1000 <- DO_0     |   | - IP: 192.168.1.11 : 10040      |   | - IP: 192.168.1.12:10040 |
| - Sensor Đai 1   -> DI_11    |   | - Chân IN#1  <-- MC DO_1        |   | - R2 IN#1 <-- MC DO_2    |
| - Sensor HOME_A  -> DI_10    |   | - Chân OT#3  --> MC DI_12 (Xung)|   | - Đai 2   <-- MC DO_3    |
| - Encoder 1      -> MW00100  |   | - Chân OT#1  --> Giác hút R1    |   | - Sensor 2--> MC DI_13   |
+------------------------------+   +---------------------------------+   +--------------------------+
```

---

## 2. Bảng 1: Quy Định Địa Chỉ Đấu Dây I/O Trên Module LIO-04 (Dùng Để Viết Ladder MPE720)

### A. Ngõ Vào Số — Digital Inputs (`MC DI` $\rightarrow$ Địa chỉ Bit trong MPE720)

| Chân Vật Lý (`MC DI`) | Địa Chỉ MPE720 (`Bit`) | Thiết Bị Đấu Nối (`Target Device`) | Tín Hiệu & Chức Năng Trong Chương Trình PLC |
| :--- | :--- | :--- | :--- |
| **`DI_0`** | **`IB0C400`** | Control Panel | **`START button`** — Nhấn để khởi động hệ thống băng tải |
| **`DI_1`** | **`IB0C401`** | Control Panel | **`STOP button`** — Nhấn để dừng hệ thống băng tải |
| **`DI_2`** | **`IB0C402`** | Control Panel | **`Servo_ON button`** — Cấp nguồn Servo cho Đĩa xoay Trục A1 |
| **`DI_3`** | **`IB0C403`** | Control Panel | **`Robot_ON button`** — Bật/Tắt Servo Robot qua Ethernet / Reset Alarm |
| **`DI_10`** | **`IB0C40A`** | A axis (Đĩa xoay) | **`Sensor HOME_A`** — Cảm biến gốc 0° của Đĩa xoay (dùng cho lệnh `ZRN[A1]0` Method 19) |
| **`DI_11`** | **`IB0C40B`** | Conveyor 1 | **`Sensor Conveyor 1`** — Phát hiện có phôi tại vị trí gắp S2 cuối Băng tải 1 |
| **`DI_12`** | **`IB0C40C`** | Robot 1 (`OT#3`) | **`Robot 1 OT#3`** — Nhận xung cạnh lên `100ms` từ Robot 1 báo đã thả vật lên Đĩa xoay |
| **`DI_13`** | **`IB0C40D`** | Conveyor 2 | **`Sensor Conveyor 2`** — Phát hiện phôi thành phẩm tới cuối Băng tải 2 vuông góc |
| **`DI_16..23`** | **`IB0C410..17`** | Control Panel | **`I0..I7 switches`** — Các công tắc gạt trên bảng Control Panel |
| **`DI_24..31`** | **`IB0C418..1F`** | I/O Box | **`I0..I7 switches`** — Các công tắc gạt trên hộp I/O Box |

---

### B. Ngõ Ra Số — Digital Outputs (`MC DO` $\rightarrow$ Địa chỉ Coil trong MPE720)

| Chân Vật Lý (`MC DO`) | Địa Chỉ MPE720 (`Coil`) | Thiết Bị Đấu Nối (`Target Device`) | Tín Hiệu & Chức Năng Trong Chương Trình PLC |
| :--- | :--- | :--- | :--- |
| **`DO_0`** | **`OB0C400`** | Inverter J1000 (`S1`) | **`Chạy Băng tải 1`** — Khi `ON`, biến tần kéo Băng tải 1 chạy dọc trục `+Y` |
| **`DO_1`** | **`OB0C401`** | Robot 1 (`IN#1`) | **`Báo Robot 1 gắp phôi`** — Lên `ON` khi phôi chạm cảm biến `DI_11` trên Băng tải 1 |
| **`DO_2`** | **`OB0C402`** | Robot 2 (`IN#1`) / `R1 IN#2` | **`Báo có phôi trên Đĩa xoay`** — Lên `ON` sau khi Đĩa xoay quay `90°` đưa phôi sang hướng `+X` |
| **`DO_3`** | **`OB0C403`** | Inverter 2 (`S1`) | **`Chạy Băng tải 2 vuông góc`** — Khi `ON`, kéo Băng tải 2 chạy dọc trục `+X` |
| **`DO_16..23`** | **`OB0C410..17`** | Control Panel | **`Q0..Q7 lamps`** — Đèn báo trạng thái trên bảng Control Panel |
| **`DO_24..31`** | **`OB0C418..1F`** | I/O Box | **`Q0..Q7 lamps`** — Đèn báo trạng thái trên hộp I/O Box (`DO_24` = Đèn `Q0`) |

---

### C. Thanh Ghi Nội & Tham Số Servo Trục A1 (Dùng Trong Ladder & Motion `MPM001` / `MPM002`)

| Địa Chỉ Thanh Ghi | Tên Biến Gợi Ý | Ý Nghĩa Sử Dụng |
| :--- | :--- | :--- |
| **`DB000000`** | `VAR_Start` | Bit tự giữ (Latch) duy trì trạng thái chạy của hệ thống sau khi nhấn `START` (`IB0C400`) |
| **`MA00100`** | `MOTION_002` | Địa chỉ Struct quản lý hàm `MSEE` gọi chương trình Motion `MPM002` (Trang 6 & 7 Phụ lục) |
| **`MB001000`** | `MOTION_002.Run` | Bit báo chương trình Motion đang chạy (`Status`) |
| **`MB001010`** | `MOTION_002.Start` | Bit kích khởi động chương trình Motion bằng cạnh lên (`Control`) |
| **`%OW803C`** | `Home_Method_A1` | Phương pháp về Home của trục A1 (`19` = Dùng tín hiệu cảm biến `DI_10` - Trang 5 Phụ lục) |
| **`%OL8040`** | `Creep_Speed_A1` | Tốc độ dò cảm biến gốc `DI_10` của trục A1 (Đơn vị `0.1 deg/s`, VD: `500` = `50.0 deg/s`) |
| **`%OL8010`** | `Speed_Ref_A1` | Tốc độ quay trục A1 (Đơn vị `0.1 deg/s`, VD: `900` = `90.0 deg/s`) |
| **`MOV[A1]900;`** | Lệnh Motion | Quay trục `[A1]` thêm một góc `900` $\times 0.1^\circ = +90.0^\circ$ (khi khai báo sau lệnh `INC;`) |

