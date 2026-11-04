# ============================================================================
# BÀI TẬP LẬP TRÌNH TRÊN GUI: TÍNH TỌA ĐỘ VẬT THỂ & GỐC HỆ TỌA ĐỘ USER [OBJECT]
# TỪ TÍN HIỆU CÓ SẴN CỦA ENCODER 1, ENCODER 2 & CẢM BIẾN TIỆM CẬN S1..S4
# ============================================================================
#
# 1. CÁC TÍN HIỆU PHẦN CỨNG ĐẦU VÀO ĐÃ CÓ SẴN (Tự động cập nhật ở 50 Hz):
#   - enc1_pulses   (int)   : Số xung hiện tại của Encoder 1 (gắn trên Băng tải 1 dọc trục +Y)
#   - enc2_pulses   (int)   : Số xung hiện tại của Encoder 2 (gắn trên Băng tải 2 dọc trục +X)
#   - s1_sensor     (bool)  : Cảm biến tiệm cận S1 đầu Băng tải 1 (tại X = 220.0 mm, Y = -240.0 mm)
#   - s2_sensor     (bool)  : Cảm biến tiệm cận S2 cuối Băng tải 1 (tại X = 220.0 mm, Y = 0.0 mm)
#   - s3_sensor     (bool)  : Cảm biến tiệm cận S3 đầu Băng tải 2 (tại X = 400.0 mm, Y = 260.0 mm)
#   - s4_sensor     (bool)  : Cảm biến tiệm cận S4 cuối Băng tải 2 (tại X = 880.0 mm, Y = 260.0 mm)
#   - on_rotary     (bool)  : Cảm biến báo vật đang nằm trên Đĩa xoay Trục A1 (tâm X=220, Y=260, R=45mm)
#   - rotary_deg    (float) : Góc quay hiện tại của Servo Đĩa xoay A1 (độ)
#   - pulses_per_mm (float) : Độ phân giải của Encoder = 20.0 xung / mm
#   - mem           (dict)  : Bộ nhớ lưu biến chốt xung giữa các chu kỳ quét:
#                             mem['latch1'], mem['latch2'], mem['rot_start_deg'], mem['stage']
#
# 2. ĐẦU RA BẠN CẦN TÍNH (Gốc của Hệ tọa độ USER [OBJECT] gửi sang Robot R1 & R2):
#   - x_obj   (mm)  : Tọa độ X của tâm vật thể trong hệ World
#   - y_obj   (mm)  : Tọa độ Y của tâm vật thể trong hệ World
#   - z_obj   (mm)  : Tọa độ Z mặt trên của vật thể (mặt băng tải Z=20mm + chiều cao vật 20mm = 40.0mm)
#   - yaw_obj (deg) : Góc hướng (Yaw) của hệ tọa độ USER gắn trên vật thể
# ============================================================================

# --- BƯỚC 1: BẮT SƯỜN LÊN CẢM BIẾN TIỆM CẬN ĐỂ CHỐT XUNG ENCODER (LATCH) ---
if s1_sensor and mem.get('latch1') is None:
    mem['latch1'] = enc1_pulses
    mem['stage'] = 'CONV1'

if on_rotary and mem.get('stage') != 'ROTARY':
    mem['stage'] = 'ROTARY'
    mem['rot_start_deg'] = rotary_deg

if s3_sensor and mem.get('latch2') is None:
    mem['latch2'] = enc2_pulses
    mem['stage'] = 'CONV2'

# --- BƯỚC 2: CÔNG THỨC TÍNH TỌA ĐỘ VẬT THỂ (x_obj, y_obj, z_obj, yaw_obj) ---
stage = mem.get('stage', 'CONV1')

if stage == 'CONV1' and mem.get('latch1') is not None:
    # Vật đang đi trên Băng tải 1 (dọc trục +Y từ cảm biến S1 tại Y = -240.0 mm):
    delta_pulses_1 = enc1_pulses - mem['latch1']
    quang_duong_y_mm = delta_pulses_1 / pulses_per_mm

    x_obj = 220.0
    y_obj = -240.0 + quang_duong_y_mm
    z_obj = 40.0
    yaw_obj = 0.0

elif stage == 'ROTARY':
    # Vật nằm trên Đĩa xoay Trục A1 (Tâm đĩa X = 220.0 mm, Y = 260.0 mm, bán kính đặt vật R = 45.0 mm)
    # Điểm đặt ban đầu ở mép dưới (-90 độ), sau đó quay theo góc chênh lệch của Đĩa xoay:
    d_deg = rotary_deg - mem.get('rot_start_deg', 0.0)
    goc_rad = math.radians(-90.0 + d_deg)

    x_obj = 220.0 + 45.0 * math.cos(goc_rad)
    y_obj = 260.0 + 45.0 * math.sin(goc_rad)
    z_obj = 40.0
    yaw_obj = d_deg

elif stage == 'CONV2' and mem.get('latch2') is not None:
    # Vật đang đi trên Băng tải 2 vuông góc (dọc trục +X từ cảm biến S3 tại X = 400.0 mm):
    delta_pulses_2 = enc2_pulses - mem['latch2']
    quang_duong_x_mm = delta_pulses_2 / pulses_per_mm

    x_obj = 400.0 + quang_duong_x_mm
    y_obj = 260.0
    z_obj = 40.0
    yaw_obj = -90.0

else:
    # Khi vật mới thả ở đầu Băng tải 1 và chưa chạm cảm biến tiệm cận S1:
    x_obj = 220.0
    y_obj = -360.0
    z_obj = 40.0
    yaw_obj = 0.0
