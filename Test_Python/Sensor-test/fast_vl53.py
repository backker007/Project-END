import board
import busio
import adafruit_vl53l0x
import time
from collections import deque

OFFSET_MM = 0

# Moving average buffer size
BUFFER_SIZE = 10
CHANGE_THRESHOLD = 3  # ยอมให้ต่าง 3 mm ถึงจะอัปเดต

i2c = busio.I2C(board.SCL, board.SDA)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

# เพิ่ม budget เพื่อลด noise
vl53.measurement_timing_budget = 50000
vl53.signal_rate_limit = 0.05

print("Start ranging... Press Ctrl+C to stop.")

# buffer เก็บค่า moving average
buffer = deque(maxlen=BUFFER_SIZE)
last_stable = None

while True:
    # อ่านค่ามา 1 sample
    current_range = vl53.range + OFFSET_MM
    buffer.append(current_range)

    # คำนวณค่าเฉลี่ย
    avg_range = sum(buffer) / len(buffer)

    # ถ้าครั้งแรก ให้ใช้เลย
    if last_stable is None:
        stable_range = avg_range
    else:
        # ถ้าต่างจากค่าเก่าเกิน threshold จึงอัปเดต
        if abs(avg_range - last_stable) >= CHANGE_THRESHOLD:
            stable_range = avg_range
        else:
            stable_range = last_stable

    print("Range: {:.1f} mm".format(stable_range))
    last_stable = stable_range
    time.sleep(0.05)
