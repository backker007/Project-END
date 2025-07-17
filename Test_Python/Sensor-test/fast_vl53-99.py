import board
import busio
import adafruit_vl53l0x
import time
from collections import deque

OFFSET_MM = 0

BUFFER_SIZE = 5          # ขนาด buffer ค่าเฉลี่ย
CHANGE_THRESHOLD = 5     # ถ้าต่างเกิน 5 mm จึงอัปเดต

i2c = busio.I2C(board.SCL, board.SDA)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

# Timing budget พอประมาณ
vl53.measurement_timing_budget = 33000
vl53.signal_rate_limit = 0.05

print("Start ranging... Press Ctrl+C to stop.")

buffer = deque(maxlen=BUFFER_SIZE)
last_stable = None

while True:
    # อ่านค่าเดียว
    current_range = vl53.range + OFFSET_MM
    buffer.append(current_range)

    # คำนวณค่าเฉลี่ยใน buffer
    avg_range = sum(buffer) / len(buffer)

    if last_stable is None:
        stable_range = avg_range
    else:
        # ถ้าต่างเกิน threshold ให้เปลี่ยนค่า
        if abs(avg_range - last_stable) >= CHANGE_THRESHOLD:
            stable_range = avg_range
        else:
            stable_range = last_stable

    print("Range: {:.1f} mm".format(stable_range))
    last_stable = stable_range
    time.sleep(0.05)
