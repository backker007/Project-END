import board
import busio
import adafruit_vl53l0x
import time
import statistics

OFFSET_MM = 0

i2c = busio.I2C(board.SCL, board.SDA)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

vl53.measurement_timing_budget = 20000
vl53.signal_rate_limit = 0.05

print("Start ranging... Press Ctrl+C to stop.")

last_range = None
CHANGE_THRESHOLD = 10

while True:
    samples = []
    for _ in range(5):
        samples.append(vl53.range)
        time.sleep(0.02)
    median = statistics.median(samples)
    corrected_range = max(0, median + OFFSET_MM)

    if last_range is None:
        stable_range = corrected_range
    else:
        if abs(corrected_range - last_range) >= CHANGE_THRESHOLD:
            stable_range = corrected_range
        else:
            stable_range = last_range

    # ถ้าน้อยกว่า 70 mm ให้ถือว่าเป็น 0
    if stable_range < 70:
        display_range = 0
    else:
        display_range = stable_range

    print("Range: {} mm".format(display_range))
    last_range = stable_range
    time.sleep(0.1)
