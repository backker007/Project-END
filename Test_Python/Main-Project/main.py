import board
import busio
import adafruit_vl53l0x
import time
from collections import deque
from adafruit_mcp230xx.mcp23017 import MCP23017
from digitalio import Direction


OFFSET_MM = 0

BUFFER_SIZE = 5
CHANGE_THRESHOLD = 5
ZERO_THRESHOLD = 70

# I2C สำหรับ VL53L0X
i2c_vl53 = busio.I2C(board.SCL, board.SDA)

# I2C bus0 สำหรับ MCP23017
i2c_mcp = busio.I2C(1, 0)
mcp = MCP23017(i2c_mcp)

# ตัวอย่าง: set pin 0 เป็น output
pin0 = mcp.get_pin(15)
pin0.direction = Direction.OUTPUT


# สร้าง VL53L0X instance
vl53 = adafruit_vl53l0x.VL53L0X(i2c_vl53)

vl53.measurement_timing_budget = 33000
vl53.signal_rate_limit = 0.05

print("Start ranging... Press Ctrl+C to stop.")

buffer = deque(maxlen=BUFFER_SIZE)
last_stable = None

while True:
    current_range = vl53.range + OFFSET_MM
    buffer.append(current_range)

    avg_range = sum(buffer) / len(buffer)

    if last_stable is None:
        stable_range = avg_range
    else:
        if abs(avg_range - last_stable) >= CHANGE_THRESHOLD:
            stable_range = avg_range
        else:
            stable_range = last_stable

    if stable_range < ZERO_THRESHOLD:
        display_range = 0
    else:
        display_range = stable_range

    print("Range: {:.1f} mm".format(display_range))

    # ตัวอย่าง: ถ้าระยะ <100mm เปิด pin0, ถ้า >=100 ปิด pin0
    if display_range <= 100:
        pin0.value = True
    else:
        pin0.value = False

    last_stable = stable_range
    time.sleep(0.05)
