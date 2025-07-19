import board
import busio
import digitalio
import adafruit_vl53l0x
import time
from collections import deque
from adafruit_mcp230xx.mcp23017 import MCP23017
from digitalio import Direction

OFFSET_MM = 0
BUFFER_SIZE = 5
CHANGE_THRESHOLD = 5
ZERO_THRESHOLD = 70

# ==== MCP23017 บน I2C bus0 ====
i2c_mcp = busio.I2C(1, 0)
mcp = MCP23017(i2c_mcp)
pin0 = mcp.get_pin(15)
pin0.direction = Direction.OUTPUT

# ==== VL53L0X บนบัสหลัก ====
i2c_vl53 = busio.I2C(board.SCL, board.SDA)

# ==== XSHUT GPIO สำหรับ VL53L0X (สูงสุด 4 ตัว) ====
xshut_pins = [board.D17, board.D27, board.D22, board.D5]
xshuts = []
vl53_sensors = []
address_base = 0x30

# ปิดทุก sensor ก่อน
for pin in xshut_pins:
    x = digitalio.DigitalInOut(pin)
    x.direction = digitalio.Direction.OUTPUT
    x.value = False
    xshuts.append(x)
time.sleep(0.1)

# เปิดทีละตัว เปลี่ยน address แล้วเพิ่มเข้า list
for i, x in enumerate(xshuts):
    x.value = True
    time.sleep(0.1)
    try:
        sensor = adafruit_vl53l0x.VL53L0X(i2c_vl53)
        new_addr = address_base + i
        sensor.set_address(new_addr)
        sensor.measurement_timing_budget = 33000
        vl53_sensors.append(sensor)
        print(f"✅ Sensor {i+1} ready at address 0x{new_addr:X}")
    except Exception as e:
        print(f"⚠️  Sensor {i+1} not detected. Skipping. ({e})")
        x.value = False  # ปิดตัวที่ไม่ได้ใช้งาน

# ไม่มี sensor เลย? ออก
if not vl53_sensors:
    print("🚫 No VL53L0X sensors detected. Exiting.")
    exit()

# เตรียม buffer
buffers = [deque(maxlen=BUFFER_SIZE) for _ in vl53_sensors]
last_values = [None] * len(vl53_sensors)

print("📡 Start reading from sensors...")

while True:
    display_ranges = []
    for i, sensor in enumerate(vl53_sensors):
        try:
            raw_range = sensor.range + OFFSET_MM
            buffers[i].append(raw_range)
            avg = sum(buffers[i]) / len(buffers[i])

            if last_values[i] is None or abs(avg - last_values[i]) >= CHANGE_THRESHOLD:
                stable = avg
                last_values[i] = avg
            else:
                stable = last_values[i]

            display = 0 if stable < ZERO_THRESHOLD else stable
        except Exception as e:
            display = -1
            print(f"❌ Sensor {i+1} read failed: {e}")

        display_ranges.append(display)

    # แสดงผลทั้งหมด
    output_line = " | ".join(
        [f"S{i+1}: {r:.1f} mm" if r >= 0 else f"S{i+1}: ERROR" for i, r in enumerate(display_ranges)]
    )
    print(output_line)

    # Sensor ตัวแรกควบคุม MCP23017 pin0
    if display_ranges[0] >= 0 and display_ranges[0] <= 80:
        pin0.value = True
    else:
        pin0.value = False

    time.sleep(0.05)
