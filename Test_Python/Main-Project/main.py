import board
import digitalio
import adafruit_vl53l0x
import time
import os
import sys
import select
import tty
import termios
from collections import deque
from adafruit_mcp230xx.mcp23017 import MCP23017
from digitalio import Direction
from adafruit_pca9685 import PCA9685
import busio

# ==== CONFIG ====
XSHUT_PINS = [board.D17, board.D27, board.D22, board.D5]
ADDRESS_BASE = 0x30
OFFSET_MM = 0
BUFFER_SIZE = 5
CHANGE_THRESHOLD = 5
ZERO_THRESHOLD = 70
CHECK_INTERVAL = 120  # 2 minutes
SERVO_CHANNELS = [0, 1, 2, 3]  # ช่อง 1-4 ใช้ช่อง 0-3 บน PCA9685

# ==== GLOBAL ====
vl53_sensors = []
xshuts = []
buffers = []
last_values = []
mcp = None
relay_pins = []
mcp_pins = []  # 0-15 เรียงตามลำดับ A0–A7, B0–B7
selected_sensor_index = None
reading_active = False

# ใช้ I2C สำหรับทั้ง MCP23017 และ PCA9685 บน bus 1
shared_i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(shared_i2c)
pca.frequency = 50

# ==== MAIN ====
def main():
    global reading_active
    init_mcp()
    init_xshuts()
    reset_vl53_addresses()
    init_sensors()

    last_check = time.time()

    while True:
        if time.time() - last_check >= CHECK_INTERVAL:
            check_slots_and_update_leds()
            last_check = time.time()

        wait_for_user_input()

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while reading_active:
                handle_servo_sequence(selected_sensor_index)
                break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# ==== SERVO SEQUENCE ====
def handle_servo_sequence(index):
    print(f"🔄 สั่งให้เปิดมอเตอร์ช่อง {index + 1} (angle=120)")
    move_servo(index, 1)
    time.sleep(2.0)

    initial = read_sensor(index)
    timeout = time.time() + 10

    while time.time() < timeout:
        result = read_sensor(index)
        print(f"ตรวจสอบเซ็นเซอร์ S{index + 1}: {result:.1f} mm")
        if result > 0 and initial > 0 and result < initial - CHANGE_THRESHOLD:
            print("📦 ตรวจพบว่าระยะลดลง แสดงว่าใส่วัตถุแล้ว")
            break
        time.sleep(0.2)

    print(f"🔒 สั่งให้ปิดมอเตอร์ช่อง {index + 1} (angle=0)")
    move_servo(index, 0)
    time.sleep(2.0)

    relay_pins[index].value = False
    global reading_active
    reading_active = False

    if not is_door_closed(index):
        print(f"❌ ช่อง {index + 1}: Magnetic lock ปิดไม่สนิท!")
    else:
        print(f"✅ ช่อง {index + 1}: Magnetic lock ปิดเรียบร้อย")

# ==== SERVO CONTROL ====
def move_servo(channel: int, state: int):
    angle = 180 if state == 1 else 0
    print(f"  → กำหนดมุม Servo = {angle}°")

    pulse_us = 500 + (angle / 180) * 2000  # ลองขยับเป็น 600–2400 ถ้ายังไม่หมุนสุด
    duty_cycle = int(pulse_us * 65535 / 20000)

    print(f"    → pulse_us = {pulse_us:.2f} us, duty_cycle = {duty_cycle}")
    pca.channels[channel].duty_cycle = duty_cycle
    time.sleep(2.0)  # เพิ่มเวลาให้หมุนจริง
    pca.channels[channel].duty_cycle = 0




# ==== INIT ====
def init_sensors():
    global vl53_sensors, buffers, last_values
    vl53_sensors.clear()
    buffers.clear()
    last_values.clear()
    i2c = shared_i2c
    for i, x in enumerate(xshuts):
        x.value = True
        time.sleep(0.2)
        try:
            sensor = adafruit_vl53l0x.VL53L0X(i2c)
            sensor.set_address(ADDRESS_BASE + i)
            sensor.measurement_timing_budget = 33000
            vl53_sensors.append(sensor)
            buffers.append(deque(maxlen=BUFFER_SIZE))
            last_values.append(None)
        except:
            x.value = False
    if not vl53_sensors:
        print("⚠️ No sensors found. Trying I2C reset...")
        reset_i2c_bus()
        print("🔁 Retrying sensor init...")
        init_sensors()

def reset_i2c_bus():
    os.system("sudo i2cdetect -y 1 > /dev/null 2>&1")
    time.sleep(0.5)

def reset_vl53_addresses():
    i2c = shared_i2c
    for i, x in enumerate(xshuts):
        x.value = True
        time.sleep(0.1)
        try:
            sensor = adafruit_vl53l0x.VL53L0X(i2c, address=ADDRESS_BASE + i)
            sensor.set_address(0x29)
        except:
            pass
        x.value = False
    time.sleep(0.2)

def init_xshuts():
    global xshuts
    xshuts = []
    for pin in XSHUT_PINS:
        x = digitalio.DigitalInOut(pin)
        x.direction = digitalio.Direction.OUTPUT
        x.value = False
        xshuts.append(x)
    time.sleep(0.2)

def init_mcp():
    global mcp, relay_pins, mcp_pins
    mcp = MCP23017(shared_i2c)

    relay_pin_nums = [12, 13, 14, 15]  # B4-B7
    relay_pins = []
    mcp_pins = []

    for pin_num in range(16):
        pin = mcp.get_pin(pin_num)
        if pin_num in relay_pin_nums:
            pin.direction = Direction.OUTPUT
            pin.value = False
            relay_pins.append(pin)
        elif pin_num <= 7:
            pin.direction = Direction.OUTPUT
        else:
            pin.direction = Direction.INPUT
            pin.pull_up = True
        mcp_pins.append(pin)

# ==== SENSOR ====
def read_sensor(sensor_index):
    try:
        sensor = vl53_sensors[sensor_index]
        raw = sensor.range + OFFSET_MM
        buffers[sensor_index].append(raw)
        avg = sum(buffers[sensor_index]) / len(buffers[sensor_index])
        if last_values[sensor_index] is None or abs(avg - last_values[sensor_index]) >= CHANGE_THRESHOLD:
            last_values[sensor_index] = avg
        stable = last_values[sensor_index]
        return 0 if stable < ZERO_THRESHOLD else stable
    except:
        return -1

# ==== SENSOR SWITCH CHECK ====
def is_door_closed(channel: int) -> bool:
    try:
        pin_number = 8 + channel
        pin = mcp_pins[pin_number]
        low_count = 0
        for _ in range(15):
            if pin.value == 0:
                low_count += 1
            time.sleep(0.066)
        return low_count >= 12
    except:
        return False

# ==== SERVO CONTROL ====
def set_servo_angle(channel: int, angle: int):
    pulse_us = 500 + (angle / 180) * 2000
    duty_cycle = int(pulse_us * 65535 / 20000)
    pca.channels[channel].duty_cycle = duty_cycle

def reset_servo(channel: int):
    pca.channels[channel].duty_cycle = 0

# ==== LED STATUS CHECK ====
def check_slots_and_update_leds():
    print("🔧 Checking slots and updating LEDs...")
    for i in range(len(vl53_sensors)):
        result = read_sensor(i)
        green_led = mcp_pins[i]
        red_led = mcp_pins[8 + i]

        if result == -1:
            print(f"S{i+1}: Sensor error")
            green_led.value = False
            red_led.value = True
        elif result == 0:
            print(f"S{i+1}: Slot is EMPTY")
            green_led.value = True
            red_led.value = False
        else:
            print(f"S{i+1}: Slot is FULL ({result:.1f} mm)")
            green_led.value = False
            red_led.value = True

# ==== USER INPUT ====
def wait_for_user_input():
    global selected_sensor_index, reading_active
    print("📡 Ready. เลือก Relay (1–4) เพื่อเริ่มอ่าน หรือ 0 เพื่อปิดทั้งหมด:")
    val = input().strip()
    if val in ["0", "1", "2", "3", "4"]:
        n = int(val)
        selected_sensor_index = n - 1 if n > 0 else None
        reading_active = n > 0
        for i, pin in enumerate(relay_pins):
            pin.value = (i == selected_sensor_index) if reading_active else False

# ==== ENTRY ====
if __name__ == "__main__":
    main()
