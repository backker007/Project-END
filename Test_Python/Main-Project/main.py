import board
import digitalio
import adafruit_vl53l0x
import time
import os
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
CHECK_INTERVAL = 120
SERVO_CHANNELS = [0, 1, 2, 3]

# ==== GLOBAL ====
vl53_sensors = []
xshuts = []
buffers = []
last_values = []
mcp = None
relay_pins = []
mcp_pins = []
selected_sensor_index = None
reading_active = False

# I2C & PCA9685
shared_i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(shared_i2c)
pca.frequency = 50

# ==== MAIN LOOP ====
def main():
    global reading_active
    init_mcp()
    init_xshuts()
    reset_vl53_addresses()
    init_sensors()

    last_check = time.time()

    try:
        while True:
            if time.time() - last_check >= CHECK_INTERVAL:
                check_slots_and_update_leds()
                last_check = time.time()

            wait_for_user_input()

            if reading_active:
                handle_servo_sequence(selected_sensor_index)
    except KeyboardInterrupt:
        print("\n🛑 Program terminated by user.")

# ==== SERVO CONTROL ====
def angle_to_duty_cycle(angle):
    pulse_us = 500 + (angle / 180.0) * 2000
    return int((pulse_us / 20000.0) * 65535)

def move_servo_180(channel, angle):
    duty_cycle = angle_to_duty_cycle(angle)
    print(f"  → SG90-180° CH{channel} → {angle}° (duty: {duty_cycle})")
    pca.channels[channel].duty_cycle = duty_cycle
    time.sleep(0.7)
    pca.channels[channel].duty_cycle = 0

# ==== SERVO SEQUENCE ====
def handle_servo_sequence(index):
    print(f"🔄 เปิดมอเตอร์ช่อง {index + 1} (→ 180°)")
    move_servo_180(index, 180)
    relay_pins[index].value = True

    # อ่านค่าเริ่มต้น
    initial = -1
    timeout = time.time() + 5
    while initial <= 0 and time.time() < timeout:
        initial = read_sensor(index)
        time.sleep(0.2)

    if initial <= 0:
        print("❌ อ่านค่าเริ่มต้นไม่สำเร็จ")
        move_servo_180(index, 0)
        relay_pins[index].value = False
        return

    print(f"📏 ค่าความลึกเริ่มต้น: {initial:.1f} mm")

    timeout = time.time() + 10
    while time.time() < timeout:
        current = read_sensor(index)
        print(f"⏳ รอการใส่ของ... S{index + 1}: {current:.1f} mm")
        if current > 0 and current < initial - CHANGE_THRESHOLD:
            print("📦 ตรวจพบการใส่ของครั้งแรก")
            break
        time.sleep(0.2)
    else:
        print("⏱️ หมดเวลาใส่ของ → ปิดมอเตอร์")
        move_servo_180(index, 0)
        relay_pins[index].value = False
        return

    # ตรวจจับการขยับ
    last_motion_time = time.time()
    last_distance = read_sensor(index)
    print("🔁 เริ่มตรวจจับความเคลื่อนไหว...")

    while True:
        current = read_sensor(index)
        print(f"📡 S{index + 1}: {current:.1f} mm")

        if abs(current - last_distance) >= CHANGE_THRESHOLD:
            print("📍 พบการขยับ → รีเซ็ตตัวจับเวลา")
            last_motion_time = time.time()
            last_distance = current

        if time.time() - last_motion_time >= 3:
            print("⏳ ไม่มีการขยับนาน 3 วิ → ปิดมอเตอร์")
            break

        time.sleep(0.2)

    print(f"🔒 ปิดมอเตอร์ช่อง {index + 1} (← 0°)")
    move_servo_180(index, 0)
    relay_pins[index].value = False

    global reading_active
    reading_active = False

    if not is_door_closed(index):
        print(f"❌ ช่อง {index + 1}: Magnetic lock ปิดไม่สนิท!")
    else:
        print(f"✅ ช่อง {index + 1}: Magnetic lock ปิดเรียบร้อย")

# ==== SENSOR INIT ====
def init_sensors():
    global vl53_sensors, buffers, last_values
    vl53_sensors.clear()
    buffers.clear()
    last_values.clear()
    for i, x in enumerate(xshuts):
        x.value = True
        time.sleep(0.3)
        try:
            sensor = adafruit_vl53l0x.VL53L0X(shared_i2c)
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
    for i, x in enumerate(xshuts):
        x.value = True
        time.sleep(0.1)
        try:
            sensor = adafruit_vl53l0x.VL53L0X(shared_i2c, address=ADDRESS_BASE + i)
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
    door_switch_pins = [8, 9, 10, 11]  # B0-B3
    relay_pins.clear()
    mcp_pins.clear()
    for pin_num in range(16):
        pin = mcp.get_pin(pin_num)
        if pin_num in relay_pin_nums:
            pin.direction = Direction.OUTPUT
            pin.value = False
            relay_pins.append(pin)
        elif pin_num in door_switch_pins:
            pin.direction = Direction.INPUT
            pin.pull_up = True
        else:
            pin.direction = Direction.OUTPUT
        mcp_pins.append(pin)

# ==== SENSOR READ ====
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

# ==== DOOR SWITCH ====
def is_door_closed(channel):
    try:
        pin = mcp_pins[8 + channel]  # B0-B3 → 8–11
        return pin.value == 0
    except:
        return False

# ==== LED STATUS ====
def check_slots_and_update_leds():
    print("🔧 Checking slots and updating LEDs...")
    for i in range(len(vl53_sensors)):
        result = read_sensor(i)
        green_led = mcp_pins[i]          # Placeholder: LED green
        red_led = mcp_pins[8 + i]        # Placeholder: LED red (reusing door pins)
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
    while True:
        print("📡 Ready. เลือก Relay (1–4) เพื่อเริ่มอ่าน หรือ 0 เพื่อปิดทั้งหมด:")
        val = input().strip()
        if val in ["0", "1", "2", "3", "4"]:
            n = int(val)
            selected_sensor_index = n - 1 if n > 0 else None
            reading_active = n > 0
            for i, pin in enumerate(relay_pins):
                pin.value = (i == selected_sensor_index) if reading_active else False
            break

# ==== START ====
if __name__ == "__main__":
    main()
