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

# ==== STATE MACHINE ====
STATE_IDLE = "idle"
STATE_WAIT_INSERT = "wait_insert"
STATE_MONITOR_MOVEMENT = "monitor_movement"
STATE_CLOSE_SERVO = "close_servo"
STATE_DONE = "done"

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
user_role = "student"  # or "professor", "admin"
slot_status = [
    {"capacity_mm": 0, "door_closed": True, "available": True} for _ in range(4)
]

# I2C & PCA9685
shared_i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(shared_i2c)
pca.frequency = 50

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

            if reading_active:
                run_state_machine(selected_sensor_index)
                handle_door_open_sequence(selected_sensor_index)
    except KeyboardInterrupt:
        print("\n🚑 Program terminated by user.")

# ==== STATE MACHINE CONTROLLER ====
def run_state_machine(index):
    state = STATE_WAIT_INSERT
    move_servo_180(index, 180)
    print(f"🔄 เปิดมอเตอร์ช่อง {index + 1} (→ 180°)")

    initial = -1
    timeout = time.time() + 5
    while initial <= 0 and time.time() < timeout:
        initial = read_sensor(index)
        time.sleep(0.2)

    if initial <= 0:
        print("❌ อ่านค่าเริ่มต้นไม่สำเร็จ")
        state = STATE_CLOSE_SERVO

    while state != STATE_DONE:
        if state == STATE_WAIT_INSERT:
            timeout = time.time() + 10
            while time.time() < timeout:
                current = read_sensor(index)
                print(f"⏳ รอการใส่ของ... S{index + 1}: {current:.1f} mm")
                if current > 0 and current < initial - CHANGE_THRESHOLD:
                    print("📦 ตรวจพบการใส่ของครั้งแรก")
                    state = STATE_MONITOR_MOVEMENT
                    break
                time.sleep(0.2)
            else:
                print("⏱️ หมดเวลาใส่ของ → ปิดมอเตอร์")
                state = STATE_CLOSE_SERVO

        elif state == STATE_MONITOR_MOVEMENT:
            last_motion_time = time.time()
            last_distance = read_sensor(index)
            print("🔁 เริ่มตรวจจับความเคลื่อนไหว...")

            while True:
                current = read_sensor(index)
                print(f"🛁 S{index + 1}: {current:.1f} mm")
                if abs(current - last_distance) >= CHANGE_THRESHOLD:
                    print("📍 พบการขยับ → รีเซ็ตตัวจับเวลา")
                    last_motion_time = time.time()
                    last_distance = current
                if time.time() - last_motion_time >= 3:
                    print("⏳ ไม่มีการขยับนาน 3 วิ → ปิดมอเตอร์")
                    break
                time.sleep(0.2)

            state = STATE_CLOSE_SERVO

        elif state == STATE_CLOSE_SERVO:
            print(f"🔒 ปิดมอเตอร์ช่อง {index + 1} (← 0°)")
            move_servo_180(index, 0)
            slot_status[index]["capacity_mm"] = read_sensor(index)
            slot_status[index]["door_closed"] = is_door_closed(index)
            slot_status[index]["available"] = slot_status[index]["capacity_mm"] == 0
            publish_status(index)
            state = STATE_DONE

    global reading_active
    reading_active = False

    if not is_door_closed(index):
        print(f"❌ ช่อง {index + 1}: Magnetic lock ปิดไม่สนิท!")
    else:
        print(f"✅ ช่อง {index + 1}: Magnetic lock ปิดเรียบร้อย")

# ==== DOOR OPEN SEQUENCE ====
def handle_door_open_sequence(index):
    if user_role not in ["admin", "professor"]:
        print("🚫 คุณไม่มีสิทธิ์เปิดประตูเอาเอกสารออก")
        return

    relay_pins[index].value = True
    print(f"🔓 เปิดประตูช่อง {index + 1} (Relay ON)")

    while is_door_closed(index):
        print("⏳ รอให้เปิดประตู...")
        time.sleep(0.2)

    while not is_door_closed(index):
        print("⏳ รอให้ปิดประตู...")
        time.sleep(0.2)

    relay_pins[index].value = False
    print(f"🔒 ปิดประตูช่อง {index + 1} (Relay OFF)")

    new_value = read_sensor(index)
    slot_status[index]["capacity_mm"] = new_value
    slot_status[index]["door_closed"] = is_door_closed(index)
    slot_status[index]["available"] = new_value == 0
    publish_status(index)

# ==== STATUS REPORT ====
def publish_status(index):
    data = slot_status[index]
    print(f"📤 ส่งข้อมูล Slot {index + 1} ไปยัง server: {data}")
    # TODO: send to MQTT or HTTP endpoint

# ==== SENSOR INIT ====
def init_xshuts():
    global xshuts
    xshuts = []
    for pin in XSHUT_PINS:
        x = digitalio.DigitalInOut(pin)
        x.direction = digitalio.Direction.OUTPUT
        x.value = False
        xshuts.append(x)
    time.sleep(0.2)

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

# ==== READ SENSOR ====
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

# ==== DOOR SWITCH CHECK ====
def is_door_closed(channel):
    try:
        pin = mcp_pins[8 + channel]  # B0-B3 → 8–11
        return pin.value == 0
    except:
        return False

# ==== LED STATUS UPDATE ====
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

# Warpper To  Mian-func

if __name__ == "__main__":
    main()
