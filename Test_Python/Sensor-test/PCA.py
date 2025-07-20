import board
import busio
import digitalio
import adafruit_vl53l0x
import time
import os
import sys
import select
import tty
import termios
from collections import deque
from smbus2 import SMBus
from adafruit_mcp230xx.mcp23017 import MCP23017
from digitalio import Direction

# ==== CONFIG ====
XSHUT_PINS = [board.D17, board.D27, board.D22, board.D5]
ADDRESS_BASE = 0x30
OFFSET_MM = 0
BUFFER_SIZE = 5
CHANGE_THRESHOLD = 5
ZERO_THRESHOLD = 70
CHECK_INTERVAL = 120  # 2 minutes
SERVO_CHANNELS = [0, 1, 2, 3]
PCA9685_ADDRESS = 0x40

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

# ==== INIT I2C ====
i2c_vl53 = busio.I2C(board.SCL, board.SDA)  # VL53L0X on bus 1

try:
    i2c_bus0 = SMBus(0)  # MCP23017 & PCA9685 on bus 0
except Exception as e:
    print(f"❌ I2C init error (bus 0): {e}")
    sys.exit(1)

# ==== PCA9685 ADAPTER ====
class PCA9685:
    def __init__(self, bus: SMBus, address: int = 0x40):
        self.bus = bus
        self.address = address
        self.set_all_pwm(0, 0)
        self.write8(0x00, 0x20)  # MODE1 restart
        time.sleep(0.005)
        self.write8(0x01, 0x04)  # MODE2 OUTDRV

    def set_pwm_freq(self, freq_hz: float):
        prescaleval = 25000000.0
        prescaleval /= 4096.0
        prescaleval /= float(freq_hz)
        prescaleval -= 1.0
        prescale = int(prescaleval + 0.5)

        oldmode = self.read8(0x00)
        newmode = (oldmode & 0x7F) | 0x10
        self.write8(0x00, newmode)
        self.write8(0xFE, prescale)
        self.write8(0x00, oldmode)
        time.sleep(0.005)
        self.write8(0x00, oldmode | 0x80)

    def set_pwm(self, channel: int, on: int, off: int):
        self.write8(0x06 + 4 * channel, on & 0xFF)
        self.write8(0x07 + 4 * channel, on >> 8)
        self.write8(0x08 + 4 * channel, off & 0xFF)
        self.write8(0x09 + 4 * channel, off >> 8)

    def set_all_pwm(self, on: int, off: int):
        self.write8(0xFA, on & 0xFF)
        self.write8(0xFB, on >> 8)
        self.write8(0xFC, off & 0xFF)
        self.write8(0xFD, off >> 8)

    def write8(self, reg: int, value: int):
        self.bus.write_byte_data(self.address, reg, value)

    def read8(self, reg: int) -> int:
        return self.bus.read_byte_data(self.address, reg)

# ==== INIT PCA9685 ====
try:
    pca = PCA9685(i2c_bus0, address=PCA9685_ADDRESS)
    pca.set_pwm_freq(50)
except Exception as e:
    print(f"❌ PCA9685 init error: {e}")
    print("💡 ตรวจสอบการเชื่อมต่อและ address (ใช้ i2cdetect -y 0)")
    sys.exit(1)

# ==== SERVO CONTROL ====
def set_servo_angle(channel: int, angle: float):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 50       # 50 Hz
    pulse_length //= 4096     # 12 bits of resolution
    pulse = int((angle * 2.0 / 180.0 + 0.5) * 1000 / pulse_length)
    pca.set_pwm(channel, 0, pulse)

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
                result = read_sensor(selected_sensor_index)
                if result >= 0:
                    print(f"S{selected_sensor_index + 1}: {result:.1f} mm")
                else:
                    print(f"S{selected_sensor_index + 1}: ERROR")
                time.sleep(0.1)

                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    if key == str(selected_sensor_index + 1):
                        relay_pins[selected_sensor_index].value = False
                        set_servo_angle(selected_sensor_index, 0)
                        reading_active = False

                        if not is_door_closed(selected_sensor_index):
                            print(f"❌ ช่อง {selected_sensor_index + 1}: Magnetic lock ปิดไม่สนิท!")
                        else:
                            print(f"✅ ช่อง {selected_sensor_index + 1}: Magnetic lock ปิดเรียบร้อย")
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# ==== INIT ====
def init_sensors():
    global vl53_sensors, buffers, last_values
    vl53_sensors.clear()
    buffers.clear()
    last_values.clear()
    for i, x in enumerate(xshuts):
        x.value = True
        time.sleep(0.2)
        try:
            sensor = adafruit_vl53l0x.VL53L0X(i2c_vl53)
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
    os.system("sudo i2cdetect -y 0 > /dev/null 2>&1")
    time.sleep(0.5)

def reset_vl53_addresses():
    for i, x in enumerate(xshuts):
        x.value = True
        time.sleep(0.1)
        try:
            sensor = adafruit_vl53l0x.VL53L0X(i2c_vl53, address=ADDRESS_BASE + i)
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
    mcp = MCP23017(i2c_bus0)

    relay_pin_nums = [12, 13, 14, 15]
    relay_pins.clear()
    mcp_pins.clear()

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
        pin = mcp_pins[8 + channel]
        low_count = 0
        for _ in range(15):
            if pin.value == 0:
                low_count += 1
            time.sleep(0.066)
        return low_count >= 12
    except:
        return False

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
    print("📱 Ready. เลือก Relay (1–4) เพื่อเริ่มอ่าน หรือ 0 เพื่อปิดทั้งหมด:")
    val = input().strip()
    if val in ["0", "1", "2", "3", "4"]:
        n = int(val)
        selected_sensor_index = n - 1 if n > 0 else None
        reading_active = n > 0
        for i, pin in enumerate(relay_pins):
            pin.value = (i == selected_sensor_index) if reading_active else False
        if reading_active:
            set_servo_angle(selected_sensor_index, 90)

# ==== ENTRY ====
if __name__ == "__main__":
    main()
