import busio
import digitalio
import time
from adafruit_mcp230xx.mcp23017 import MCP23017

# ใช้ I2C-0
i2c0 = busio.I2C(1,0)

# สร้าง MCP23017 instance
mcp = MCP23017(i2c0)

# กำหนดขา
pin1 = mcp.get_pin(12)
pin2 = mcp.get_pin(13)
pin3 = mcp.get_pin(14)
pin4 = mcp.get_pin(15)

# ตั้งค่า Output
pin1.direction = digitalio.Direction.OUTPUT
pin2.direction = digitalio.Direction.OUTPUT
pin3.direction = digitalio.Direction.OUTPUT
pin4.direction = digitalio.Direction.OUTPUT

while True:
    pin1.value = False
    pin2.value = True
    pin3.value = True
    pin4.value = True
    time.sleep(1)

    pin1.value = True
    pin2.value = False
    pin3.value = True
    pin4.value = True
    time.sleep(1)

    pin1.value = True
    pin2.value = True
    pin3.value = False
    pin4.value = True
    time.sleep(1)

    pin1.value = True
    pin2.value = True
    pin3.value = True
    pin4.value = False
    time.sleep(1)
