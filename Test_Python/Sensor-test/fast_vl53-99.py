import board
import busio
import time
import adafruit_vl53l0x

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_vl53l0x.VL53L0X(i2c)

while True:
    try:
        print("Distance: {} mm".format(sensor.range))
        time.sleep(0.5)
    except:
        print("Read error")
