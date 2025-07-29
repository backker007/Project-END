import time
import board
import busio
from adafruit_pca9685 import PCA9685

# สร้าง I2C และ PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

channel = 0  # ช่องเซอร์โว

# ฟังก์ชันแปลง "องศา" เป็น duty_cycle (16-bit)
def angle_to_duty(angle):
    pulse_us = 500 + (angle / 180.0) * 2000  # 500–2500us
    duty = int((pulse_us / 20000.0) * 65535)
    return duty

# ฟังก์ชันขยับเซอร์โวจากมุม A → B ด้วยความเร็วคงที่
def move_servo_smooth(from_angle, to_angle, total_duration_sec):
    steps = abs(to_angle - from_angle)
    if steps == 0:
        return
    step_delay = total_duration_sec / steps
    step = 1 if to_angle > from_angle else -1

    for angle in range(from_angle, to_angle + step, step):
        duty = angle_to_duty(angle)
        pca.channels[channel].duty_cycle = duty
        time.sleep(step_delay)

    # หยุดส่ง PWM (ไม่ให้เซอร์โวเกร็ง)
    pca.channels[channel].duty_cycle = 0

# ===== ทดสอบหมุนไป–กลับด้วยความเร็วคงที่ =====
try:
    while True:
        print("→ หมุนจาก 0° → 180°")
        move_servo_smooth(0, 180, total_duration_sec=4.0)  # หมุนใช้เวลา 2 วิ

        time.sleep(1)

        print("← หมุนกลับ 180° → 0°")
        move_servo_smooth(180, 0, total_duration_sec=4.0)

        time.sleep(1)

except KeyboardInterrupt:
    pca.channels[channel].duty_cycle = 0  
    print("🛑 หยุดการทำงาน")
