from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import board
import busio
import time
import digitalio
i2c_bus = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c_bus)

# Set the PWM frequency to 60hz.
pca.frequency = 700

class Motor:
    def __init__(self, pca, en_channel, in1_channel, in2_channel):
        self.pca = pca
        self.en = pca.channels[en_channel]
        self.in1 = pca.channels[in1_channel]
        self.in2 = pca.channels[in2_channel]

    def drive(self, direction, speed): #2 = front
        # Convert speed from 0-100 to 0-0xffff
        speed = int(speed * 655.35)
        self.en.duty_cycle = speed
        if direction == 1:
            self.in1.duty_cycle = 0xffff
            self.in2.duty_cycle = 0
        elif direction == 2:
            self.in1.duty_cycle = 0
            self.in2.duty_cycle = 0xffff
    def stop(self):
        self.en.duty_cycle = 1
        self.in1.duty_cycle = 0
        self.in2.duty_cycle = 0

motorFL = Motor(pca, 15, 14, 13)
motorFR = Motor(pca, 9, 7, 8)
motorBL = Motor(pca, 10, 11, 12)
motorBR = Motor(pca, 4, 6, 5)

motorBR.stop()
motorBL.stop()
motorFR.stop()
motorFL.stop()
time.sleep(.5)
motorFL.drive(2, 25)
motorFR.drive(2, 25)
motorBL.drive(2, 25)
motorBR.drive(2, 25)
time.sleep(1)
motorBR.stop()
motorBL.stop()
motorFR.stop()
motorFL.stop()
time.sleep(3)
motorFL.drive(1, 30)
motorFR.drive(1, 30)
motorBL.drive(1, 30)
motorBR.drive(1, 30)
time.sleep(1.5)
motorBR.stop()
motorBL.stop()
motorFR.stop()
motorFL.stop()
time.sleep(.5)
motorFL.drive(2, 20)
motorFR.drive(1, 20)
motorBL.drive(2, 20)
motorBR.drive(1, 20)
time.sleep(2)
motorBR.stop()
motorBL.stop()
motorFR.stop()
motorFL.stop()