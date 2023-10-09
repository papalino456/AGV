"""
Line follower robot using pca9685, raspberry pi zero W, 2x L298N H-bridge and 4 gear motors
"""

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
pca.frequency = 400

class Motor:
    def __init__(self, pca, en_channel, in1_channel, in2_channel):
        self.pca = pca
        self.en = pca.channels[en_channel]
        self.in1 = pca.channels[in1_channel]
        self.in2 = pca.channels[in2_channel]

    def drive(self, direction, speed):
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
        self.en.duty_cycle = 0
        self.in1.duty_cycle = 0
        self.in2.duty_cycle = 0

IRsensor = digitalio.DigitalInOut(board.D26)
IRsensor.direction = digitalio.Direction.INPUT
while True:
    print(IRsensor.value)





"""
# Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits>
# but the PCA9685 will only actually give 12 bits of resolution.
# Increase brightness:
for i in range(0xffff):
    pca.channels[4].duty_cycle = i

# Decrease brightness:
for i in range(0xffff, 0, -1):
    pca.channels[4].duty_cycle = i
    """