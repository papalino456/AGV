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
speedHIGH = 50
speedLOW = 25

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

motorFL = Motor(pca, 15, 14, 13)
motorFR = Motor(pca, 9, 7, 8)
motorBL = Motor(pca, 10, 11, 12)
motorBR = Motor(pca, 4, 6, 5)

IRsensorFL = digitalio.DigitalInOut(board.D12)
IRsensorL = digitalio.DigitalInOut(board.D21)
IRsensorC = digitalio.DigitalInOut(board.D20)
IRsensorR = digitalio.DigitalInOut(board.D16)
IRsensorFR = digitalio.DigitalInOut(board.D1)

IRsensorFL.direction = digitalio.Direction.INPUT
IRsensorL.direction = digitalio.Direction.INPUT
IRsensorC.direction = digitalio.Direction.INPUT
IRsensorR.direction = digitalio.Direction.INPUT
IRsensorFR.direction = digitalio.Direction.INPUT

IRsensorFL.pull = digitalio.Pull.UP
IRsensorL.pull = digitalio.Pull.UP
IRsensorC.pull = digitalio.Pull.UP
IRsensorR.pull = digitalio.Pull.UP
IRsensorFR.pull = digitalio.Pull.UP

while True:
    valFL = not IRsensorFL.value
    valL = not IRsensorL.value
    valC = not IRsensorC.value
    valR = not IRsensorR.value
    valFR = not IRsensorFR.value

    if not valC:  # If line is detected by the center sensor
        # Move forward
        motorFL.drive(2, 40)
        motorFR.drive(2, 40)
        motorBL.drive(2, 40)
        motorBR.drive(2, 40)
    elif not valL:  # If line is detected by the left sensor
        # Move slightly tothe right
        motorFL.drive(2, 25)
        motorFR.drive(1, 20)
        motorBL.drive(2, 25)
        motorBR.drive(1, 20)
        time.sleep(0.2)
    elif not valR:  # If line is detected by the right sensor
        # Move slightly to the left
        motorFL.drive(1, 20)
        motorFR.drive(2, 25)
        motorBL.drive(1, 20)
        motorBR.drive(2, 25)
        time.sleep(0.2)
    elif not valFR:
        motorFL.drive(2, 25)
        motorFR.stop()
        motorBL.drive(2, 25)
        motorBR.stop()
        time.sleep(0.2)
    elif not valFL:  # If line is detected by the right sensor
        # Move slightly to the left
        motorFL.stop()
        motorFR.drive(2, 25)
        motorBL.stop()
        motorBR.drive(2, 25)
        time.sleep(0.2)
    else:  # If line is not detected
        # Stop
        motorFL.drive(2, 40)
        motorFR.drive(2, 40)
        motorBL.drive(2, 40)
        motorBR.drive(2, 40)





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