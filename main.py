"""
Line follower robot using pca9685, raspberry pi zero W, 2x L298N H-bridge and 4 gear motors
"""

from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import board
import busio
import adafruit_hcsr04
import time
import digitalio
i2c_bus = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c_bus)
count=0
# Set the PWM frequency to 60hz.
pca.frequency = 700

USsensor = adafruit_hcsr04.HCSR04(trigger_pin=board.D26, echo_pin=board.D25, timeout=0.06)
#USsensor2 = adafruit_hcsr04.HCSR04(trigger_pin=board.D20, echo_pin=board.D25, timeout=0.06)

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

motorFL = Motor(pca, 0, 1, 2)
motorFR = Motor(pca, 6, 7, 8)
motorBL = Motor(pca, 5, 3, 4)
motorBR = Motor(pca, 11, 10, 9)

IRsensorL = digitalio.DigitalInOut(board.D21)
IRsensorR = digitalio.DigitalInOut(board.D16)
#IRsensorR2 = digitalio.DigitalInOut(board.D24)
#IRsensorL2 = digitalio.DigitalInOut(board.D23)

IRsensorL.direction = digitalio.Direction.INPUT
IRsensorR.direction = digitalio.Direction.INPUT
#IRsensorL2.direction = digitalio.Direction.INPUT
#IRsensorR2.direction = digitalio.Direction.INPUT

IRsensorL.pull = digitalio.Pull.UP
IRsensorR.pull = digitalio.Pull.UP
#IRsensorL.pull = digitalio.Pull.UP
#IRsensorR.pull = digitalio.Pull.UP

dist = 16

while True:
    valL = IRsensorL.value
    valR = IRsensorR.value
    print(valL)
    print(valR)
    try:
        dist = USsensor.distance
        print(dist)
    except RuntimeError:
        dist = 11
        print("nononononono")
    if dist < 10 and dist > 1 and not valR and not valL:
        motorBR.stop()
        motorBL.stop()
        motorFR.stop()
        motorFL.stop()
        time.sleep(5)
    if valR and valL is True:
        count += 1
        if count % 2 == 0:
            motorBR.stop()
            motorBL.stop()
            motorFR.stop()
            motorFL.stop()
            time.sleep(.5)
            motorFL.drive(2, 30)
            motorFR.drive(2, 30)
            motorBL.drive(2, 30)
            motorBR.drive(2, 30)
            time.sleep(0.5)
            motorBR.stop()
            motorBL.stop()
            motorFR.stop()
            motorFL.stop()
            time.sleep(5)
            motorFL.drive(1, 30)
            motorFR.drive(1, 30)
            motorBL.drive(1, 30)
            motorBR.drive(1, 30)
            time.sleep(0.8)
            motorBR.stop()
            motorBL.stop()
            motorFR.stop()
            motorFL.stop()
            time.sleep(.5)
            motorFL.drive(2, 33)
            motorFR.drive(1, 33)
            motorBL.drive(2, 33)
            motorBR.drive(1, 33)
            time.sleep(1.1)
            motorBR.stop()
            motorBL.stop()
            motorFR.stop()
            motorFL.stop()
    if valL:  # If line is detected by the left sensor
        # Move slightly tothe right
        motorFL.drive(2, 30)
        motorFR.drive(1, 20)
        motorBL.drive(2, 30)
        motorBR.drive(1, 20)
        time.sleep(0.05)
    elif valR:  # If line is detected by the right sensor
        # Move slightly to the left
        motorFL.drive(1, 20)
        motorFR.drive(2, 30)
        motorBL.drive(1, 20)
        motorBR.drive(2, 30)
        time.sleep(0.05)
    else:  # If line is not detected
        # Stop
        motorFL.drive(2, 22)
        motorFR.drive(2, 22)
        motorBL.drive(2, 22)
        motorBR.drive(2, 22)