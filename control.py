
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import board
import busio
import time
import digitalio
from inputs import get_gamepad, devices

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

motorFL = Motor(pca, 15, 14, 13)
motorFR = Motor(pca, 9, 7, 8)
motorBL = Motor(pca, 10, 11, 12)
motorBR = Motor(pca, 4, 6, 5)

def process_controller():
    while True:
        events = get_gamepad()
        for event in events:
            if event.code == 'ABS_HAT0Y':  # Dpad Y-axis (forward/backward)
                speed = event.state  # Dpad values are -1, 0, or 1
                motorFL.drive(1 if speed <= 0 else 2, abs(speed)*100)
                motorFR.drive(1 if speed <= 0 else 2, abs(speed)*100)
                motorBL.drive(1 if speed <= 0 else 2, abs(speed)*100)
                motorBR.drive(1 if speed <= 0 else 2, abs(speed)*100)
            elif event.code == 'ABS_HAT0X':  # Dpad X-axis (left/right)
                strafe_speed = event.state  # Dpad values are -1, 0, or 1
                if strafe_speed >= 0:  # Strafe right
                    motorFL.drive(1, abs(strafe_speed)*100)
                    motorBL.drive(2, abs(strafe_speed)*100)
                    motorFR.drive(2, abs(strafe_speed)*100)
                    motorBR.drive(1, abs(strafe_speed)*100)
                else:  # Strafe left
                    motorFL.drive(2, abs(strafe_speed)*100)
                    motorBL.drive(1, abs(strafe_speed)*100)
                    motorFR.drive(1, abs(strafe_speed)*100)
                    motorBR.drive(2, abs(strafe_speed)*100)
            elif event.code == 'ABS_RX':  # Right joystick X-axis (turning)
                turn_speed = event.state / 330  # Normalize to 0-100
                if turn_speed >= 0:  # Turn right
                    motorFL.drive(1, abs(turn_speed))
                    motorBL.drive(1, abs(turn_speed))
                    motorFR.drive(2, abs(turn_speed))
                    motorBR.drive(2, abs(turn_speed))
                else:  # Turn left
                    motorFL.drive(2, abs(turn_speed))
                    motorBL.drive(2, abs(turn_speed))
                    motorFR.drive(1, abs(turn_speed))
                    motorBR.drive(1, abs(turn_speed))

# Start processing controller input
while len(devices.gamepads) == 0:
    print("Waiting for controller...")
    time.sleep(1)
    
process_controller()

