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
import RPi.GPIO as GPIO
import threading

i2c_bus = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c_bus)

# Set the PWM frequency to 60hz.
pca.frequency = 700

USsensor = adafruit_hcsr04.HCSR04(trigger_pin=board.D20, echo_pin=board.D25, timeout=0.06)
#USsensor2 = adafruit_hcsr04.HCSR04(trigger_pin=board.D26, echo_pin=board.D12, timeout=0.06)

class Motor:
    def __init__(self, pca, en_channel, in1_channel, in2_channel, encoder_pin):
        self.pca = pca
        self.en = pca.channels[en_channel]
        self.in1 = pca.channels[in1_channel]
        self.in2 = pca.channels[in2_channel]
        self.encoder_pin = encoder_pin
        self.speed = 0
        self.setpoint = 0
        self.error = 0
        self.error_sum = 0
        self.Kp = 100  # Proportional gain
        self.Ki = 10  # Integral gain
        self.last_time = time.time()
        GPIO.setup(encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(encoder_pin, GPIO.RISING, callback=self.calculate_speed)


    def drive(self, direction, setpoint):  # setpoint in pulses per second
        self.setpoint = setpoint
        self.direction = direction

    def control_speed(self):
        self.error = self.setpoint - self.speed
        self.error_sum += self.error
        control_value = self.Kp * self.error + self.Ki * self.error_sum
        # Ensure control_value is within 0-0xffff
        control_value = max(0, min(0xffff, int(control_value)))
        self.en.duty_cycle = control_value
        if self.direction == 1:
            self.in1.duty_cycle = 0xffff
            self.in2.duty_cycle = 0
        elif self.direction == 2:
            self.in1.duty_cycle = 0
            self.in2.duty_cycle = 0xffff   
     
    def start_control_loop(self):
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()

    def stop_control_loop(self):
        self.control_loop_running = False
        self.control_thread.join()

    def control_loop(self):
        self.control_loop_running = True
        while self.control_loop_running:
            self.control_speed()
            time.sleep(0.01)  # Control loop frequency     
    def stop(self):
        self.en.duty_cycle = 1
        self.in1.duty_cycle = 0
        self.in2.duty_cycle = 0

    def calculate_speed(self, channel):
        current_time = time.time()
        pulse_time = current_time - self.last_time
        self.speed = 1 / pulse_time  # Speed in pulses per second
        self.last_time = current_time

    def start_speed_measurement(self):
        self.speed_thread = threading.Thread(target=self.calculate_speed)
        self.speed_thread.start()

    def stop_speed_measurement(self):
        GPIO.remove_event_detect(self.encoder_pin)
        self.speed_thread.join()

motorFL = Motor(pca, 0, 1, 2,27)
motorFR = Motor(pca, 6, 8, 7,17)
motorBL = Motor(pca, 5, 3, 4, 22)
motorBR = Motor(pca, 11, 10, 9, 10)

motorFL.start_speed_measurement()
motorFR.start_speed_measurement()
motorBL.start_speed_measurement()
motorBR.start_speed_measurement()

motorFL.start_control_loop()
# Do the same for the other motors


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
motorFL.drive(2, 500)
while True:
    print("Speed: ", int(motorFL.speed),", Error: ",int(motorFL.error),", SP: ",int(motorFL.setpoint))

