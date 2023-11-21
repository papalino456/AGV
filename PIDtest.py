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
        self.direction = 0
        self.Kp = 100  # Proportional gain
        self.Ki = 10  # Integral gain
        self.last_time = time.time()
        GPIO.setup(encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(encoder_pin, GPIO.RISING, callback=self.calculate_speed)
        self.lock = threading.Lock()


    def drive(self, direction, setpoint):  # setpoint in pulses per second
        self.setpoint = setpoint
        self.direction = direction

    def control_speed(self):
        with self.lock:
            error = self.setpoint - self.speed
        self.error_sum += error
        self.control_value = self.Kp * error + self.Ki * self.error_sum
        # Ensure control_value is within 0-0xffff
        self.satcontrol_value = max(0, min(0xffff, int(self.control_value)))
        self.en.duty_cycle = self.satcontrol_value
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
        with self.lock:
            self.current_time = time.time()
            self.pulse_time = self.current_time - self.last_time
            self.speed = 1 / self.pulse_time  # Speed in pulses per second
            self.last_time = self.current_time

motorFL = Motor(pca, 0, 1, 2,27)
motorFL.start_control_loop()

while True:
    #5 second tests
    motorFL.Kp = float(input("Kp: ",motorFL.Kp," new: "))
    motorFL.Ki = float(input("Ki: ",motorFL.Ki," new: "))
    motorFL.drive(2, 500)
    timeCount = 0
    while timeCount < 5.0:
        print("Speed: ", int(motorFL.speed),", Error: ",int(motorFL.error),", SP: ",int(motorFL.setpoint))
        timeCount += 0.01
        time.sleep(0.01)

