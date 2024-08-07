# Line Follower Robot

This project implements an automatic guided vehicle robot using a Raspberry Pi Zero W, PCA9685 PWM controller, L298N H-bridges, and four gear motors. The robot is designed to follow a line using IR sensors and can be controlled remotely using a gamepad.

## Features

- Line following using IR sensors
- Remote control using a gamepad
- Ultrasonic sensor for obstacle detection
- PID control for motor speed

## Hardware Requirements

- Raspberry Pi Zero W
- PCA9685 PWM controller
- 2x L298N H-bridge motor drivers
- 4x Gear motors with encoders
- IR sensors
- Ultrasonic sensor
- Gamepad controller (for remote control)

## Software Dependencies

- adafruit_pca9685
- adafruit_hcsr04
- RPi.GPIO
- inputs (for gamepad control)

## File Structure

- `main.py`: Main line follower logic
- `stop.py`: Script to stop all motors
- `PIDtest.py`: PID control testing for motor speed
- `control.py`: Remote control using a gamepad
- `Sensortest.py`: Test script for IR and ultrasonic sensors
- `turnTest.py`: Test script for turning movements

## Setup and Usage

1. Connect the hardware components according to the pin configurations in the code.
2. Install the required dependencies:
   ```
   pip install adafruit-circuitpython-pca9685 adafruit-circuitpython-hcsr04 RPi.GPIO inputs
   ```
3. Run the desired script:
   - For line following: `python main.py`
   - For remote control: `python control.py`
   - For motor stop: `python stop.py`
   - For PID testing: `python PIDtest.py`
   - For sensor testing: `python Sensortest.py`
   - For turn testing: `python turnTest.py`

## Configuration

You can adjust various parameters in the code to fine-tune the robot's behavior:

- Motor speed and direction
- PID control parameters (Kp, Ki)
- Sensor thresholds
- PWM frequency

## TODO

- Implement full PID control for smoother line following
- Improve turning algorithms for better performance
