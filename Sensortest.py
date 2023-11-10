import time
import board
import digitalio
import adafruit_hcsr04

sonar = adafruit_hcsr04.HCSR04(trigger_pin=board.D20, echo_pin=board.D12)
"""
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
"""

while True:
    """
    valFL = not IRsensorFL.value
    print("valFL: ", valFL)
    valL = not IRsensorL.value
    print("valL: ", valL)
    valC = not IRsensorC.value
    print("valC: ", valC)
    valR = not IRsensorR.value
    print("valR: ", valR)
    valFR = not IRsensorFR.value
    print("valFR: ", valFR)
    """
    try:
        print((sonar.distance))
    except RuntimeError:
        print("Retrying!")
    time.sleep(0.1)