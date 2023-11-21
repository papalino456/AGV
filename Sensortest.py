import time
import board
import digitalio
import adafruit_hcsr04

sonar = adafruit_hcsr04.HCSR04(trigger_pin=board.D20, echo_pin=board.D25,timeout=0.06)

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

while True:

    valL =  IRsensorL.value
    print("valL: ", valL)
    valR =  IRsensorR.value
    print("valR: ", valR)
"""
    try:
        print((sonar.distance))
    except RuntimeError:
        print("Retrying!")
    time.sleep(0.1)
    """