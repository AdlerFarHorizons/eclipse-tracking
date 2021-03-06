"""
Handles LED output for the Raspberry Pi 3
Image tracking software. Imported using 'import ui'
Version: 5/06/17
Dependencies: RPi.GPIO
Note: will only work on a Raspberry Pi!
"""
import RPi.GPIO as gpio
import time

ledPin = 16 #GPIO23

#Set up RPi GPIO
def setup():
    gpio.setmode(gpio.BOARD)
    gpio.setwarnings(False)
    gpio.setup(ledPin, gpio.OUT)

def blink(n):
    for i in range(0, n):
        gpio.output(ledPin, True)
        time.sleep(0.5)
        gpio.output(ledPin, False)
        time.sleep(0.5)
