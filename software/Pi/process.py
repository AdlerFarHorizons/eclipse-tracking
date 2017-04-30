"""
Detects brightest region in image and finds angles needed to rotate
to center it based on GoPro 4 FOV
Note: uncomment code commented out with ### when in use on Raspberry Pi 3
Algorithms: Gaussian Blur and Min/Max Loc
Python Version: 2.7
See trello for a comprehensive description of the communication protocol
Dependencies: PySerial, Numpy, and OpenCV
Version: 4/30/17
"""

import numpy as np
import cv2
import serial
import time
###import RPi.GPIO as gpio
import gopro_wifi_api_hero4

#Constants
widthAngle   = 13.58
heightAngle  = 7.66
widthPixels  = 1280
heightPixels = 720

logfile      = open('log.txt', 'a')
ledPin       = 7

conversion   = (widthAngle / widthPixels, heightAngle / heightPixels)
center       = (widthPixels / 2, heightPixels / 2)

#Set up RPi GPIO
###gpio.setmode(gpio.BOARD)
###gpio.setup(ledPin, gpio.OUTPUT)

#Helper functions
date = lambda _=None: time.strftime("%a, %b %d %Y %H:%M:%S", time.localtime())
def blink(ledPin):
    gpio.output(ledPin, True)
    time.sleep(0.5)
    gpio.output(ledPin, False)
    time.sleep(0.5)

logfile.write("========= Log File on " + date() + " =========\n")

try: myPort = serial.Serial('dev/serial0', 115200)
except:
    ###while True:
        ###blink(ledPin)
    logfile.write("[serial cnnct error], " + date() + ", --------" + "\n")

#Loop
while True:
    # The image will eventually be retrieved from a GoPro 4
    image = cv2.imread("myTest.png")

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (41, 41), 0)

    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)

    result = ""

    for i in range(0, 2):
        d = int((maxLoc[i] - center[i]) * conversion[i])
        if (d == 0):
            result += '0000'
            continue
        elif d < 0:
            result += '1'
            d = abs(d)
        else:
            result += '0'
        for i in range(0, 3 - len(str(d))):
            result += '0'
        result += str(d)
    #send the data
    try: myPort.write(bytes(result))
    except: logfile.write("[serial write error], "   + date() + ", " + result + "\n")

logfile.close()
