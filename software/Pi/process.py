"""
Detects brightest region in image and finds angles needed to rotate
to center it based on GoPro 4 FOV
Note: uncomment serial code when in use on Raspberry Pi 3 (start with ###)
Algorithms: Gaussian Blur and Min/Max Loc
Python Version: 2.7
See trello for a comprehensive description of the communication protocol
Dependencies: PySerial, Numpy, and OpenCV
Version: 4/23/17
"""

import numpy as np
import cv2
import serial

#Constants
widthAngle = 13.58
heightAngle = 7.66
widthPixels = 1280
heightPixels = 720

conversion = (widthAngle / widthPixels, heightAngle / heightPixels)
center = (widthPixels / 2, heightPixels / 2)
###myPort = serial.Serial('dev/serial0', 11520)

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
        if d < 0:
            result += '1'
            d = abs(d)
        else:
            result += '0'
        for i in range(0, 3 - len(str(d))):
            result += '0'
        result += str(d)
    print(result)
    ###myPort.write(bytes(result))
