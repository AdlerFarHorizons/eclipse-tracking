"""
Detects brightest region in image and finds angles needed to rotate
to center it based on GoPro 4 FOV
Note: uncomment serial code when in use on Raspberry Pi 3 (start with ###)
Algorithms: Gaussian Blur and Min/Max Loc
Dependencies: PySerial, Numpy, and OpenCV
Version: 3/27/17
"""

import numpy as np
import cv2
import serial

###myPort = serial.Serial('dev/serial0', 11520)

while True:
    # The image will eventually be retrieved from a GoPro 4
    image = cv2.imread("myTest.png")

    widthAngle = 13.58
    heightAngle = 7.66
    widthPixels = 1280
    heightPixels = 720

    center = (widthPixels / 2, heightPixels / 2)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (41, 41), 0)

    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)

    cv2.circle(image, maxLoc, 10, (255, 0, 0), 2)

    # negative means move right
    delta = [maxLoc[0] - center[0], maxLoc[1] - center[1]]

    delta[0] *= widthAngle / widthPixels
    delta[1] *= heightAngle / heightPixels

    delta[0] = int(delta[0])
    delta[1] = int(delta[1])

    result = ""

    for d in delta:
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

    print result

    ###myPort.write(bytes(result))
