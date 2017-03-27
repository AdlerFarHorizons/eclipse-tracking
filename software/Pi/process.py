"""
Detects brightest region in image and finds angles needed to rotate
to center it based on GoPro 4 FOV
Note: uncomment serial code when in use on Raspberry Pi 3
Algorithms: Gaussian Blur and Min/Max Loc
Dependencies: PySerial, Numpy, and OpenCV
Version: 3/27/17
"""

import numpy as np
import cv2
import serial

#myPort = serial.Serial('dev/serial0', 11520)

# The image will eventually be retrieved from a GoPro 4
image = cv2.imread("test.png")

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

if delta[0] < 0:
    result += '1'
else:
    result += '0'

if abs(delta[0]) < 100:
    result += '0'
    if abs(delta[0]) < 10:
        result += '0'

result += str(delta[0])

if delta[0] < 0:
    result += '1'
else:
    result += '0'

if abs(delta[1]) < 100:
    result += '0'
    if abs(delta[1]) < 10:
        result += '0'

result += str(delta[1])

#myPort.write(bytes(result))

cv2.imshow("Brightest Area", image)
cv2.waitKey(0)
