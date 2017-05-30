"""
Detects brightest region in image and finds angles needed to rotate
to center it based on GoPro 4 FOV
Algorithms: Gaussian Blur and Min/Max Loc
Python Version: 2.7
See trello for a comprehensive description of the communication protocol
Dependencies: PySerial, Numpy, time, gopro_wifi_api_hero4, log, ui, and OpenCV
Version: 5/13/17
"""

import numpy as np
import cv2
import serial
import gopro_wifi_api_hero3 as api
import log
import ui
import time
import subprocess

#Constants
widthAngle   = 13.58
heightAngle  = 7.66
widthPixels  = 1280
heightPixels = 720

sunThreshold = 0 # to be determined by experimentation

conversion   = widthAngle / widthPixels
center       = (widthPixels / 2, heightPixels / 2)

myPort = None

#Returns true if the sun is in the image
def sun_in_image():
    api.capture()
    api.transfer_latest_photo()
    subprocess.Popen('mv *.JPG image.JPG', shell=True)
    image = cv2.imread("image.JPG")
    subprocess.Popen('rm *.JPG', shell=True)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (41, 41), 0)

    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
    if maxVal >= sunThreshold:
        return True
    return False

#Set up hardware and logging
def setup():
    ui.setup()
    log.setup()
    myPort = serial.Serial('/dev/serial0', 115200)
    ui.blink(3)
    while True:
        if sun_in_image():
            myPort.write(b'0') # tell the arduino
            break

#Correction loop
def main():
    while True:
        # Retrieve image
        api.capture()
        api.transfer_latest_photo()
        subprocess.Popen('mv *.JPG image.JPG', shell=True)
        image = cv2.imread("image.JPG")
        subprocess.Popen('rm *.JPG', shell=True)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (41, 41), 0)

        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)

        result = ""

        delta = int((maxLoc[0] - center[0]) * conversion)
        if (delta == 0):
            result = '0000'
        elif delta < 0:
            result += '1'
            delta = abs(d)
        else:
            result += '0'
        for i in range(0, 3 - len(str(delta))):
            result += '0'
        result += str(delta)

        try:
            myPort.write(bytes(result))
            log.write_log('normal exec_', result)
        except:
            log.write_log('serial error', result)
	
        time.sleep(0.5)

    log.close()

if __name__ == '__main__':
    setup()
    main()
