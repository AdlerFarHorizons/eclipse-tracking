"""
Detects brightest region in image and finds angles needed to rotate
to center it based on GoPro 4 FOV
Algorithms: Gaussian Blur and Min/Max Loc
Python Version: 2.7.12
See trello for a comprehensive description of the communication protocol
Dependencies: PySerial, Numpy, time, gopro_wifi_api_hero4, log, ui, and OpenCV
Version: 5/29/17
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

sunThreshold = 250 # to be determined by experimentation

conversion   = widthAngle / widthPixels
center       = (widthPixels / 2, heightPixels / 2)

myPort = None

#Returns a black and white image with a Gaussian Blur
def get_image():
    api.capture()
    api.transfer_latest_photo()
    subprocess.Popen('mv *.JPG image.JPG', shell=True)
    image = cv2.imread("image.JPG")
    subprocess.Popen('rm *.JPG', shell=True)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (41, 41), 0)
    return gray

#Returns true if the sun is in the image
def sun_in_image():
    gray = get_image()
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
    if maxVal >= sunThreshold:
        return True
    return False

#Calculates and formats degrees to move stepper to center the sun
def getFormattedDelta(maxLocX, centerX):
    result = ""
    #Calculate degrees to move
    delta = int((maxLocX - centerX) * conversion)
    str_delta = str(delta)
    #Handle positive, negative, or zero
    if (delta == 0):
        return '0000'
    elif delta < 0:
        result = '1'
        delta = abs(delta)
    else:
        result = '0'
    #Pad result with zeros
    for i in range(0, 3 - len(str_delta)):
        result += '0'
    #Add in the actual value
    result += str_delta
    return result

#Set up hardware and logging
def setup():
    ui.setup()
    log.setup()
    myPort = serial.Serial('/dev/serial0', 115200)
    api.initialize_camera()
    ui.blink(3)
    while True:
        if sun_in_image():
            myPort.write(b'0') # tell the arduino to start correcting
            break

#Correction loop
def main():
    while True:
        #Retrieve image
        gray = get_image()
        #Find the brightest region
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
        #Calculate and format offset
        result = getFormattedDelta(maxLoc[0], center[0])
        #Send result to the Arduino Uno and log it
        try:
            myPort.write(bytes(result))
            log.write_log('normal exec_', result)
        except:
            log.write_log('serial error', result)
        #Wait for half a second
        time.sleep(7)

    log.close()

if __name__ == '__main__':
    setup()
    main()
