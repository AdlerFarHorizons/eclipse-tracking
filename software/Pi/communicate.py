"""
A program for Raspberry Pi to Arduino communication
Uses the PySerial library to send commands / data
Version: 3/18/17
"""
import serial

myPort = serial.Serial('dev/tty/ACM0', 115200)

myPort.write(b'AAAA')

myPort.close()
