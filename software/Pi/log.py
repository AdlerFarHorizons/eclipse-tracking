"""
Handles logging error messages to a file
This module can easily be extended to log
routine data.
Version: 5/01/17
Dependencies: none
"""

import time

date = lambda _=None: time.strftime("%a, %b %d %Y %H:%M:%S", time.localtime())

def setup():
    global logfile
    logfile = open('log.txt', 'a')
    logfile.write("========= Log File on " + date() + " =========\n")

def write_log(errortype, result):
    logfile.write("[" + errortype + "], "   + date() + ", " + result + "\n")

def close():
    logfile.close()
