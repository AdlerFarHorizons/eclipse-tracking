"""
GoPro HERO 3 WiFi API for sending /receiving commands
"""

import urllib2
import json
import numpy as np
from contextlib import closing

# control URLs

SETTINGS_URL = "http://10.5.5.9/camera/sx?t={pwd}"
MEDIA_URL = "http://10.5.5.9:8080/gp/gpMediaList?t={pwd}"


BASE_COMMAND_URL = "http://10.5.5.9/{param1}/{param2}?t={pwd}&p=%{opt}"
PASSWORD = "100kfeet"

MEDIA_LOCATION = "http://10.5.5.9:8080/videos/DCIM/139GOPRO/{filename}"

def _send_command(target_url):
    with closing(urllib2.urlopen(target_url)) as f:
        return f.read()


def get_camera_status():
    # TODO: byte parsing
    return _send_command(SETTINGS_URL.format(pwd=PASSWORD))

def get_media_list(file_extension='.JPG', latest=False):
    d = json.loads(_send_command(MEDIA_URL.format(pwd=PASSWORD)))
    return [ix['n']
            for ix in d['media'][0]['fs']
            if ix['n'][-len(file_extension):] == file_extension]

def transfer_latest_photo():
    latest_photo = sorted(get_media_list())[-1]
    with open(latest_photo, "w") as photo:
        photo.write(_send_command(MEDIA_LOCATION.format(filename=latest_photo)))

def initialize_camera():
    # set camera to photo mode
    _send_command(BASE_COMMAND_URL.format(param1='camera', param2='CM', pwd=PASSWORD, opt="01"))
    # set resolution to 5pmW
    _send_command(BASE_COMMAND_URL.format(param1='camera', param2='CM', pwd=PASSWORD, opt="01"))
    # set protune to off
    _send_command(BASE_COMMAND_URL.format(param1='camera', param2='PT', pwd=PASSWORD, opt="01"))

def capture():
    return _send_command(BASE_COMMAND_URL.format(param1='bacpac', param2='SH', pwd=PASSWORD, opt="01"))


