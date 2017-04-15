"""
GoPro WiFi API for sending /receiving commands
"""

import urllib2
import json
import numpy as np
from contextlib import closing

# control URLs
STATUS_URL = "http://10.5.5.9/gp/gpControl/status"
MEDIA_LS_URL = "http://10.5.5.9/gp/gpMediaList"
TRIGGER_URL = "http://10.5.5.9/gp/gpControl/command/shutter?p=1"

# initial settings urls
INIT_SETTINGS_KEYS = {54: 1  # set quick capture to 1
                      }

# status parsing constants
BATTERY_STATUS_MAP = {"3": "FULL",
                      "2": "HALFWAY",
                      "1": "LOW"}

OUTPUT_MODE_MAP = {"0": "VIDEO",
                   "1": "PHOTO",
                   "2": "MULTISHOT"}

PHOTO_MEGAPIXEL_MAP = {"0": "12MP_WIDE",
                       "1": "7MP_WIDE",
                       "2": "7PM_MED",
                       "3": "5PM_MED"}

SHUTTER_SPEED_MAP = {"0": 0,
                     "1": 2,
                     "2": 5,
                     "3": 10,
                     "4": 15,
                     "5": 20,
                     "6": 30}

SHARPNESS_MAP = {"0": "HIGH",
                 "1": "MEDIUM",
                 "2": "LOW"}

def _send_command(target_url):
    with closing(urllib2.urlopen(target_url)) as f:
        return json.loads(f.read())

def get_camera_status():
    status = _send_command(STATUS_URL)

    # parse status flags
    battery = BATTERY_STATUS_MAP[status['status']['2']]
    output_mode = OUTPUT_MODE_MAP[status['status']['43']]
    remaining_photos = int(status['status']['34'])

    # parse settings flags
    photo_mp = PHOTO_MEGAPIXEL_MAP[status['settings']['17']]
    shutter_speed = SHUTTER_SPEED_MAP[status['settings']['19']]
    sharpness = SHARPNESS_MAP[status['settings']['25']]

    return {"Battery": battery,
            "OutputMode": output_mode,
            "RemainingPhotos": remaining_photos,
            "PhotoResolution": photo_mp,
            "ShutterSpeed": shutter_speed,
            "Sharpness": sharpness}
    

def get_media_list():
    return _send_command(MEDIA_LS_URL)

def capture():
    return _send_command(TRIGGER_URL)

