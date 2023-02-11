#
# This file contains logic to get the ultrasonic readings and prepare mapping 
#
from picar_4wd.pwm import PWM
from picar_4wd.adc import ADC
from picar_4wd.pin import Pin
from picar_4wd.motor import Motor
from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic 
from picar_4wd.speed import Speed
from picar_4wd.filedb import FileDB  
from picar_4wd.utils import *
import time
import numpy as np
import math
import matplotlib.pyplot as plt
from PIL import Image


ANGLE_RANGE = 100
STEP = 8
us_step = STEP
angle_distance = [0,0]
current_angle = 0
max_angle = ANGLE_RANGE/2
min_angle = -ANGLE_RANGE/2

# Init Ultrasonic
us = Ultrasonic(Pin('D8'), Pin('D9'))
servo = Servo(PWM("P0"), offset=0)

#
# get distance from ultrasound
#
def get_distance():
    global current_angle, us_step
    current_angle += us_step
    if current_angle >= max_angle:
        current_angle = max_angle
        us_step = -STEP
    elif current_angle <= min_angle:
        current_angle = min_angle
        us_step = STEP
    
    servo.set_angle(current_angle)
    time.sleep(0.04)
    
    dist = us.get_distance()

    return current_angle, dist

##
def interpolate(mappingarr):
    # for y, indy in mappingarr:
    #     for x, indx in y:
    #         if x == 1:
    #             mappingarr[indy]
    return mappingarr

#
# this function calculates the ultrasound obstacles and prepares numpy array
#
def get_mapping():
    carposition = (50, 0)
    mappingarr = np.zeros((100, 100), dtype=int)

    touched_both = 0
    while touched_both != 2:

        current_angle, dist = get_distance()

        touched_both += current_angle == ANGLE_RANGE / 2 or current_angle == -ANGLE_RANGE / 2

        x = dist * math.sin(math.radians(current_angle))
        y = dist * math.cos(math.radians(current_angle))

        x = int(carposition[0] + x)
        y = int(carposition[1] + y)

        x = 0 if x < 0 else x
        x = 99 if x >= 100 else x
        y = 0 if y < 0 else y
        y = 99 if y >= 100 else y
        #print("angle ", current_angle, " at distance: ", dist, " ", x, " ", y)
        
        mappingarr[y][x] = 1
    return interpolate(mappingarr)

import numpy as np
import numpy as np
from numpy.lib.stride_tricks import as_strided



def main():
    mapping = get_mapping()

    mapping[mapping == 1] = 255
    mapping[mapping == 0] = 50
    img = Image.fromarray(np.uint8(mapping))
    img.save("current_map.jpg")    


if __name__ == '__main__':
  main()
