#
# Testing and trial files. Not part of actual execution. 
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
import picar_4wd as fc

import time

SPEED = 5

# cnt = 0
# while(cnt < 5):
# #fc.forward(10); time.sleep(1)
#     fc.forward(SPEED); time.sleep(1)
#     cnt += 1
# #fc.backward(SPEED); time.sleep(1) 
# #fc.turn_left(10); time.sleep(1.5); fc.forward(10); time.sleep(1); fc.backward(10); time.sleep(1) 
# #fc.turn_right(10); time.sleep(1.5) ; fc.forward(10); time.sleep(1); fc.backward(10); time.sleep(1) 
fc.stop()


import asyncio

async def main():
    print('Hello ...')
    await asyncio.sleep(1)
    print('... World!')

asyncio.run(main())
print('out')