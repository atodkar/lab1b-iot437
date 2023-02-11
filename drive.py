#
# This is the main file which contains the logic of self driving car
#
import threading
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
from enum import Enum
from createmap import *
from detect_image import detect_image
from astar import *
import time
import numpy as np
from PIL import Image
import cv2

class Direction(Enum):
    Forward = 1
    Backward = 2
    Left = 3
    Right = 4

class Face(Enum):
    North = 1
    South = 2
    East = 3
    West = 4    

#
# This function drives the car in given direction one step at a time
#
def drive(dir: Direction):
    if dir == Direction.Forward:
        fc.forward(SPEED); time.sleep(1.05)
    if dir == Direction.Backward:
        fc.backward(SPEED); time.sleep(1)
    if dir == Direction.Left:
        fc.turn_left(SPEED); time.sleep(1.9); fc.forward(SPEED); time.sleep(1.1)
    if dir == Direction.Right:
        fc.turn_right(SPEED); time.sleep(1.5) ; fc.forward(SPEED); time.sleep(1.1)
    fc.stop()

def tostr(elem):
    if elem == 0:
        return " "
    return "1"

keep_scanning = True
object_detected = False

## Start and end goal
start, goal = (3, 1), (0, 9)
SPEED = 5

current_mapping = np.zeros((100, 100), dtype=int)

#
# This function was prepared to show multithreaded processing and not to halt the car after every step
# This implementation is not perfect so not used.
#
def continuous_mapping():
    global object_detected
    while keep_scanning:
        total = 0
        current_mapping = get_mapping()
        for i in current_mapping[7:18]:
            total += sum(i[40:60])
        object_detected = total >= 5
        print("Total ", total, ", Object detected ", object_detected)

def objectdetected():
    #global current_mapping
    return object_detected

#
# function to draw simple text based mapping
#
def draw_mapping(mapping):
    cnt = 1
    print(''.join([str(elem) for elem in range(99)]))
    for y in mapping:
        listToStr = ''.join([tostr(elem) for elem in y])
        cnt += 1
        print(cnt, listToStr)

#
# function to draw maping in the form of image
#
def draw_image(mapping):
    mapping[mapping == 1] = 255
    mapping[mapping == 0] = 50
    img = Image.fromarray(np.uint8(mapping))
    img.save("current_map.jpg")

entire_field = GridWithWeights(5, 10)
entire_field.walls = []

### Below code shows the trials doe to have a mapping as a continous operation and see if the car can be constently moved.
###
#t1 = threading.Thread(target=continuous_mapping, args=())
#t1.start()

### These parameters decide face and position of car in the beginning
current = start
face = Face.East
while(current != goal):

    total = 0
    current_mapping = get_mapping()

    for i in current_mapping[10:30]:
        total += sum(i[40:60])
    object_detected = total >= 7
    print("Total ", total, ", Object detected ", object_detected)
    draw_image(current_mapping)
    found_person = False

    if object_detected:
        # object detection and update wall
        print("detected obstacle while at ", current, " path needs to be changed.")

        detection_result = detect_image("efficientdet_lite0.tflite", 0, 640, 480, 4, False)
        print(detection_result)
        for det in detection_result.detections:
            for cat in det.categories:
                if cat.category_name == 'person':
                    print("There is a person in front of car, wait till he/she moves.")
                    found_person = True
                    break

        if found_person:
            time.sleep(1)
            continue
        else:
            # This is a stop sign, add that as a wall in algorithm
            if face == Face.North:
                entire_field.walls.append((current[0], current[1] + 1))
            elif face == Face.South:
                entire_field.walls.append((current[0], current[1] - 1))
            elif face == Face.East:
                entire_field.walls.append((current[0] - 1, current[1]))
            else:
                entire_field.walls.append((current[0] + 1, current))

    came_from, cost_so_far = a_star_search(entire_field, current, goal)

    path=reconstruct_path(came_from, start=current, goal=goal)
    
    print(path)

    # change the face as per requirement
    (x1, y1) = current
    (x2, y2) = path[1]
    if x2 == x1 + 1: 
        if face == Face.North:
            face = Face.West
            drive(Direction.Left)
        elif face == Face.South:
            face = Face.West
            drive(Direction.Right)
        elif face == Face.East:
            drive(Direction.Backward)
        else:
            drive(Direction.Forward)

    if x2 == x1 - 1:
        if face == Face.North:
            face = Face.East
            drive(Direction.Right)            
        elif face == Face.South:
            face = Face.East
            drive(Direction.Left)
        elif face == Face.East:
            drive(Direction.Forward)
        else:
            drive(Direction.Backward)

    if y2 == y1 + 1:
        if face == Face.North:
            drive(Direction.Forward)
        elif face == Face.South:
            drive(Direction.Backward)
        elif face == Face.East:
            face = Face.North
            drive(Direction.Left)
        else:
            face = Face.North
            drive(Direction.Right)

    if y2 == y1 - 1:
        if face == Face.North:
            drive(Direction.Backward)
        elif face == Face.South:
            drive(Direction.Forward)
        elif face == Face.East:
            face = Face.South
            drive(Direction.Right)
        else:
            face = Face.South
            drive(Direction.Left)

    current = path[1]

keep_scanning = False

#t1.join()