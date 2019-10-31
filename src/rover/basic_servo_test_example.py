#!/usr/bin/env python3 
import time
from adafruit_motorkit import MotorKit
from adafruit_servokit import ServoKit

servo = ServoKit(channels=16)
kit = MotorKit(0x61)

servo.servo[15].angle=90
servo.servo[14].angle=90
servo.servo[0].angle=90
servo.servo[1].angle=90

time.sleep(3)

servo.servo[15].angle=50
servo.servo[14].angle=50
servo.servo[0].angle=50
servo.servo[1].angle=50
