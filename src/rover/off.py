#!/usr/bin/env python3
import rover_lib
import time

motors = rover_lib.Motors()
servos = rover_lib.Servos()
motors.allOff()
servos.allOff()
servos.armOff()
