#!/usr/bin/env python3
import rover_lib
import time

motors = rover_lib.Motors()
motors.allGentleThrottle(1)
time.sleep(3)
motors.allGentleThrottle(0)
