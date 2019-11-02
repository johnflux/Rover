#!/usr/bin/env python3 
import rover_lib
import time

servos = rover_lib.Servos()
servos.twist()
time.sleep(3)
servos.noTwist()
