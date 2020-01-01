#!/usr/bin/env python3 
import rover_lib
import time
import sys, getopt

servos = rover_lib.Servos()

servos.setTwist(0)
time.sleep(2)
servos.allOff()
