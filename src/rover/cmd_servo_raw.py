#!/usr/bin/env python3 
import rover_lib
import time
import sys, getopt

servos = rover_lib.Servos()

if len(sys.argv) != 3:
	print("./servo servo_number angle")
	print("  Where servo_number is between 0 and 15")
	exit(1)

try:
	servo_number = int(sys.argv[1])
	amount = int(sys.argv[2])
	if servo_number < 0 or servo_number > 15:
		print("Servo number must be between 0 and 15")
		exit(1)

except:
	print("Invalid arguments")
	exit(1)

print(servo_number, " is currently at: ", servos.servo.servo[servo_number].angle)
servos.servo.servo[servo_number].angle = amount
print(servo_number, " is now at: ", servos.servo.servo[servo_number].angle)
time.sleep(2)
servos.servo.servo[servo_number].angle = None

print(servo_number, " is reset to: ", servos.servo.servo[servo_number].angle)
