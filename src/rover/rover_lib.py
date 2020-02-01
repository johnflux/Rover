#!/usr/bin/env python3

import sys
if (sys.version_info < (3, 0)):
	print("Sorry, python 2 is not supported")
	exit(1)


import time
from adafruit_motorkit import MotorKit
from adafruit_servokit import ServoKit
import json
from typing import Optional

# See MotorKit code for full api of MotorKit (but there's not much to it:
# https://github.com/adafruit/Adafruit_CircuitPython_MotorKit/blob/master/adafruit_motorkit.py
# And for the Motor:
# https://github.com/adafruit/Adafruit_CircuitPython_Motor/blob/master/adafruit_motor/motor.py

#TODO: Use  rospy.get_param('somekey', default)

class Motor:
	def __init__(self):
		self.dcmotor = None # type: Optional[MotorKit]
		self.wired_backwards = False
		self.arm = False

	@property
	def throttle(self):
		return self.dcmotor.throttle

	@throttle.setter
	def throttle(self, throttle):
		if throttle != None and throttle != 0:
			if throttle > 0:
				throttle = throttle / 2 + 0.5
			else:
				throttle = throttle / 2 - 0.5
		if self.wired_backwards and throttle != None:
			throttle = -throttle
		if throttle != None:
			throttle = min(1, max(-1, throttle))
		self.dcmotor.throttle = throttle

class Motors:
	left_front = None  # type: Optional[Motor]
	left_middle = None  # type: Optional[Motor]
	left_back = None  # type: Optional[Motor]
	right_front = None  # type: Optional[Motor]
	right_middle = None  # type: Optional[Motor]
	right_back = None  # type: Optional[Motor]
	arm_middle = None  # type: Optional[Motor]
	arm_bottom = None  # type: Optional[Motor]

	def __init__(self):
		# We have the equivalent of self.left_front = Motor() etc
		self.motor_names = ['left_front', 'left_middle', 'left_back', 'right_front', 'right_middle', 'right_back', 'arm_middle', 'arm_bottom']
		for motor_name in self.motor_names:
			setattr(self, motor_name, Motor())

		self.reload_conf()
		self.currentAllThrottle = 0.0  #TODO - make this per motor.  This is to try to smooth out the motors

	def reload_conf(self):

		with open('/home/ubuntu/catkin_ws/src/rover/src/rover/motor_configuration.json') as json_file:
			self.conf = json.load(json_file)
		print(self.conf)

		self.kitRight = MotorKit(int(self.conf["right_controller_board"]["i2c_address"], 16))
		self.kitLeft = MotorKit(int(self.conf["left_controller_board"]["i2c_address"], 16))

		for motor_name in self.motor_names:
			motor_conf = self.conf[motor_name]
			motor = getattr(self, motor_name)
			kit = self.kitLeft if motor_conf['left_controller'] else self.kitRight
			motor.dcmotor = getattr(kit, 'motor' + str(motor_conf['controller_motor_number']))
			motor.wired_backwards = motor_conf['wired_backwards']
			motor.arm = motor_conf['arm']
			print("Configured", motor_name, "on", motor_conf['controller_motor_number'], 'and it', 'is' if motor_conf['wired_backwards'] else 'is not', 'wired backwards')

	def setMotorThrottle(self, motor_name, throttle):
		""" Set the given motor to the given value (between -100 to 100) divided by 100 (To make it between -1 and 1).  Only useful for debugging """
		motor = getattr(self, motor_name)
		if throttle == 0:
			motor.throttle = None
		else:
			motor.throttle = throttle

	# Only real use is to set throttle to 0 for all, or for debugging
	def allThrottle(self, throttle):
		self.currentAllThrottle = throttle
		if self.currentAllThrottle == None:
			self.currentAllThrottle = 0
		for motor_name in self.motor_names:
			motor = getattr(self, motor_name)
			if not motor.arm:
				motor.throttle = throttle
	def allOff(self):
		self.currentAllThrottle = 0
		for motor_name in self.motor_names:
			motor = getattr(self, motor_name)
			motor.throttle = None

	def armMiddleThrottle(self, throttle):
			self.arm_middle.throttle = throttle

	def armBottomThrottle(self, throttle):
			self.arm_bottom.throttle = throttle

	def allGentleThrottle(self, throttle):
		if self.currentAllThrottle == None:
			self.currentAllThrottle = 0
		step = 1 if throttle > self.currentAllThrottle else -1
		for t in range(round(self.currentAllThrottle*10),  round(throttle*10), step):
			self.allThrottle(t/10.0)
			time.sleep(0.01)
		self.allThrottle(throttle)

	def twistThrottle(self, throttle):
		self.left_front.throttle = throttle
		self.left_middle.throttle = throttle/2
		self.left_back.throttle = throttle
		self.right_front.throttle = -throttle
		self.right_middle.throttle = -throttle/2
		self.right_back.throttle = -throttle

class Servo:
	dcmotor = None # type: MotorKit
	servomotor = None # type: ServoKit

	def __init__(self):
		self.servomotor = None
		self.backwards = False
		self.zero_offset = 0
		self.max = 180
		self.min = 0

	@property
	def angle(self):
		return self.dcmotor.angle

	@angle.setter
	def angle(self, angle):
		if angle != None:
			if angle > self.max:
				angle = self.max
			if angle < self.min:
				angle = self.min
		self.servomotor.angle = angle

	@property
	def offset_angle(self):
		if self.dcmotor.angle == None:
			return 0
		if not self.backwards:
			return self.dcmotor.angle - self.zero_offset
		else:
			return -(self.dcmotor.angle - self.zero_offset)

	@offset_angle.setter
	def offset_angle(self, offset):
		if offset == None:
			self.servomotor.angle = None
		else:
			if self.backwards:
				offset = -offset
			self.angle = self.zero_offset + offset

class Servos():
	right_front = None # type: Servo
	right_back = None # type: Servo
	left_front = None # type: Servo
	left_back = None # type: Servo
	arm_updown = None # type: Servo
	arm_leftright = None # type: Servo

	def __init__(self):
		self.servo = ServoKit(channels=16)

		self.servo_names = [ 'right_front', 'right_back', 'left_front', 'left_back', 'arm_updown', 'arm_leftright' ]
		for servo_name in self.servo_names:
			setattr(self, servo_name, Servo())

		self.right_front.servomotor = self.servo.servo[14]
		self.right_back.servomotor = self.servo.servo[15]
		self.left_front.servomotor = self.servo.servo[13]
		self.left_back.servomotor = self.servo.servo[12]
		self.arm_updown.servomotor = self.servo.servo[0]
		self.arm_leftright.servomotor = self.servo.servo[1]

		self.right_front.zero_offset = 90
		self.right_back.zero_offset = 110
		self.left_front.zero_offset = 90
		self.left_back.zero_offset = 130
		self.arm_updown.zero_offset = 140

		self.right_front.backwards=True
		self.left_back.backwards=True

		self.right_front.min = 60
		self.right_front.max = 110
		self.left_front.min = 45
		self.left_front.max = 120
		self.arm_leftright.max = 180
		self.arm_leftright.min = 60
		self.arm_updown.max = 160
		self.arm_updown.min = 85

		self.rate_arm_updown = None
		self.rate_arm_leftright = None

	def allOff(self):
		self.right_front.angle=None
		self.right_back.angle=None
		self.left_front.angle=None
		self.left_back.angle=None

	def setServo(self, servo_name, offset_angle):
		""" Set the given motor to the given offset (typically between about -60 to 60). 0 is a special case that cuts power to the servo """
		servo = getattr(self, servo_name)
		if offset_angle == 0:
			servo.angle = None
		else:
			servo.offset_angle = offset_angle

	def armOff(self):
		self.arm_updown.angle=None
		self.arm_leftright.angle=None
		self.rate_arm_updown = None
		self.rate_arm_leftright = None

	def setTwist(self, offset_angle):
		self.right_front.offset_angle=offset_angle
		self.right_back.offset_angle=offset_angle
		self.left_front.offset_angle=offset_angle
		self.left_back.offset_angle=offset_angle
	def twist(self):
		self.right_front.offset_angle = 30
		self.right_back.offset_angle = 40
		self.left_front.offset_angle = 35
		self.left_back.offset_angle = 40

	def armAllTheWayUp(self):
		self.arm_updown.angle = self.min_arm_updown
	def armStraight(self):
		self.arm_updown.offset_angle = 0
	def armAllTheWayDown(self):
		self.arm_updown.angle = self.max_arm_updown
	def armHorizontalCenter(self):
		self.arm_leftright.angle = self.min_arm_leftright
	def armReset(self):
		self.armAllTheWayUp()
		self.arm_leftright.angle = self.max_arm_leftright
		self.rate_arm_updown = None
		self.rate_arm_leftright = None

	def moveArmUp(self, amount):
		if amount == None:
			self.arm_updown.angle = None
			self.rate_arm_updown = None
		elif self.arm_updown.angle != None:
			self.rate_arm_updown = amount
		else:
			self.rate_arm_updown = 0
			self.armStraight()

	def moveArmLeft(self,amount):
		if amount == None:
			self.arm_leftright.angle = None
			self.rate_arm_leftright = None
		elif self.arm_leftright.angle != None:
			self.rate_arm_leftright = amount
		else:
			self.rate_arm_leftright = 0
			self.armHorizontalCenter()

	def update(self):
		if self.rate_arm_leftright != 0 and self.rate_arm_leftright != None and self.arm_leftright.angle != None:
			self.arm_leftright.angle = min(max(self.arm_leftright.angle + self.rate_arm_leftright, self.min_arm_leftright),self.max_arm_leftright)
			#print("arm leftright is now: ", self.arm_leftright.angle)
		if self.rate_arm_updown != 0 and self.rate_arm_updown != None and self.arm_updown.angle != None:
			self.arm_updown.angle = min(max(self.arm_updown.angle + self.rate_arm_updown, self.min_arm_updown),self.max_arm_updown)
			#print("arm updown is now: ", self.arm_updown.angle)
