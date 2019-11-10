#!/usr/bin/env python3 

import sys
if (sys.version_info < (3, 0)):
	print("Sorry, python 2 is not supported")
	exit(1)


import time
from adafruit_motorkit import MotorKit
from adafruit_servokit import ServoKit
import json

# See MotorKit code for full api of MotorKit (but there's not much to it:
# https://github.com/adafruit/Adafruit_CircuitPython_MotorKit/blob/master/adafruit_motorkit.py
# And for the Motor:
# https://github.com/adafruit/Adafruit_CircuitPython_Motor/blob/master/adafruit_motor/motor.py 

#TODO: Use  rospy.get_param('somekey', default)

class Motor:
	def __init__(self):
		self.dcmotor = None
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
		self.dcmotor.throttle = throttle

class Motors:
	def __init__(self):
		# We have the equivalent of self.left_front = Motor() etc
		self.motor_names = ['left_front', 'left_middle', 'left_back', 'right_front', 'right_middle', 'right_back', 'arm_middle']
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
			motor.arn = motor_conf['arm']
			print("Configured", motor_name, "on", motor_conf['controller_motor_number'], 'and it', 'is' if motor_conf['wired_backwards'] else 'is not', 'wired backwards')

	# Only real use is to set throttle to 0 for all, or for debugging
	def allThrottle(self, throttle):
		self.currentAllThrottle = throttle
		for motor_name in self.motor_names:
			motor = getattr(self, motor_name)
			if not motor.arm:
				motor.throttle = throttle

	def armThrottle(self, throttle):
			self.arm_middle.throttle = throttle

	def allGentleThrottle(self, throttle):
		step = 1 if throttle > self.currentAllThrottle else -1
		for t in range(round(self.currentAllThrottle*10),  round(throttle*10), step):
			self.allThrottle(t/10.0)
			time.sleep(0.01)
		self.allThrottle(throttle)

	def twistThrottle(self, throttle):
		print("Twist throttle to", throttle)
		self.left_front.throttle = throttle
		self.left_middle.throttle = throttle/2
		self.left_back.throttle = throttle
		self.right_front.throttle = -throttle
		self.right_middle.throttle = -throttle/2
		self.right_back.throttle = -throttle

class Servos():
	def __init__(self):
		self.servo = ServoKit(channels=16)
		self.right_front = self.servo.servo[15]
		self.right_back = self.servo.servo[14]
		self.left_front = self.servo.servo[1]
		self.left_back = self.servo.servo[0]
		self.arm_updown = self.servo.servo[10]
		self.arm_leftright = self.servo.servo[11]

		self.max_arm_leftright = 180
		self.min_arm_leftright = 60
		self.max_arm_updown = 160
		self.min_arm_updown = 85

		self.rate_arm_updown = None
		self.rate_arm_leftright = None

	def allOff(self):
		self.right_front.angle=None
		self.right_back.angle=None
		self.left_front.angle=None
		self.left_back.angle=None
		#self.arm_updown.angle=None
		#self.arm_leftright.angle=None
		pass

	def setTwist(self, angle):
		self.right_front.angle=30+angle
		self.right_back.angle=110+angle
		self.left_front.angle=90+angle
		self.left_back.angle=130-angle
	def twist(self):
		self.right_front.angle=30 - 30
		self.right_back.angle=110 + 40
		self.left_front.angle=90 + 35
		self.left_back.angle=130 - 40

	def armAllTheWayUp(self):
		self.arm_updown.angle = self.min_arm_updown
	def armStraight(self):
		self.arm_updown.angle = 140
	def armAllTheWayDown(self):
		self.arm_updown.angle = self.max_arm_updown
	def armHorizontalCenter(self):
		self.arm_leftright.angle = self.min_arm_leftright
	def armReset(self):
		self.armAllTheWayUp()
		self.arm_leftright.angle = self.max_arm_leftright
	
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
