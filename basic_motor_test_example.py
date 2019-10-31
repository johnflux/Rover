#!/usr/bin/env python3 
import time
from adafruit_motorkit import MotorKit
import json

# See MotorKit code for full api of MotorKit (but there's not much to it:
# https://github.com/adafruit/Adafruit_CircuitPython_MotorKit/blob/master/adafruit_motorkit.py
# And for the Motor:
# https://github.com/adafruit/Adafruit_CircuitPython_Motor/blob/master/adafruit_motor/motor.py 

class Motor:
	def __init__(self):
		self.dcmotor = None
		self.wired_backwards = False

	@property
	def throttle(self):
		return self.dcmotor.throttle

	@throttle.setter
	def throttle(self, value):
		print("Setting throttle to ", value)
		if self.wired_backwards:
			value = -value
		self.dcmotor.throttle = value

class Motors:
	def __init__(self):
		# We have the equivalent of self.left_front = Motor() etc
		self.motor_names = ['left_front', 'left_middle', 'left_back', 'right_front', 'right_middle', 'right_back']
		for motor_name in self.motor_names:
			setattr(self, motor_name, Motor())

		self.reload_conf()
		self.currentAllThrottle = 0.0  #TODO - make this per motor.  This is to try to smooth out the motors

	def reload_conf(self):

		with open('motor_configuration.json') as json_file:
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
			print("Configured", motor_name, "on", motor_conf['controller_motor_number'], 'and it', 'is' if motor_conf['wired_backwards'] else 'is not', 'wired backwards')

	# Only real use is to set throttle to 0 for all, or for debugging
	def allThrottle(self, throttle):
		self.currentAllThrottle = throttle
		for motor_name in self.motor_names:
			getattr(self, motor_name).throttle = throttle

	def allGentleThrottle(self, throttle):
		step = 1 if throttle > self.currentAllThrottle else -1
		for t in range(round(self.currentAllThrottle*10),  round(throttle*10), step):
			print(t/10.0)
			self.allThrottle(t/10.0)
			time.sleep(0.1)
		self.allThrottle(throttle)
	
motors = Motors()
motors.allGentleThrottle(1)
time.sleep(3)
motors.allGentleThrottle(0)
