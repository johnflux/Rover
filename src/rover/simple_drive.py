#!/usr/bin/python3

import traceback
from .rover_lib import Motors,Servos
import rospy
import struct

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

def main():
	rospy.init_node("simple_drive")
	motors = Motors()
	servos = Servos()
	rate = rospy.Rate(10) # 10hz
	
	def on_new_twist(data):
		try:
			print("Twist.  Forward:", data.linear.x, ", Rotate:", round(data.angular.z * 45))
			print("arm updown:", data.linear.y, ", leftright:", data.linear.z)
			motors.allGentleThrottle(data.linear.x)
			motors.armMiddleThrottle(data.linear.y)
			motors.armBottomThrottle(data.linear.z)
			#if data.linear.y == -2:
			#	servos.moveArmUp(None)
			#else:
			#	servos.moveArmUp(-data.linear.y*10)
			#if data.linear.z == -2:
			#	servos.moveArmLeft(None)#
			#else:
			#	servos.moveArmLeft(data.linear.z*10)

			if data.angular.z == 0 and data.linear.x == 0:
				print("Servos off")
				servos.allOff()
			elif data.angular.z == 0 or data.linear.x != 0:
				print("Servos on")
				servos.setTwist(0)
			else:
				servos.twist()
				motors.twistThrottle(data.angular.z)
		except Exception as e:
			print(traceback.format_exc())
			motors.allOff()
			servos.allOff()
		
		# data.linear.x, data.angular.z)

	def on_new_servo(data):
		print("New servo!", data.data)
		# data.data

	subscriber_twist = rospy.Subscriber("cmd_vel", Twist, on_new_twist, queue_size=10)
	subscriber_servo = rospy.Subscriber("servo_pos", Float32, on_new_servo, queue_size=10)

	while not rospy.is_shutdown():
		servos.update()
		rate.sleep()
