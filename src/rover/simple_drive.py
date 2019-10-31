#!/usr/bin/python3

from .rover_lib import Motors
import rospy
import struct

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

def main():
	rospy.init_node("simple_drive")
	motors = Motors()
	
	def on_new_twist(data):
		print("Twist.  Forward:", data.linear.x, ", Rotate:", data.angular.z)
		motors.allGentleThrottle(data.linear.x*-2)
		
		# data.linear.x, data.angular.z)

	def on_new_servo(data):
		print("New servo!", data.data)
		# data.data

	subscriber_twist = rospy.Subscriber("cmd_vel", Twist, on_new_twist, queue_size=10)
	subscriber_servo = rospy.Subscriber("servo_pos", Float32, on_new_servo, queue_size=10)

	rospy.spin()
