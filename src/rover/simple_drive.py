#!/usr/bin/python3

import traceback
from .rover_lib import Motors,Servos
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from rover.msg import Motor, Servo

def main():
	rospy.init_node("simple_drive")
	motors = Motors()
	servos = Servos()
	rate = rospy.Rate(10) # 10hz

	def on_new_twist(data):
		try:
			print("Twist.  Forward:", data.linear.x, ", Rotate:", round(data.angular.z * 45))
			print("arm updown:", data.linear.y, ", leftright:", data.linear.z)
			motors.armMiddleThrottle(data.linear.y/5)
			motors.armBottomThrottle(data.linear.z/5)
			if abs(data.angular.z) > abs(data.linear.x):
				data.linear.x = 0
			servos.handLeftRight(data.angular.x*5)
			servos.handUpDown(data.angular.y*5)

			if data.angular.z == 0 and data.linear.x == 0:
				print("Servos off")
				motors.allGentleThrottle(data.linear.x)
				servos.allOff()
			elif data.angular.z == 0 or data.linear.x != 0:
				#motors.allGentleThrottle(data.linear.x)
				print("Servos on")
				servos.setTwist(0)
			else:
				servos.twist()
				motors.twistThrottle(data.angular.z/6)

		except Exception as e:
			rospy.logerr(traceback.format_exc())
			motors.allOff()
			servos.allOff()

		# data.linear.x, data.angular.z)

	def on_new_motor_cmd(data):
		""" This command is mostly for debugging.  So that the user can directly manipulate a motor, to
		    test that it is working, mapped correctly etc """
		motors.setMotorThrottle('left_front', data.left_front / 100.0)
		motors.setMotorThrottle('left_middle', data.left_middle / 100.0)
		motors.setMotorThrottle('left_back', data.left_back / 100.0)
		motors.setMotorThrottle('right_front', data.right_front / 100.0)
		motors.setMotorThrottle('right_middle', data.right_middle / 100.0)
		motors.setMotorThrottle('right_back', data.right_back / 100.0)
		motors.setMotorThrottle('arm_middle', data.arm_middle / 100.0)
		motors.setMotorThrottle('arm_bottom', data.arm_bottom / 100.0)

	def on_new_servo_cmd(data):
		servos.setServo('right_front', data.right_front)
		servos.setServo('right_back', data.right_back)
		servos.setServo('left_front', data.left_front)
		servos.setServo('left_back', data.left_back)
		servos.setServo('arm_updown', data.arm_updown)
		servos.setServo('arm_leftright', data.arm_leftright)

	def on_new_servo(data):
		if data.data == 1.0:
			servos.quadcopterCoverOpen()
		else:
			servos.quadcopterCoverClose()
		print("Quadcopter cover:", data.data)

	subscriber_twist = rospy.Subscriber("cmd_vel", Twist, on_new_twist, queue_size=10)
	subscriber_servo = rospy.Subscriber("quadcopter_cover", Float32, on_new_servo, queue_size=10)

	subscriber_motor_cmd = rospy.Subscriber("motor_cmd", Motor, on_new_motor_cmd, queue_size=1)
	subscriber_servo_cmd = rospy.Subscriber("servo_cmd", Servo, on_new_servo_cmd, queue_size=1)

	while not rospy.is_shutdown():
		servos.update()
		rate.sleep()
