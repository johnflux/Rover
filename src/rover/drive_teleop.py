#!/usr/bin/python

import rospy
import subprocess

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

class DriveTeleop:
    def __init__(self):
        self.speed_setting = 1 # default to full speed
        self.quadcopter_cover = 0 # Default to closed

        self.cmd_vel_pub = rospy.Publisher("teleop/cmd_vel", Twist, queue_size=1)
        self.quadcopter_cover_pub = rospy.Publisher("quadcopter_cover", Float32, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy)

    def on_joy(self, data):
        # Buttons:
        #   A=0
        #   B=1  - Increase speed
        #   Y=3
        #   X=2  - quadcopter cover open
        # d-pad up-down=7  and left-right=6
        self.speed_setting = 2
        if data.buttons[1]:
            self.speed_setting = 1

        # Drive sticks
        linear_vel = data.axes[4] / self.speed_setting # right stick forward/back
        angular_vel = -data.axes[3] # right stick left/right (rad/s)

        arm_leftright = data.axes[0] / self.speed_setting # horiz
        arm_updown = data.axes[1] / self.speed_setting # vert

        hand_leftright = data.axes[6]
        hand_updown = data.axes[7]

        # Publish Twist
        twist = Twist()
        twist.linear.x = linear_vel
        twist.linear.y = arm_updown
        twist.linear.z = arm_leftright
        twist.angular.z = angular_vel
        twist.angular.x = hand_leftright
        twist.angular.y = hand_updown

        self.cmd_vel_pub.publish(twist)

        # Quadcopter servo
        if data.buttons[2]: # Toggle quadcopter cover
            self.quadcopter_cover = not self.quadcopter_cover
            self.quadcopter_cover_pub.publish( 1.0 if self.quadcopter_cover else 0.0 )

def main():
    rospy.init_node("drive_teleop")
    controller = DriveTeleop()
    rospy.spin()
