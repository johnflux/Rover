#!/usr/bin/python3

import traceback
from .rover_lib import Motors,Servos
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from rover.msg import Motor, Servo
from rover.srv import PowerOff
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty
import os
from . import serial_write

def main():
    rospy.init_node("simple_drive")
    motors = Motors()
    servos = Servos()
    rate = rospy.Rate(10) # 10hz

    quadcopter_cover = 0 # Default to closed
    servos.quadcopterCoverClose()

    def on_new_twist_body(data):
        try:
            #print("Twist.  Forward:", data.linear.x, ", Rotate:", round(data.angular.z * 45))
            if data.angular.z == 0 and data.linear.x == 0:
                #print("Movement stop")
                motors.allGentleThrottle(data.linear.x)
                servos.allOff()
            elif data.angular.z == 0 or data.linear.x != 0:
                motors.allGentleThrottle(data.linear.x)
                print("Servos on")
                servos.setTwist(0)
            else:
                servos.twist()
                motors.twistThrottle(data.angular.z/6)

        except Exception as e:
            rospy.logerr(traceback.format_exc())
            motors.allOff()
            servos.allOff()

    def on_new_twist_arm(arm_middle, arm_bottom, hand_leftright, hand_updown):
        try:
            #print("arm updown:", data.linear.y, ", leftright:", data.linear.z)
            motors.armMiddleThrottle(arm_middle)
            motors.armBottomThrottle(arm_bottom)
            if hand_leftright != 0:
                serial_write.arm_led_on()
            else:
                serial_write.arm_led_off()
            servos.handLeftRight(hand_leftright)
            servos.handUpDown(hand_updown)
        except Exception as e:
            rospy.logerr(traceback.format_exc())
            motors.allOff()
            servos.allOff()

    def on_new_raw_motor(data):
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

    def on_new_raw_servo(data):
        """ This command is mostly for debugging.  So that the user can directly manipulate a servo, to
            test that it is working, mapped correctly etc """
        servos.setServo('right_front', data.right_front)
        servos.setServo('right_back', data.right_back)
        servos.setServo('left_front', data.left_front)
        servos.setServo('left_back', data.left_back)
        servos.setServo('hand_updown', data.hand_updown)
        servos.setServo('hand_leftright', data.hand_leftright)
        servos.setServo('quadcopter_cover', data.quadcopter_cover)


    def on_new_quadcopter_cover(data):
        if data.data == 1.0:
            servos.quadcopterCoverOpen()
        else:
            servos.quadcopterCoverClose()
        print("Quadcopter cover:", data.data)

    def on_new_joy(data):
        global quadcopter_cover
        # Buttons:
        #   A=0
        #   B=1  - Increase speed
        #   Y=3
        #   X=2  - quadcopter cover open
        # d-pad up-down=7  and left-right=6
        speed_setting = 2
        if data.buttons[1]:
            speed_setting = 1

        if data.buttons[8]:
            rospy.loginfo("Joystick power button pressed - shutting down")
            try:
                rospy.ServiceProxy('power_off/power_off', PowerOff)()
            except rospy.ServiceException as e:
                rospy.logerr("Failed to poweroff:" + traceback.format_exc())
            return

        # Drive sticks
        linear_vel = data.axes[4] / speed_setting # right stick forward/back
        angular_vel = -data.axes[3] # right stick left/right (rad/s)

        arm_middle = data.axes[0] / speed_setting # horiz
        arm_bottom = data.axes[1] / speed_setting # vert

        hand_leftright = data.axes[6] * 5
        hand_updown = data.axes[7] * 5

        # Publish Twist
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        if abs(twist.angular.z) > abs(twist.linear.x):
            twist.linear.x = 0
        on_new_twist_body(twist)
        #cmd_vel_pub.publish(twist)

        on_new_twist_arm(arm_middle, arm_bottom, hand_leftright, hand_updown)

        # Quadcopter servo
        if data.buttons[2]: # Toggle quadcopter cover
            quadcopter_cover = not quadcopter_cover
            quadcopter_cover_value = 1.0 if quadcopter_cover else 0.0
            #quadcopter_cover_pub.publish( 1.0 if quadcopter_cover else 0.0 )
            on_new_quadcopter_cover(quadcopter_cover_value)
        if data.buttons[0]:
            on_display_ip_on_leds()

    def on_display_ip_on_leds():
        os.system("~/rover/src/rover/serial_write.py")
        rospy.loginfo("IP Address is " + serial_write.get_ip())

    subscriber_twist = rospy.Subscriber("cmd_vel", Twist, on_new_twist_body, queue_size=10)
    subscriber_servo = rospy.Subscriber("quadcopter_cover", Float32, on_new_quadcopter_cover, queue_size=10)
    subscriber_joystock = rospy.Subscriber("joy", Joy, on_new_joy, queue_size=1)

    subscriber_motor_cmd = rospy.Subscriber("motor_cmd", Motor, on_new_raw_motor, queue_size=1)
    subscriber_servo_cmd = rospy.Subscriber("servo_cmd", Servo, on_new_raw_servo, queue_size=1)

    service_displayip = rospy.Service('display_ip', Empty, on_display_ip_on_leds)

    on_display_ip_on_leds()

    while not rospy.is_shutdown():
        servos.update()
        rate.sleep()
