![controller](controller.png)
##

This folder should be inside ~/catkin_ws/src
e.g. ~/catkin_ws/src/rover/

1. Building

$ cd ~/catkin_ws
$ catkin_build

To test, doing:
$ rosmsg list | grep rover/

Should show (maybe more):

rover/Motor
rover/Servo

2. Launch ROS nodes:

```
$ roslaunch rover drive_teleop.launch joy_dev:=/dev/input/js0
$ roslaunch rover cmd_vel_mux.launch
$ roslaunch rover simple_drive.launch

OR all-in-one launch:
$ roslaunch rover drive.launch
```

3. Examples:
   To manually make motor front left move forward at 60% speed:

    $ rostopic pub -1 /motor_cmd rover/Motor -- 60 0 0 0 0 0 0 0

    And backwards at 50% speed

    $ rostopic pub -1 /motor_cmd rover/Motor -- -50 0 0 0 0 0 0 0

    To shut off all motors:

    $ rostopic pub -1 /motor_cmd rover/Motor -- 0 0 0 0 0 0 0 0

    (Note that 0 is a special case that is taken to mean to cut power to the motor,
     rather than to hold)

    To move all the servos to 1 degree:

    $ rostopic pub -1 /servo_cmd rover/Servo -- 1 1 1 1 0 0

    And 30 degrees:

    $ rostopic pub -1 /servo_cmd rover/Servo -- 30 30 30 30 0 0

    And turn them all off

    $ rostopic pub -1 /servo_cmd rover/Servo -- 0 0 0 0 0 0