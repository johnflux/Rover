![controller](web/public/controller.png)
##

This folder should be inside `~/catkin_ws/src`
e.g. `~/catkin_ws/src/rover/`

1. Building

        cd ~/catkin_ws
        catkin_make

    To test, do:

        $ rosmsg list | grep rover/

    Should show (maybe more):

        rover/Motor
        rover/Servo

2. Launch ROS nodes:


        ~/rover/launch_gui.sh

    This is somewhat equivalent to:

        roslaunch rover drive.launch

    But lets us also monitor the nodes, restart them, etc.

    To make this launch automatically on startup, make it a systemd service:


        cd ~/rover
        sudo cp rover.service /lib/systemd/system
        sudo systemctl daemon-reload
        sudo systemctl enable rover.service

3. Examples:

   To manually make motor front left move forward at 60% speed:

        rostopic pub -1 /motor_cmd rover/Motor -- 60 0 0 0 0 0 0 0

    And backwards at 50% speed

        rostopic pub -1 /motor_cmd rover/Motor -- -50 0 0 0 0 0 0 0

    To shut off all motors:

        rostopic pub -1 /motor_cmd rover/Motor -- 0 0 0 0 0 0 0 0

    (Note that 0 is a special case that is taken to mean to cut power to the motor, rather than to hold)

    To move all the servos to 1 degree:

        rostopic pub -1 /servo_cmd rover/Servo -- 1 1 1 1 0 0

    And 30 degrees:

        rostopic pub -1 /servo_cmd rover/Servo -- 30 30 30 30 0 0

    And turn them all off

        rostopic pub -1 /servo_cmd rover/Servo -- 0 0 0 0 0 0
