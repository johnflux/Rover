#!/bin/bash
# See the comments in launch.sh
cd /home/ubuntu/rover/web
echo "Starting monlaunch"
#npm run build
bash -c "source /home/ubuntu/catkin_ws/devel/setup.bash && mon launch --name rosmon rover drive.launch"

