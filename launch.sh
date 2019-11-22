#!/bin/bash
cd /home/ubuntu/rover/web
#npm run build
bash -c "source /home/ubuntu/catkin_ws/devel/setup.bash && mon launch --name rosmon --disable-ui rover drive.launch"

