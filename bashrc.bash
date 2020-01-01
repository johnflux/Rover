# Add to ~/.bashrc:
#   source ~/rover/bashrc.bash

source /opt/ros/kinetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash
source /etc/ubiquity/env.sh
export ROS_PARALLEL_JOBS=-j2 # Limit the number of compile threads due to memory limits
export PS1='\u@\h:\[\033[01;34m\]\w\[\033[00m\]$(__git_ps1 " (%s)")\$ '


pifi status
ifconfig | grep "inet addr" | grep -v 127.0.0.1

~/rover/print_status.py

