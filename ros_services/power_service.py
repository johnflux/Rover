#!/usr/bin/env python
import rospy
import subprocess
from PowerOff.srv import PowerOff

def handle_poweroff(req):
    print("Shutting down")
    subprocess.call("./delayed_shutdown.sh", shell=True)
    return True

def handle_reboot(req):
    print("Rebooting")
    subprocess.call("./delayed_reboot.sh", shell=True)
    return True

def power_server():
    rospy.init_node('power_off_server')
    s_poweroff = rospy.Service('power_off', PowerOff, handle_poweroff)
    s_reboot = rospy.Service('reboot', PowerOff, handle_reboot)
    rospy.spin()

if __name__ == "__main__":
    power_server()
