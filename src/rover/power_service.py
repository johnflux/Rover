#!/usr/bin/env python3
import rospy
import subprocess
from rover.srv import PowerOff

def handle_poweroff(req):
    print("Shutting down")
    # Note: We can use 'sudo reboot' because we did:
    #   $ sudo visudo
    # And added:
    #   script  ALL=(root)  NOPASSWD: /sbin/reboot ""
    return subprocess.call("(sleep 1; sudo reboot -h) &", shell=True)

def handle_reboot(req):
    print("Rebooting")
    return subprocess.call("(sleep 1; sudo reboot) &", shell=True)

def main():
    rospy.init_node('power_off')
    s_poweroff = rospy.Service('power_off/power_off', PowerOff, handle_poweroff)
    s_reboot = rospy.Service('power_off/reboot', PowerOff, handle_reboot)
    rospy.spin()

if __name__ == "__main__":
    main()
