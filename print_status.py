#!/usr/bin/python3
import rospy
from termcolor import colored

from rosmon_msgs.msg import State
import os
import rpi_throttled_status

def main():
    rospy.init_node("print_status")

    def on_new_state(state):
        state_to_str = ["Idle", colored("Running","green"), colored("Crashed","red"), "Waiting"]
        for node in state.nodes:
            print(colored((node.ns + ' ' + node.name).ljust(15),'cyan', attrs=['bold']), '\t', state_to_str[node.state], "\tUser:", str(round(100*node.user_load))+'%', "\tSys:", str(round(100*node.system_load))+'%', "  \tMem:", round(node.memory/1024),'kb')

    try:
        state = rospy.wait_for_message("/rosmon/state", State, 2)
        on_new_state(state)
    except:
        print(colored('Timeout waiting for rosmon.  Note that the raspberry pi can take a minute to fully start up.', "red"))
        print(colored("Running: 'service rover status':", attrs=['bold']))
        os.system("service rover status")

    print()

    print("You can start (1), stop (2), or restart (3) services like:")
    print(colored('  rosservice call /rosmon/start_stop "joy_drive" "" 2',attrs=['bold']))
    print()

    print("or view the ncurses gui for the services by doing:")
    print(colored("  screen -r monlaunch", attrs=['bold']))
    print("Press ", colored('ctrl+a+d',attrs=['bold'])," to detach again")
    print("\x1B[95m")
    os.system("screen -ls")
    print("\x1B[0m")

    rpi_throttled_status.printThrottledStatus()

main()
