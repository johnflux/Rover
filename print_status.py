#!/usr/bin/python3
import rospy

from rosmon_msgs.msg import State

def main():
    rospy.init_node("print_status")

    def on_new_state(state):
        state_to_str = ["Idle", "Running", "Crashed", "Waiting"]
        for node in state.nodes:
            print(node.ns, node.name, '\t', state_to_str[node.state], "\tUser:", round(100*node.user_load), "\tSys:", round(100*node.system_load), "\tMem:", round(node.memory/1024) + 'kb')
        exit(0)

    state = rospy.wait_for_message("/rosmon/state", State, 2)
    on_new_state(state)

    print("You can start/stop/restart services like:")
    print('  rosservice call /rosmon/start_stop "joy_drive" "" 2')

main()
