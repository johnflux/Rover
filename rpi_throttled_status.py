#!/usr/bin/env python3
from termcolor import colored
import subprocess

def getThrottledStatus():
    GET_THROTTLED_CMD = 'vcgencmd get_throttled'
    return subprocess.check_output(GET_THROTTLED_CMD, shell=True)

def printThrottledStatus():
    MESSAGES = {
        0: colored('Under-voltage!','red'),
        1: colored('ARM frequency capped!','red'),
        2: colored('Currently throttled!','red'),
        3: colored('Soft temperature limit active','red'),
        16: colored('Under-voltage has occurred since last reboot.', 'yellow'),
        17: colored('Throttling has occurred since last reboot.', 'yellow'),
        18: colored('ARM frequency capped has occurred since last reboot.','yellow'),
        19: colored('Soft temperature limit has occurred', 'yellow')
    }

    throttled_output = getThrottledStatus()
    throttled_binary = bin(int(throttled_output.split(b'=')[1], 0))

    warnings = 0
    errors = 0
    for position, message in MESSAGES.items():
        # Check for the binary digits to be "on" for each warning message
        if len(throttled_binary) > position and throttled_binary[0 - position - 1] == '1':
            print("  ", message)
            warnings += 1
            if position  <= 3:
                errors += 1

    if warnings == 0:
        print(colored("No throttling issues - all looking good!","green"))
    elif errors == 0:
        print(colored("But no throttling, voltage/frequency/temperature problems right now", "green"))

if __name__ == "__main__":
    printThrottledStatus()

