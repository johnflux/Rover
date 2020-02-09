#!/usr/bin/env python3 
import serial
import socket
import time


def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

ser = serial.Serial('/dev/ttyUSB0', 115200, rtscts=False, dsrdtr=True, xonxoff=False)  # open serial port
ser.readline()
def send_ip():
    ip = get_ip()
    print("Ip is", ip)
    #time.sleep(3)
    ret = ser.write(ip.encode('ascii'))     # write a string
    ser.flush()
    #ser.close()             # close port
is_arm_led_on = False
last_arm_led = 'o' #off
def arm_led_on():
    global last_arm_led, is_arm_led_on
    if last_arm_led == 'r':
        last_arm_led='g'
    elif last_arm_led == 'g':
        last_arm_led='b'
    else:
        last_arm_led='r'
    is_arm_led_on = True
    ser.write(last_arm_led.encode('ascii'))
def arm_led_off():
    global last_arm_led, is_arm_led_on
    if is_arm_led_on:
        is_arm_led_on = False
        ser.write(b'o')

if __name__ == "__main__":
    send_ip()

