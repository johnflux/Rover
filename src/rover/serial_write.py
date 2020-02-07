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

def send_ip():
    ip = get_ip()
    print("Ip is", ip)
    ser = serial.Serial('/dev/ttyUSB0', 115200, rtscts=False, dsrdtr=True, xonxoff=False)  # open serial port
    time.sleep(3)
    ser.readline()
    ret = ser.write(ip.encode('ascii'))     # write a string
    ser.flush()
    ser.close()             # close port

if __name__ == "__main__":
    send_ip()

