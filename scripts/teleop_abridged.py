
#!/usr/bin/env python3
import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist, Vector3
from math import pi

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def key_map(key, vel_msg):
    # Checking user input
    if key != '\x03':
        key = getKey()
        if key == 'w':
            vel_msg.angular.z = 0
            vel_msg.linear.x = 1
        elif key == 'a':
            vel_msg.linear.x = 0
            vel_msg.angular.z = 1
        elif key == 's':
            vel_msg.angular.z = 0
            vel_msg.linear.x = -1
        elif key == 'd':
            vel_msg.linear.x = 0
            vel_msg.angular.z = -1
        elif key == ' ':
            vel_msg.angular.z = 0
            vel_msg.linear.x = 0
    return key