#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String
import termios
import tty
import sys

def getKey():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def vels(speed,turn):
    return Twist(Vector3(speed, 0, 0), Vector3(0, 0, turn))

def control():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('keyboard_control')
    rate = rospy.Rate(10) # 10hz
    speed = 0
    turn = 0
    while not rospy.is_shutdown():
        key = getKey()
        if key == 'w':
            speed = 2
            turn = 0
        elif key == 's':
            speed = -2
            turn = 0
        elif key == 'a':
            speed = 0
            turn = 2
        elif key == 'd':
            speed = 0
            turn = -2
        else:
            speed = 0
            turn = 0
        twist = vels(speed,turn)
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass