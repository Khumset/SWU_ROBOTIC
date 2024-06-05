#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from tkinter import *


def turtle_velocity_publisher():
    rospy.init_node('turtle_velocity_publisher')
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10 Hz


    def forward():
        vel_msg = Twist()
        vel_msg.linear.x = 0.5
        vel_msg.angular.z = 0.0
        velocity_publisher.publish(vel_msg)

    def backward():
        vel_msg = Twist()
        vel_msg.linear.x = -0.5
        vel_msg.angular.z = 0.0
        velocity_publisher.publish(vel_msg)

    def turn_left():
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.5
        velocity_publisher.publish(vel_msg)

    def turn_right():
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = -0.5
        velocity_publisher.publish(vel_msg)

    def stop():
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        velocity_publisher.publish(vel_msg)

    window = Tk()
    window.title("Turtlebot Control")

    forward_button = Button(window, text="Forward", command=forward)
    forward_button.grid(row=0, column=0)

    backward_button = Button(window, text="Backward", command=backward)
    backward_button.grid(row=0, column=1)

    turn_left_button = Button(window, text="Turn Left", command=turn_left)
    turn_left_button.grid(row=1, column=0)

    turn_right_button = Button(window, text="Turn Right", command=turn_right)
    turn_right_button.grid(row=1, column=1)

    stop_button = Button(window, text="Stop", command=stop)
    stop_button.grid(row=2, columnspan=2)

    window.mainloop()

if __name__ == '__main__':
    try:
        turtle_velocity_publisher()
    except rospy.ROSInterruptException:
        pass