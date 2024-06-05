#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist 
import tkinter as tk

def cmd_publisher():
    cmd = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size= 10)
    #cmd1 = rospy.Publisher('aom/cmd_vel', Twist, queue_size= 10)
    rospy.init_node('turlle_talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cmd_msd = Twist()
        cmd_msd.linear.x = 0.5
        cmd_msd.angular.z = 0.2
        cmd.publish(cmd_msd)
        #cmd1.publish(cmd_msd)
        rate.sleep()

if __name__ == '__main__':
    try:
        cmd_publisher()
    except rospy.ROSInterruptException:
        pass
    