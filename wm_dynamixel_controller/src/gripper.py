#!/usr/bin/env python
import roslib
roslib.load_manifest('my_dynamixel_tutorial')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg

class Gripper:
    def __init__(self, motor_name):
        self.name = motor_name
        self.pub = rospy.Publisher('/gripper_controller/command', Float64)
        rospy.init_node('gripper_node')
        rospy.loginfo('Waiting for gripper control')

    def open_gripper(self):
        self.pub.publish(1.5)

    def close_gripper(self):
        self.pub.publish(-1.5)