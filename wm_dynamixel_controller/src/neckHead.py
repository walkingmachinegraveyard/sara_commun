#!/usr/bin/env python
import roslib

import rospy
from std_msgs.msg import Float64
import dynamixel_msgs.msg
import sensor_msgs.msg

class dynamixelState:
    def __init__(self):
        self.pub = rospy.Publisher('/neckHead/state', sensor_msgs.msg.JointState)
        self.sub = rospy.Subscriber('/neckHead_controller/state', dynamixel_msgs.msg.JointState, self.callback)
        self.msg = sensor_msgs.msg.JointState()

    def callback(self, data):
        self.msg.name = data.name
        self.msg.position = data.current_pos
        self.msg.velocity = data.velocity
        self.msg.effort = 0

    def publish(self):
        self.pub.publish(self.msg)

def main():
    rospy.init_node('dynamixel_publisher')
    rate = rospy.Rate(10)  # 10hz
    neckHeadDynamixel = dynamixelState()
    while not rospy.is_shutdown():
        neckHeadDynamixel.publish()
        rate.sleep()

if __name__ == '__main__':
    main()
