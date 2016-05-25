#!/usr/bin/env python
import roslib

import rospy
from std_msgs.msg import Float64
import dynamixel_msgs.msg
import sensor_msgs.msg

class neckHead:
    def __init__(self):
        self.sub = rospy.Subscriber('/neckHead_controller/state', dynamixel_msgs.msg.JointState, self.callback)
        self.msg = sensor_msgs.msg.JointState()

    def callback(self, data):
        self.msg.name = data.name
        self.msg.position = data.current_pos
        self.msg.velocity = data.velocity
        self.msg.effort = 0

def main(self):
    rospy.init_node('dynamixel_publisher')
    rate = rospy.Rate(10)  # 10hz
    dynamixel = neckHead
    dynamixel.pub.publish(self.msg)
    rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()