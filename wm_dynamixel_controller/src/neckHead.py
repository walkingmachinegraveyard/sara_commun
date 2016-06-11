#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Float64
import dynamixel_msgs.msg
import sensor_msgs.msg
from threading import Lock

mutex = Lock()

class dynamixelState:
    def __init__(self):
        self.pub = rospy.Publisher('/neckHead/state', sensor_msgs.msg.JointState, queue_size=1)
        self.sub = rospy.Subscriber('/neckHead_controller/state', dynamixel_msgs.msg.JointState, self.callback)
        self.msg = sensor_msgs.msg.JointState()
        self.msg.name.append("")
        self.msg.velocity.append(0)
        self.msg.position.append(0)
        self.msg.effort.append(0)

    def callback(self, data):
        mutex.acquire()
        self.msg.name[0] = data.name
        self.msg.position[0] = data.current_pos
        self.msg.velocity[0] = data.velocity
        self.msg.effort[0] = 0
        mutex.release()

    def publish(self):
        mutex.acquire()
        self.pub.publish(self.msg)
        mutex.release()

def main():
    rospy.init_node('dynamixel_publisher')
    rate = rospy.Rate(10)  # 10hz
    neckHeadDynamixel = dynamixelState()
    while not rospy.is_shutdown():
        if(neckHeadDynamixel.msg.name[0] != ""):
            neckHeadDynamixel.publish()
        else:
            rospy.loginfo("neckHead State Publisher - Waiting for dynamixel node to launch")
            rospy.sleep(5)
        rate.sleep()

if __name__ == '__main__':
    main()
