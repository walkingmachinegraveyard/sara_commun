#!/usr/bin/env python

import rospy
from cob_perception_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import PoseStamped


class Interpreter:
    def __init__(self):

        self.repub = rospy.Publisher("/geometry_msgs/PoseStamped", PoseStamped, queue_size=100)

        self.s = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, self.arraycallback)

        self.min_y_ = -1.0
        self.max_y_ = 1.0
        self.min_x_ = -0.2
        self.max_x_ = 2.0
        self.max_z_ = 2.0

    def arraycallback(self, msg):

        self.People = msg.people[-1]
        self.geo = PoseStamped()
        self.geo.pose.position = self.People.pos
        self.geo.header = self.People.header

        if (self.People.pos.x > self.min_y_) and (self.People.pos.y < self.max_y_) and \
                (self.People.pos.x < self.max_x_) and (self.People.pos.x > self.min_x_) and \
                (self.People.pos.z < self.max_z_):
            rospy.loginfo("yolo")
            self.repub.publish(self.geo)

if __name__ == '__main__':
    rospy.init_node('position_interpreter_node', log_level=rospy.DEBUG)
    Interpreter()
    rospy.spin()
