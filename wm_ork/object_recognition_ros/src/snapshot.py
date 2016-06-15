#!/usr/bin/env python

"""
This file is a node used to read from RecognizedObjectsArray
"""

import rospy
from object_recognition_msgs.msg import RecognizedObjectArray

class Snapshot:
    def __init__(self):

        self.s = rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, self.ObjectArrayCallback)

    def ObjectArrayCallback(self, msg):

        self.ObjectArray = msg.objects
        self.length = len(self.ObjectArray)

        rospy.loginfo("yolo")
        self.types = [o.type for o in self.ObjectArray]
        print self.length
        print self.types

if __name__ == '__main__':
    rospy.init_node('snapshot_node', log_level=rospy.DEBUG)
    Snapshot()
    rospy.spin()