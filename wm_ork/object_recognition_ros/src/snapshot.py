#!/usr/bin/env python

"""
This file is a node used to read from RecognizedObjectsArray
"""

import rospy
from object_recognition_msgs.msg import RecognizedObjectArray
from sensor_msgs.msg import PointCloud2

class Snapshot:
    def __init__(self):

        self.s = rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, self.ObjectArrayCallback)
        self.repub = rospy.Publisher("/sensor_msgs/Recognized_Object_PC2", PointCloud2, queue_size=100)

    def ObjectArrayCallback(self, msg):

        self.ObjectArray = msg.objects
        self.length = len(self.ObjectArray)
        self.frame = self

        if self.length > 0:
            self.types = [o.type for o in self.ObjectArray]
            self.object = next(o for o in self.ObjectArray if o.type.key == '8535f31de64785045d3aaafe70002f3b')
            self.object.point_clouds

            self.repub.publish(self.object.point_clouds[0])
            print self.object.point_clouds[0]
if __name__ == '__main__':
    rospy.init_node('snapshot_node', log_level=rospy.DEBUG)
    Snapshot()
    rospy.spin()
