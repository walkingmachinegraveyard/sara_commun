#!/usr/bin/env python

"""
This file is a node used to read from RecognizedObjectsArray
"""

import rospy
import os
from object_recognition_msgs.msg import RecognizedObjectArray
from object_recognition_msgs.srv import GetObjectInformation, GetObjectInformationRequest
from sensor_msgs.msg import PointCloud2


class Snapshot:
    def __init__(self):
        self.i = 0
        self.s = rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, self.object_array_callback)
        self.ork_srv = rospy.ServiceProxy('get_object_info', GetObjectInformation)

    def object_array_callback(self, msg):
        object_array = msg.objects
        if len(object_array) > 0:
            types = [o.type for o in object_array]
            file_name = "test.txt"  # "test-"+str(self.i)+".test"
            print file_name
            object_info = open(file_name, "w")
            try:
                for res in types:
                    req = GetObjectInformationRequest()
                    req.type = res
                    info = self.ork_srv(req)
                    print info.information.name
                    object_info.write(info.information.name)
                    object_info.write("\n")

            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
            object_info.close()
            self.i += 1


if __name__ == '__main__':
    rospy.init_node('extract_object_info', log_level=rospy.DEBUG)
    Snapshot()
    rospy.spin()
