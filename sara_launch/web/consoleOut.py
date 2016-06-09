#!/usr/bin/env python
import rospy
from rosgraph_msgs.msg import Log
from std_msgs.msg import String


def callback(data):
    pub = rospy.Publisher('web_log', String)

    message = "["+data.name+"] : " + data.msg
    pub.publish(message)

def listener():
    rospy.init_node('rosout_webListener', anonymous=True)
    rospy.Subscriber("rosout", Log, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException: pass