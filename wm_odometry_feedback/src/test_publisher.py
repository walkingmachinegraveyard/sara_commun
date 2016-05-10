#!/usr/bin/env python

import rospy
from roboteq_msgs.msg import Feedback


def odom_test():
    pubFLW = rospy.Publisher('roboteq_driver_FLW/feedback', Feedback, queue_size=1)
    pubFRW = rospy.Publisher('roboteq_driver_FRW/feedback', Feedback, queue_size=1)
    pubRLW = rospy.Publisher('roboteq_driver_RLW/feedback', Feedback, queue_size=1)
    pubRRW = rospy.Publisher('roboteq_driver_RRW/feedback', Feedback, queue_size=1)

    rate = rospy.Rate(25)   # Hz

    while not rospy.is_shutdown():
        flw = Feedback()
        flw.measured_velocity = 12
        pubFLW.publish(flw)

        frw = Feedback()
        frw.measured_velocity = 12
        pubFRW.publish(frw)

        rlw = Feedback()
        rlw.measured_velocity = 12
        pubRLW.publish(rlw)

        rrw = Feedback()
        rrw.measured_velocity = 12
        pubRRW.publish(rrw)

        rate.sleep()


if __name__ == '__main__':

    try:
        rospy.init_node('odom_test_node')

        odom_test()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
