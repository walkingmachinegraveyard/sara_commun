#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':

    rospy.init_node('robocup_test_node')

    goal = MoveBaseGoal()
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rospy.Time.now()

    pose.pose.position.x = 1.0
    pose.pose.position.y = 1.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.z = 1.0

    goal.target_pose = pose

    move_base_client = SimpleActionClient('move_base', MoveBaseAction)

    move_base_client.send_goal(goal)

    move_base_client.wait_for_result(rospy.Duration(20))
