#!/usr/bin/env python

import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import String

from cob_people_detection.msg import *


def callback(data):

    rospy.loginfo(data.data)
    command = data.data.split("666") # The devil split

    if command[0] == "addData" and len(command) > 1:

        # Actionlib ajouter une personne
        rospy.loginfo("Adding a new person to database")
        dataclient = actionlib.SimpleActionClient('/cob_people_detection/face_capture/add_data_server', addDataAction)

        rospy.loginfo("Wait for server...")
        if dataclient.wait_for_server(rospy.Duration.from_sec(2.0)):
            # Goal du Actionlib
            datagoal = addDataGoal()
            datagoal.label = command[1]
            datagoal.capture_mode = 1
            datagoal.continuous_mode_images_to_capture = 40
            datagoal.continuous_mode_delay = 1

            dataclient.send_goal(datagoal)
            rospy.loginfo("Acquiring new person : " + command[1])
            dataclient.wait_for_result()

            if dataclient.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo(command[1] + " was added")
            else:
                rospy.loginfo("Record failed!")

    if command[0] == "reload":

        rospy.loginfo("Reloading the database...")
        loadclient = actionlib.SimpleActionClient("/cob_people_detection/face_recognizer/load_model_server", loadModelAction)

        if loadclient.wait_for_server(rospy.Duration.from_sec(2.0)):

            # Goal du Actionlib
            loadgoal = loadModelGoal()

            #loadgoal.labels = [" ", "q"]
            loadclient.send_goal(loadgoal)

            loadclient.wait_for_result()

            if loadclient.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("Database refreshed")
            else:
                rospy.loginfo("Refresh failed!")


# main
def main():
    rospy.init_node('cob_people_detection_client', anonymous=True)
    rospy.Subscriber("node_test_talk", String, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
