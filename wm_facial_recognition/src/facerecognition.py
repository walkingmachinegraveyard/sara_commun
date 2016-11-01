#!/usr/bin/env python

import roslib;

import rospy
import smach
import smach_ros
from cob_perception_msgs.msg import *
from cob_people_detection.msg import *
from people_msgs.msg import *
from math import *
from std_msgs.msg import Bool, String
import tf.transformations
from math import *

import actionlib
from actionlib_msgs.msg import *

from geometry_msgs.msg import Twist

from move_base_msgs.msg import *

DISTANCE_DETECTION = 2
PERSON = "operator1"

#Minimum and maximum headcount to detect in a crowd
MIN_HEADCOUNT_CROWD = 2
MAX_HEADCOUNT_CROWD = 10

#Amount of pictures to take to save in databse
NUM_FACE_CAPTURES = 40

#Wait time for crowd, in seconds
WAIT_TIME = 10

class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_done'])

    def execute(self, ud):
        rospy.loginfo("Entered 'INIT' state.")

        return 'init_done'

#Wait for a person being detect <=DISTANCE_DETECTION from the robot
class WaitingAsk(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['asked'])
        #Subscription to the StateMachine
        peopleSub = rospy.Subscriber("facial_recognition/start", std_msgs.msg.Bool, self.callbackPeople)
        self.action = False

    def callbackPeople(self, data):
        self.action = data

    def execute(self, userdata):
        rospy.loginfo('Executing state WaitingAsk')

        rospy.loginfo('Waiting for order')
        while not self.action:
            rospy.sleep(1)
        self.action = False
        return 'asked'

#Move head up and down until it see a face
class Speak(smach.State):
    def __init__(self):
        self.found = False
        smach.State.__init__(self, outcomes=['done'])

        self.numberOfPeoples = 0

        #publication in Sara Voice
        self.pub_voice = rospy.Publisher('sara_tts', String, queue_size=10)
        #publication at the end, to indicate the end of process
        self.pub_stop = rospy.Publisher('facial_recognition/stop', std_msgs.msg.Bool,  queue_size=1)
        #subscription to face_detector
        face_sub = rospy.Subscriber('face_detector/face_positions', ColorDepthImageArray, self.face_callback)

    def face_callback(self, data):
        self.numberOfPeoples = len(data.head_detections)

    def execute(self, userdata):
        rospy.loginfo('Executing state SPEAK')
        if self.numberOfPeoples < 2 :
            self.pub_voice.publish("I see "+ str(self.numberOfPeoples) +" person!")
        else:
            self.pub_voice.publish("I see "+ str(self.numberOfPeoples) +" persons!")
        rospy.sleep(2)
        self.pub_stop.publish(True)
        return 'done'

def main():
    rospy.init_node('smach_person_recognition')

    rospy.loginfo("----------starting python script----------")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end'])


    sm.userdata.operator_name = ""
    rospy.loginfo("Packages needed :")
    rospy.loginfo("- openni2_launch openni2.launch")
    rospy.loginfo("- wm_tts wm_tts.launch")

    # Open the container
    with sm:
        smach.StateMachine.add('INIT', InitState(), transitions={'init_done': 'WAITING_ASK'})
        smach.StateMachine.add('WAITING_ASK', WaitingAsk(), transitions={'asked': 'SPEAK'})
        smach.StateMachine.add('SPEAK', Speak(), transitions={'done': 'WAITING_ASK'})


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_Face_Recognition')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()
