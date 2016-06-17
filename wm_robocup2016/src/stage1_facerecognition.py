#!/usr/bin/env python

import roslib;

import rospy
import smach
import smach_ros
from people_msgs.msg import *
from math import *
from std_msgs.msg import Float64, String
from cob_perception_msgs.msg import *

DISTANCE_DETECTION = 2

class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_done'])
        #self.neck_pub = rospy.Publisher('neckHead_controller/command', Float64, queue_size=1, latch=True)

    def execute(self, ud):
        rospy.loginfo("Entered 'INIT' state.")

        neck_cmd = Float64()
        neck_cmd.data = 0.0
        #self.neck_pub.publish(neck_cmd)

        return 'init_done'

#Wait for a person being detect <=DISTANCE_DETECTION from the robot
class People_Detector(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found'])
        self.found = False

    def callbackPeople(self, data):
        if len(data.people) > 0:
            for people in data.people:

                x = people.pos.x
                y = people.pos.y

                distance = hypot(x, y)
                if distance <= DISTANCE_DETECTION:
                    self.found = True

    def execute(self, userdata):
        rospy.loginfo('Executing state PEOPLE_DETECTOR')
        peopleSub = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, self.callbackPeople)
        rospy.loginfo('Looking for someone...')
        #while not self.found:
        #    rospy.sleep(1);
        return 'found'

#Move head up and down until it see a face
class Face_Detector(smach.State):
    def __init__(self):
        self.found = False
        smach.State.__init__(self, outcomes=['found', 'not_found'])
        self.neck_pub = rospy.Publisher('neckHead_controller/command', Float64, queue_size=1, latch=True)
        self.face_sub = rospy.Subscriber('face_detector/face_positions', ColorDepthImageArray, self.face_callback)

    def face_callback(self, data):
        if len(data.head_detections) > 0:
            self.found = True

    def execute(self, userdata):
        rospy.loginfo('Executing state FACE_DETECTOR')
        while not self.found:
            rospy.sleep(1);
        return 'found'

#Record the face of the operator
class Record_Face(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RECORDING')

        #TODO demander nom, enregistrer la face, sauvegarder label photo

        rospy.sleep(5);
        return 'finished'

#Wait 10 seconds
class Wait_10(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WAITING')
        rospy.sleep(10);
        return 'finished'

#Do a 180 degrees turn
class Turn_180(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TURNING_180')

        # TODO tourner 180 degrés

        rospy.sleep(5);
        return 'finished'

#Find the crowd (5-10 people)
class Find_Crowd(smach.State):
    def __init__(self):
        self.found = False
        self.number = 0
        smach.State.__init__(self, outcomes=['found', 'not_found'])
        self.pub_voice = rospy.Publisher('chatter', String, queue_size=10)
        self.face_sub = rospy.Subscriber('face_detector/face_positions', ColorDepthImageArray, self.face_callback)

    def face_callback(self, data):
        if len(data.head_detections) >= 5 or len(data.head_detections) <= 10 :
            self.found = True
            self.number = len(data.head_detections)

    def execute(self, userdata):
        rospy.loginfo('Executing state FINDING_CROWD')

        # TODO trouver la foule (5-10 personnes)

        while not self.found :
            rospy.sleep(1)

        self.pub_voice.publish("I found the crowd, there is " + str(self.number) + " people")

        return 'found'

#Find the recorded face of the operator
class Find_Operator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FINDING_OPERATOR')

        # TODO trouver l'operateur, dire la position

        rospy.sleep(5);
        return 'found'

def main():
    rospy.init_node('smach_person_recognition')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end'])

    # Open the container
    with sm:

        smach.StateMachine.add('INIT', InitState(),
                               transitions={'init_done': 'PEOPLE_DETECTOR'})

        smach.StateMachine.add('PEOPLE_DETECTOR', People_Detector(),
                               transitions={'found': 'FACE_DETECTOR','not_found':'PEOPLE_DETECTOR'})

        smach.StateMachine.add('FACE_DETECTOR', Face_Detector(),
                               transitions={'found': 'RECORDING', 'not_found': 'FACE_DETECTOR'})

        smach.StateMachine.add('RECORDING', Record_Face(),
                               transitions={'finished': 'WAITING'})

        smach.StateMachine.add('WAITING', Wait_10(),
                               transitions={'finished': 'TURNING_180'})

        smach.StateMachine.add('TURNING_180', Turn_180(),
                               transitions={'finished': 'FINDING_CROWD'})

        smach.StateMachine.add('FINDING_CROWD', Find_Crowd(),
                               transitions={'found': 'FINDING_OPERATOR', 'not_found':'FINDING_CROWD'})

        smach.StateMachine.add('FINDING_OPERATOR', Find_Operator(),
                               transitions={'found': 'end', 'not_found':'FINDING_OPERATOR'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()