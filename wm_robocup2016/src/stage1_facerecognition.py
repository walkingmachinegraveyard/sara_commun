#!/usr/bin/env python

import roslib;

import rospy
import smach
import smach_ros
from cob_perception_msgs.msg import *
from cob_people_detection.msg import *
from people_msgs.msg import *
from math import *
from std_msgs.msg import Float64, String
import tf.transformations
from math import *

import actionlib
from actionlib_msgs.msg import *

from geometry_msgs.msg import Twist



from move_base_msgs.msg import *

DISTANCE_DETECTION = 2

class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_done'])
        self.neck_pub = rospy.Publisher('neckHead_controller/command', Float64, queue_size=1, latch=True)

    def execute(self, ud):
        rospy.loginfo("Entered 'INIT' state.")

        #TODO face green

        neck_cmd = Float64()
        neck_cmd.data = 0.0
        self.neck_pub.publish(neck_cmd)

        return 'init_done'

#Wait for a person being detect <=DISTANCE_DETECTION from the robot
class People_Detector(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found'])
        peopleSub = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, self.callbackPeople)
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

        rospy.loginfo('Looking for someone...')
        #TODO face yellow

        while not self.found:
            rospy.sleep(1)

        #TODO face green
        return 'found'

#Move head up and down until it see a face
class Face_Detector(smach.State):
    def __init__(self):
        self.found = False
        smach.State.__init__(self, outcomes=['found', 'not_found'])

        self.pub_voice = rospy.Publisher('sara_tts', String, queue_size=10)
        self.neck_pub = rospy.Publisher('neckHead_controller/command', Float64, queue_size=1, latch=True)
        self.face_sub = rospy.Subscriber('face_detector/face_positions', ColorDepthImageArray, self.face_callback)

    def face_callback(self, data):
        if len(data.head_detections) > 0:
            self.found = True

    def execute(self, userdata):
        rospy.loginfo('Executing state FACE_DETECTOR')

        #TODO face yellow
        neck_cmd = Float64()
        neck_cmd.data = 0.0
        direction = 1;
        while not self.found:
            if neck_cmd.data <= -2.0:
                direction = 1
            elif neck_cmd.data >= 1.0:
                direction = -1

            neck_cmd.data += direction * 0.1
            self.neck_pub.publish(neck_cmd)
            rospy.sleep(0.1);

        #TODO face green
        self.pub_voice.publish("Hi, nice to meet you !")
        return 'found'

class Ask_Name(smach.State):
    def __init__(self):
        self.name = ""
        self.nameAsked = False
        self.pub_voice = rospy.Publisher('sara_tts', String, queue_size=10)
        self.name_sub = rospy.Subscriber('recognizer_1/output', String, self.name_callback)
        smach.State.__init__(self,
                             outcomes=['name_found','name_not_found'],
                             output_keys=['ask_name_out'])

    def name_callback(self, data):
        if self.nameAsked:
            self.name = data.data.split(' ', 1)[0]

    def execute(self, userdata):
        rospy.loginfo('Executing state ASK_NAME')

        #TODO face yellow

        self.pub_voice.publish("What is your name ?")
        self.nameAsked = True

        while self.name == "":
            rospy.sleep(1);

        userdata.ask_name_out = self.name

        #TODO face green
        self.pub_voice.publish("Great " + self.name)
        return 'name_found'

#Record the face of the operator
class Record_Face(smach.State):
    def __init__(self):
        self.pub_voice = rospy.Publisher('sara_tts', String, queue_size=10)
        smach.State.__init__(self, outcomes=['finished'],
                             input_keys=['record_name_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RECORDING')



        dataclient = actionlib.SimpleActionClient('/face_capture/add_data_server', addDataAction)
        if dataclient.wait_for_server(rospy.Duration.from_sec(2.0)):

            self.pub_voice.publish("Stay still while I memorize your face")
            # Goal du Actionlib
            datagoal = addDataGoal()
            datagoal.label = userdata.record_name_in
            datagoal.capture_mode = 1
            datagoal.continuous_mode_images_to_capture = 30
            datagoal.continuous_mode_delay = 0
            dataclient.send_goal(datagoal)

            rospy.loginfo('Recording face...')
            dataclient.wait_for_result()

            if dataclient.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo(userdata.record_name_in + " was added")

                loadclient = actionlib.SimpleActionClient("/face_recognizer/load_model_server", loadModelAction)

                if loadclient.wait_for_server(rospy.Duration.from_sec(2.0)):

                    # Goal du Actionlib
                    loadgoal = loadModelGoal()

                    # loadgoal.labels = [" ", "q"]
                    loadclient.send_goal(loadgoal)

                    loadclient.wait_for_result()

                    if loadclient.get_state() == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Database refreshed")
                        self.pub_voice.publish("I finished memorizing your face")

                    else:
                        rospy.loginfo("Refresh failed!")

            else:
                rospy.loginfo("Record failed!")
        else:
            rospy.loginfo('Recording server not found...')


        rospy.sleep(5);
        return 'finished'

#Wait 10 seconds
class Wait_10(smach.State):
    def __init__(self):
        self.pub_voice = rospy.Publisher('sara_tts', String, queue_size=10)
        smach.State.__init__(self, outcomes=['finished'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WAITING')
        self.pub_voice.publish("I'm waiting 10 seconds before turning")
        #TODO waiting face
        rospy.sleep(10);
        #TODO green face
        return 'finished'

#Do a 180 degrees turn
class Turn_180(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'])
        self.pub_voice = rospy.Publisher('sara_tts', String, queue_size=10)
        self.pub_cmdvel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        #self.moveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.goal = MoveBaseGoal()
        #self.goal.target_pose.header.frame_id = 'base_link'

    def execute(self, userdata):
        rospy.loginfo('Executing state TURNING_180')

        # TODO tourner 180 degres
        self.pub_voice.publish("Watch out ! I'm turning")
        rospy.sleep(3)
        twist = Twist()
        twist.angular.z = pi/8
        timenow = rospy.get_time()

        while rospy.get_time() - timenow < 8:
        	self.pub_cmdvel.publish(twist)
        
		
        #quaternion = tf.transformations.quaternion_from_euler(0, 0, pi)
        #self.goal.target_pose.pose.orientation.x = quaternion[0]
        #self.goal.target_pose.pose.orientation.y = quaternion[1]
        #self.goal.target_pose.pose.orientation.z = quaternion[2]
        #self.goal.target_pose.pose.orientation.w = quaternion[3]

        #self.moveBaseClient.send_goal(self.goal)

        return 'finished'

#Find the crowd (5-10 people)
class Find_Crowd(smach.State):
    def __init__(self):
        self.found = False
        self.number = 0
        smach.State.__init__(self, outcomes=['found', 'not_found'])
        self.pub_voice = rospy.Publisher('sara_tts', String, queue_size=10)
        self.face_sub = rospy.Subscriber('face_detector/face_positions', ColorDepthImageArray, self.face_callback)

    def face_callback(self, data):
        if len(data.head_detections) >= 1 and len(data.head_detections) <= 10 :
            self.found = True
            self.number = len(data.head_detections)

    def execute(self, userdata):
        rospy.loginfo('Executing state FINDING_CROWD')

        # TODO trouver la foule (5-10 personnes) (move ?)

        while not self.found :
            #TODO face yellow
            rospy.sleep(1)

        #TODO face green
        self.pub_voice.publish("I found the crowd, there is " + str(self.number) + " people")

        return 'found'

#Find the recorded face of the operator
class Find_Operator(smach.State):
    def __init__(self):
        self.found = False
        self.name = ""
        smach.State.__init__(self,
                             outcomes=['found', 'not_found'],
                             input_keys=['finding_name_in'])
        self.pub_voice = rospy.Publisher('sara_tts', String, queue_size=10)
        self.face_sub = rospy.Subscriber('face_recognizer/face_recognitions', DetectionArray, self.face_callback)


    def face_callback(self, data):
        for face in data.detections:
            if face.label == self.name and not self.name == "":
                self.found = True


    def execute(self, userdata):
        self.name = userdata.finding_name_in
        rospy.loginfo('Executing state FINDING_OPERATOR')

        # TODO dire la position

        while not self.found :
            #TODO face yellow
            rospy.sleep(1)

        rospy.loginfo('Operator found')
        if userdata.finding_name_in == "":
            self.pub_voice.publish("I found the operator !")
        else:
            self.pub_voice.publish("I found you," + str(userdata.finding_name_in))

        return 'found'

def main():
    rospy.init_node('smach_person_recognition')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end'])


    sm.userdata.operator_name = ""
    rospy.loginfo("Packages needed :")
    rospy.loginfo("- openni2_launch openni2.launch")
    rospy.loginfo("- cob_people_detection people_detection.launch")
    rospy.loginfo("- leg_detection")
    rospy.loginfo("- wm_tts wm_tts.launch")
    rospy.loginfo("- sara_vocab sara_names.launch")
    rospy.loginfo("- move_base")

    # Open the container
    with sm:



        smach.StateMachine.add('INIT', InitState(),
                               transitions={'init_done': 'PEOPLE_DETECTOR'})

        smach.StateMachine.add('PEOPLE_DETECTOR', People_Detector(),
                               transitions={'found': 'FACE_DETECTOR','not_found':'PEOPLE_DETECTOR'})

        smach.StateMachine.add('FACE_DETECTOR', Face_Detector(),
                               transitions={'found': 'ASK_NAME', 'not_found': 'FACE_DETECTOR'})

        smach.StateMachine.add('ASK_NAME', Ask_Name(),
                               transitions={'name_found': 'RECORDING', 'name_not_found': 'ASK_NAME'},
                               remapping={'ask_name_out':'operator_name'})

        smach.StateMachine.add('RECORDING', Record_Face(),
                               transitions={'finished': 'WAITING'},
                               remapping={'record_name_in': 'operator_name'})

        smach.StateMachine.add('WAITING', Wait_10(),
                               transitions={'finished': 'TURNING_180'})

        smach.StateMachine.add('TURNING_180', Turn_180(),
                               transitions={'finished': 'FINDING_CROWD'})

        smach.StateMachine.add('FINDING_CROWD', Find_Crowd(),
                               transitions={'found': 'FINDING_OPERATOR', 'not_found':'FINDING_CROWD'})

        smach.StateMachine.add('FINDING_OPERATOR', Find_Operator(),
                               transitions={'found': 'end', 'not_found':'FINDING_OPERATOR'},
                               remapping={'finding_name_in': 'operator_name'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_Face_Recognition')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()
