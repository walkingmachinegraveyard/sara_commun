#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from smach import StateMachine
import actionlib
import time
import threading
from smach_ros import SimpleActionState
from smach_ros import ActionServerWrapper
from std_msgs.msg import String
from std_msgs.msg import UInt8
from wm_interpreter.msg import *
from collections import Counter

TIMEOUT_LENGTH = 10


# define state Idle
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Stop', 'Sarah'],
                             input_keys=['Idle_lastWord_in',
                                         'Idle_lastState_in'],
                             output_keys=['Idle_lastWord_out',
                                          'Idle_lastState_out'])
        self.word = ""
        self.state = "Idle"
        rospy.Subscriber("/recognizer_1/output", String, self.callback, queue_size=1)
        self.pub = rospy.Publisher('sara_tts', String, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state Idle')
        rospy.loginfo('Idle - Waiting for keyword: SARAH')
        self.word = ""
        while True:
            if self.word == 'stop':
                userdata.Idle_lastWord_out = self.word
                userdata.Idle_lastState_out = self.state
                return 'Stop'
            if self.word == 'stop':
                userdata.Idle_lastWord_out = self.word
                userdata.Idle_lastState_out = self.state
                return 'Stop'
            if self.word == 'sarah':
                userdata.Idle_lastWord_out = self.word
                userdata.Idle_lastState_out = self.state
                return 'Sarah'

    def callback(self, data):
        if data.data == "stop":
            rospy.loginfo('Idle - Keyword STOP detected !!')
            self.word = data.data

        if data.data == "sarah":
            rospy.loginfo('Idle - Keyword SARAH detected !!')
            self.word = data.data

    def SayX(self, ToSay_str):
        rospy.loginfo(ToSay_str)
        self.pub.publish(ToSay_str)


# define state AnswerQuestion
class SaidObject(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Done', 'aborted'],
                             input_keys=['AQ_question_in'])

        self.tts_pub = rospy.Publisher('sara_tts', String, queue_size=1, latch=True)
 
    def execute(self, userdata):
        rospy.loginfo('-- Executing state WaitingConfirmation --')
        self.SayX(self.ANSWERS[userdata.AQ_question_in])
        return 'Done'

    def SayX(self, ToSay_str):
        rospy.loginfo(ToSay_str)
        self.tts_pub.publish(ToSay_str)



class Tensorflow(StateMachine):
    def __init__(self):
        super(Tensorflow, self).__init__(outcomes=['success', 'aborted', 'preempted'],
                                                output_keys=['result'])

        with self:

            self.add('SaidObject', SaidObject(),
                     transitions={'Done': 'SaidObject',
                                  'aborted': 'aborted'},
                     remapping={'WQ_question_out': 'question'})


if __name__ == '__main__':
    rospy.init_node('interpreter')
    Tensorflow().execute()
    while not rospy.is_shutdown():
        rospy.spin()
