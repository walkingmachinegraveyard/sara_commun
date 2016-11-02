#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import actionlib
from actionlib_msgs.msg import *
import time
import threading
from smach_ros import SimpleActionState
from smach.state_machine import StateMachine
from std_msgs.msg import String
from wm_interpreter.msg import *
from move_base_msgs.msg import *
from speech_recognition import SpeechRecognition
from tensorflow import Tensorflow
from following import Following

"""
# gets called when ANY child state terminates
def child_term_cb(outcome_map):
    # terminate all running states if CommGoal finished with outcome 'succeeded'
    if outcome_map['CommGoal'] == 'succeeded':
        return False  # Originaly True

    # in all other case, just keep running, don't terminate anything
    return False


# gets called when ALL child states are terminated
def out_cb(outcome_map):
    if outcome_map['CommGoal'] == 'succeeded':
        return 'succeeded'
    elif outcome_map['CommGoal'] == 'preempted':
        return 'preempted'
    elif outcome_map['CommGoal'] == 'aborted':
        return 'aborted'
"""


# define state Idle
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Stop', 'Sara'],
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

            if self.word == 'sarah':
                userdata.Idle_lastWord_out = self.word
                userdata.Idle_lastState_out = self.state
                return 'Sara'

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

    def request_preempt(self):
        # type: () -> object
        """Overload the preempt request method just to spew an error."""
        smach.State.request_preempt(self)
        rospy.logwarn("Preempted!")


# define state WaitingCommand
class WaitingCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Command', 'Sarah', 'Stop', 'Timeout'],
                             input_keys=['WComm_lastWord_in',
                                         'WComm_lastState_in'],
                             output_keys=['WComm_lastWord_out',
                                          'WComm_lastState_out',
                                          'WComm_lastCommand_out'])
        self.word = ""
        self.state = "WaitingCommand"
        rospy.Subscriber("/recognizer_1/output", String, self.callback, queue_size=1)
        self.pub = rospy.Publisher('sara_tts', String, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state WaitingCommand')
        userdata.WComm_lastState_out = self.state

        self.SayX('Oui maitre')
        self.word = ""
        timeout = time.time() + 15  # 15 sec
        while True:
            if self.word == 'stop':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'Stop'

            if self.word == 'say hello':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                self.SayX(
                    "Bonjour, je suis un robot d'assistance personnelle autonome. Bientot je pourrai accomplir une multitude de taches pour vous !")
                return 'Timeout'

            if self.word == 'follow me':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'Command'

            if self.word == 'question':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'Command'

            if self.word == 'what do you see':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'Command'

            """
            if self.word == 'go foward':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'Command'

            if self.word == 'get me the beer':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'Command'

            if self.word == 'go backward':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'Command'

            if self.word == 'rotate left':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'Command'

            if self.word == 'rotate right':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'Command'
                """

            '''
            if self.word == 'sarah':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'Sarah'
            '''
            if time.time() > timeout:
                userdata.WComm_lastState_out = self.state
                return 'Timeout'

    def callback(self, data):
        if data.data == "stop":
            rospy.loginfo('Wcomm - Keyword STOP detected !!')
            self.word = data.data

        if data.data == "say hello":
            rospy.loginfo('Wcomm - Phrase SAY HI detected !!')
            self.word = data.data

        if data.data == "follow me":
            rospy.loginfo('Wcomm - Phrase FOLLOW ME detected !!')
            self.word = data.data

        if data.data == "question":
            rospy.loginfo('Wcomm - Phrase QUESTION detected !!')
            self.word = data.data

        if data.data == "what do you see":
            rospy.loginfo('Wcomm - Phrase TENSORFLOW detected !!')
            self.word = data.data

        '''
        if data.data == "get me the beer":
            rospy.loginfo('Wcomm - Phrase SAY HELLO detected !!')
            self.word = data.data


        if data.data == 'go foward':
            rospy.loginfo('Wcomm - Phrase GO FOWARD detected !!')
            self.word = data.data

        if data.data == 'go backward':
            rospy.loginfo('Wcomm - Phrase GO BACKWARD detected !!')
            self.word = data.data

        if data.data == 'rotate left':
            rospy.loginfo('Wcomm - Phrase ROTATE LEFT detected !!')
            self.word = data.data

        if data.data == 'rotate right':
            rospy.loginfo('Wcomm - Phrase ROTATE RIGHT detected !!')
            self.word = data.data

        if data.data == "sarah":
            rospy.loginfo('Wcomm - Keyword SARAH detected !!')
            self.word = data.data
        '''

    def SayX(self, ToSay_str):
        rospy.loginfo(ToSay_str)
        self.pub.publish(ToSay_str)


# define state WaitingConfirmation
class WaitingConfirmation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Timeout', 'Yes', 'No', 'Stop', 'Sarah'],
                             input_keys=['WConf_lastWord_in',
                                         'WConf_lastState_in',
                                         'WConf_lastCommand_in'],
                             output_keys=['WConf_lastWord_out',
                                          'WConf_lastState_out',
                                          'WConf_lastCommand_out'])
        self.word = ""
        self.state = "WaitingConfirmation"
        self.lastWord = ''
        rospy.Subscriber("/recognizer_1/output", String, self.callback)
        self.pub = rospy.Publisher('sara_tts', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo('Executing state WaitingConfirmation')
        userdata.WConf_lastState_out = self.state
        self.lastWord = userdata.WConf_lastWord_in
        self.SayX('Avez-vous dit ')
        self.SayX(self.lastWord)
        self.word = ""
        timeout = time.time() + 15  # 15 sec
        while True:
            if self.word == 'stop':
                userdata.WConf_lastWord_out = self.word
                return 'Stop'

            if self.word == 'no':
                userdata.WConf_lastWord_out = self.word
                return 'No'

            if self.word == 'yes':
                userdata.WConf_lastWord_out = self.word
                return 'Yes'

            if time.time() > timeout:
                return 'Timeout'

    def callback(self, data):
        if data.data == "stop":
            rospy.loginfo('Keyword STOP detected !!')
            self.word = data.data

        if data.data == 'yes':
            rospy.loginfo('Keyword YES detected !!')
            self.word = data.data

        if data.data == 'no':
            rospy.loginfo('Keyword NO detected !!')
            self.word = data.data
        '''
        if data.data == "sarah":
            rospy.loginfo('Keyword SARAH detected !!')
            self.word = data.data
        '''

    def SayX(self, ToSay_str):
        rospy.loginfo(ToSay_str)
        self.pub.publish(ToSay_str)


class Selector(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Idle', 'SpeechRecognition', 'Following', 'Tensorflow'],
                             input_keys=['Sel_lastWord_in',
                                         'Sel_lastState_in',
                                         'Sel_lastCommand_in'],
                             output_keys=['Sel_lastWord_out',
                                          'Sel_lastState_out'])
        self.state = "Selector"
        rospy.Subscriber("/recognizer_1/output", String, self.callback, queue_size=1)

    def execute(self, userdata):
        if userdata.Sel_lastCommand_in == "follow me":
            return 'Following'
        if userdata.Sel_lastCommand_in == "question":
            return 'SpeechRecognition'
        if userdata.Sel_lastCommand_in == "what do you see":
            return 'Tensorflow'


    def callback(self, data):
        if data.data == "stop":
            rospy.loginfo('Idle - Keyword STOP detected !!')
            self.word = data.data

        if data.data == "sarah":
            rospy.loginfo('Idle - Keyword SARAH detected !!')
            self.word = data.data

    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        smach.State.request_preempt(self)
        rospy.logwarn("Preempted!")

    '''
    cc = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted'],
                           default_outcome='aborted',
                           input_keys=['cc_Comm_goal'],
                           output_keys=['cc_Comm_result'],
                           child_termination_cb=child_term_cb,
                           outcome_cb=out_cb)
    with cc:

        def comm_goal_cb(userdata, goal):
            goal.goal = userdata.Comm_goal
            return goal

        def comm_result_cb(userdata, status, result):
            if status == GoalStatus.SUCCEEDED:
                userdata.Comm_result = result
                return 'succeeded'
            elif status == GoalStatus.PREEMPTED:
                userdata.Comm_result = result
                return 'preempted'
            elif status == GoalStatus.ABORTED:
                userdata.Comm_result = result
                return 'aborted'
        '''
    ''' DESACTIVATED FOR NOW
        def Nav_goal_cb(userdata):
            goal = 'GoThere'
            return goal

        def Nav_result_cb(userdata, status, result):
            if status == GoalStatus.SUCCEEDED:
                userdata.Nav_output = result
                return 'succeeded'
        '''

    '''  TODO
        def Arm_goal_cb(userdata):
            goal = 'WaitForCommand'
            return goal

        def Arm_result_cb(userdata, status, result):
            if status == GoalStatus.SUCCEEDED:
                userdata.Comm_output = result
                return 'my_outcome'
        '''

    ''' TODO
        def Vis_goal_cb(userdata):j
            goal = 'WaitForCommand'
            return goal

        def Vis_result_cb(userdata, status, result):
            if status == GoalStatus.SUCCEEDED:
                userdata.Comm_output = result
                return 'my_outcome'
        '''
    '''
        smach.Concurrence.add('CommGoal',
                              SimpleActionState('SaraComm',
                                                CommAction,
                                                goal_cb=comm_goal_cb,
                                                input_keys=['Comm_goal'],
                                                result_cb=comm_result_cb,
                                                output_keys=['Comm_result']),
                              remapping={'Comm_goal': 'cc_Comm_goal',
                                         'Comm_result': 'cc_Comm_result'})
        '''
    ''' DESACTIVATED FOR NOW
        smach.Concurrence.add('NavGoal',
                               SimpleActionState('move_base',
                                                 MoveBaseAction,
                                                 goal_cb=Nav_goal_cb,
                                                 result_cb=Nav_result_cb),
                               remapping={'Nav_output': 'Nav_result'})
        '''
    ''' TODO
        smach.Concurrence.add('ArmGoal',
                               SimpleActionState('Arm',
                                                 CommAction,
                                                 goal_cb=Arm_goal_cb,
                                                 result_cb=Arm_result_cb),
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})
        '''
    ''' TODO
        smach.Concurrence.add('VisGoal',
                               SimpleActionState('SaraComm',
                                                 CommAction,
                                                 goal_cb=Vis_goal_cb,
                                                 result_cb=Vis_result_cb),
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})
        '''
    ''' DESACTIVATED FOR NOW
    sm_FetchItem = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    with sm_FetchItem:

        def sm_FetchItem_cb():
            return 'TODO'

        smach.StateMachine.add('LocateItem',
                               cc,
                               transitions={'succeeded': sm_FetchItem_cb,
                                            'preempted': sm_FetchItem_cb,
                                            'aborted': sm_FetchItem_cb},
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})

        smach.StateMachine.add('MoveToIt',
                               cc,
                               transitions={'succeeded': sm_FetchItem_cb,
                                            'preempted': sm_FetchItem_cb,
                                            'aborted': sm_FetchItem_cb},
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})

        smach.StateMachine.add('TakeIt',
                               cc,
                               transitions={'succeeded': sm_FetchItem_cb,
                                            'preempted': sm_FetchItem_cb,
                                            'aborted': sm_FetchItem_cb},
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})

        smach.StateMachine.add('ComeBack',
                               cc,
                               transitions={'succeeded': sm_FetchItem_cb,
                                            'preempted': sm_FetchItem_cb,
                                            'aborted': sm_FetchItem_cb},
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})

        smach.StateMachine.add('GiveIt',
                               cc,
                               transitions={'succeeded': sm_FetchItem_cb,
                                            'preempted': sm_FetchItem_cb,
                                            'aborted': sm_FetchItem_cb},
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})

        smach.StateMachine.add('DropIt',
                               cc,
                               transitions={'succeeded': sm_FetchItem_cb,
                                            'preempted': sm_FetchItem_cb,
                                            'aborted': sm_FetchItem_cb},
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})

        smach.StateMachine.add('ChangeAngle',
                               cc,
                               transitions={'succeeded': sm_FetchItem_cb,
                                            'preempted': sm_FetchItem_cb,
                                            'aborted': sm_FetchItem_cb},
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})
    # TODO
    sm_FollowMe = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with FollowMe:

    sm_GoTo = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with sm_GoTo:

    sm_FindPerson = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with sm_FindPerson:

    sm_LearnPerson = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with sm_LearnPerson:

    sm_LearnObject = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with sm_LearnObject:

    sm_Stop = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with sm_Stop:

    '''


class SupremePlanner(StateMachine):

    def __init__(self):
        super(SupremePlanner, self).__init__(outcomes=['sm_top_end'],
                                             output_keys=['result'])
        with self:
            smach.StateMachine.add('Idle', Idle(),
                                   transitions={'Sara': 'WaitingCommand',
                                                'Stop': 'Idle'},
                                   remapping={'Idle_lastWord_in': 'lastWord',
                                              'Idle_lastState_in': 'lastState',
                                              'Idle_lastWord_out': 'lastWord',
                                              'Idle_lastState_out': 'lastState'})

            smach.StateMachine.add('WaitingCommand', WaitingCommand(),
                                   transitions={'Stop': 'Idle',
                                                'Sarah': 'WaitingCommand',
                                                'Command': 'WaitingConfirmation',
                                                'Timeout': 'Idle'},
                                   remapping={'WComm_lastWord_in': 'lastWord',
                                              'WComm_lastState_in': 'lastState',
                                              'WComm_lastWord_out': 'lastWord',
                                              'WComm_lastState_out': 'lastState',
                                              'WComm_lastCommand_out': 'lastCommand'})

            smach.StateMachine.add('WaitingConfirmation', WaitingConfirmation(),
                                   transitions={'Timeout': 'Idle',
                                                'Yes': 'Selector',
                                                'No': 'Idle',
                                                'Stop': 'Idle',
                                                'Sarah': 'WaitingCommand'},
                                   remapping={'WConf_lastWord_in': 'lastWord',
                                              'WConf_lastState_in': 'lastState',
                                              'WConf_lastCommand_in': 'lastCommand',
                                              'WConf_lastWord_out': 'lastWord',
                                              'WConf_lastState_out': 'lastState',
                                              'WConf_lastCommand_out': 'lastCommand',})

            self.add('Selector', Selector(),
                     transitions={'Idle': 'Idle', 'SpeechRecognition': 'SpeechRecognition', 'Following': 'Following', 'Tensorflow': 'Tensorflow'},
	           remapping={'Sel_lastWord_in': 'lastWord',
	                      'Sel_lastState_in': 'lastState',
                              'Sel_lastCommand_in': 'lastCommand',
	                      'Sel_lastWord_out': 'lastWord',
	                      'Sel_lastState_out': 'lastState'})

            smach.StateMachine.add('SpeechRecognition', SpeechRecognition(),
                                   transitions={'success': 'Idle', 'aborted': 'Idle', 'preempted': 'Idle'})

            smach.StateMachine.add('Following', Following(),
                                   transitions={'success': 'Idle', 'aborted': 'Idle', 'preempted': 'Idle'})

            smach.StateMachine.add('Tensorflow', Tensorflow(),
                                   transitions={'success': 'Idle', 'aborted': 'Idle', 'preempted': 'Idle'})

        ''' DESACTIVATED FOR NOW
        smach.Statemachine.add('FetchItem',
                               sm_FetchItem,
                               transition={'succeeded': 'Idle',
                                           'preempted': 'Idle',
                                           'aborted': 'Idle'})

        smach.Statemachine.add('FollowMe',
                               sm_FollowMe,
                               transition={'succeeded': 'Idle',
                                           'preempted': 'Idle',
                                           'aborted': 'Idle'})

        smach.Statemachine.add('GoTo',
                               sm_GoTo,
                               transition={'succeeded': 'Idle',
                                           'preempted': 'Idle',
                                           'aborted': 'Idle'})

        smach.Statemachine.add('FindPerson',
                               sm_FindPerson,
                               transition={'succcceeded': 'Idle',
                                           'preempted': 'Idle',
                                           'aborted': 'Idle'})

        smach.Statemachine.add('LearnPerson',
                               sm_LearnPerson,
                               transition={'succeeded': 'Idle',
                                           'preempted': 'Idle',
                                           'aborted': 'Idle'})

        smach.Statemachine.add('LearnObject',
                               sm_LearnObject,
                               transition={'succeeded': 'Idle',
                                           'preempted': 'Idle',
                                           'aborted': 'Idle'})
        '''
        '''
        smach.StateMachine.add('Stop',
                               cc,
                               transitions={'succeeded': 'Idle',
                                            'preempted': 'Idle',
                                            'aborted': 'Idle'},
                               remapping={'cc_Comm_in': 'sm_top_Comm_out',
                                          'cc_Comm_out': 'sm_top_Comm_out'})
                                          '''


if __name__ == '__main__':
    rospy.init_node('SupremePlanner')
    SupPlan = SupremePlanner()
    sis = smach_ros.IntrospectionServer('SupremePlanner', SupPlan, '/SM_ROOT')
    sis.start()
    SupPlan.execute()
    # Create and start the introspection server

    while not rospy.is_shutdown():
        rospy.spin()
        # Request the container to preempt
        SupPlan.request_preempt()
