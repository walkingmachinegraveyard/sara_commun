#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import actionlib
import time
import threading
from smach_ros import SimpleActionState
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

RECOGNIZER_CALLBACK = None
RECOGNIZER_CALLBACK2 = None

def handleRecognizerMessage(msg):
    if RECOGNIZER_CALLBACK is not None:
        RECOGNIZER_CALLBACK(msg)

def handleRecognizerMessage2(msg):
    if RECOGNIZER_CALLBACK2 is not None:
        RECOGNIZER_CALLBACK2(msg)

# define state Idle
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Stop', 'Sarah'],
                             input_keys=['Idle_lastWord_in',
                                         'Idle_lastState_in'],
                             output_keys=['Idle_lastWord_out',
                                          'Idle_lastState_out',
                                          'Idle_lastCommand_out'])
        self.word = ""
        self.state = "Idle"
        self.pub = rospy.Publisher('SaraVoice', String, queue_size=1)
        self.sub = rospy.Subscriber("/recognizer_1/output", String, self.callback, queue_size=1)
        
    def execute(self, userdata):
        '''global RECOGNIZER_CALLBACK
        RECOGNIZER_CALLBACK = self.callback'''
        rospy.loginfo('Executing state Idle')
        rospy.loginfo('Idle - Waiting for keyword: SARAH')
        self.word = ""
        while True:
            if self.word == 'stop':
                userdata.Idle_lastWord_out = self.word
                userdata.Idle_lastState_out = self.state
                userdata.Idle_lastCommand_out = 'stop'
                self.sub.unregister()
                return 'Stop'
            if self.word == 'sarah':
                userdata.Idle_lastWord_out = self.word
                userdata.Idle_lastState_out = self.state
                self.sub.unregister()
                return 'Sarah'
            
    def callback(self,data): 
        if data.data == "stop":
            rospy.loginfo('Idle - Keyword STOP detected !!')
            self.word = data.data

        if data.data == "sarah":
            rospy.loginfo('Idle - Keyword SARAH detected !!')
            self.word = data.data

    def SayX(self, ToSay_str):
        rospy.loginfo(ToSay_str)
        self.pub.publish(ToSay_str)


# define state WaitingCommand
class WaitingCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Command','DoIt','Sarah','Stop','Timeout'],
                             input_keys=['WComm_lastWord_in',
                                         'WComm_lastState_in'],
                             output_keys=['WComm_lastWord_out',
                                          'WComm_lastState_out',
                                          'WComm_lastCommand_out'])
        self.word = ""
        self.state = "WaitingCommand"
        self.pub = rospy.Publisher('SaraVoice', String, queue_size=1)

    def execute(self, userdata):
        global RECOGNIZER_CALLBACK
        RECOGNIZER_CALLBACK = self.callback
        rospy.loginfo('Executing state WaitingCommand')
        userdata.WComm_lastState_out = self.state

        self.SayX('Yes master')
        self.word = ""
        timeout = time.time() + 15  # 15 sec
        while True:
            if self.word == 'stop':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'Stop'

            if self.word == 'sarah':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'Sarah'

            if self.word == 'say hello':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word  
                self.SayX('Hi. I am a assistance robot here to serve you. I am not totally fonctionnal for now, but soon i will be able to do the chores for you.')  
                return 'Timeout'

            if self.word == 'what do you see':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'DoIt'

            if self.word == 'get me the beer':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'Command'

            if self.word == 'be happy':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'DoIt'

            if self.word == 'be sad':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'DoIt'

            if self.word == 'follow me':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'Command'

            if self.word == 'go foward':
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
            '''
            if self.word == 'sarah':
                userdata.WComm_lastWord_out = self.word
                userdata.WComm_lastCommand_out = self.word
                return 'Sarah'
            '''
            if time.time() > timeout:
                userdata.WComm_lastState_out = self.state  
                return 'Timeout'

    def callback(self,data): 
        if data.data == "stop":
            rospy.loginfo('Wcomm - Keyword STOP detected !!')
            self.word = data.data

        if data.data == "get me the beer":
            rospy.loginfo('Wcomm - Phrase SAY HI detected !!')
            self.word = data.data

        if data.data == "what do you see":
            rospy.loginfo('Wcomm - Phrase WHAT DO YOU SEE detected !!')
            self.word = data.data

        if data.data == "follow me":
            rospy.loginfo('Wcomm - Phrase FOLLOW ME detected !!')
            self.word = data.data

        if data.data == "be happy":
            rospy.loginfo('Wcomm - Phrase BE HAPPY detected !!')
            self.word = data.data

        if data.data == "be sad":
            rospy.loginfo('Wcomm - Phrase BE SAD detected !!')
            self.word = data.data

        if data.data == "say hello":
            rospy.loginfo('Wcomm - Phrase SAY HI detected !!')
            self.word = data.data

        if data.data == 'go foward':
            rospy.loginfo('Wcomm - Phrase GO FORWARD detected !!')
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

    def SayX(self, ToSay_str):
        rospy.loginfo(ToSay_str)
        self.pub.publish(ToSay_str)


# define state WaitingConfirmation
class WaitingConfirmation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Timeout','Yes','No','Stop','Sarah'],
                             input_keys=['WConf_lastWord_in',
                                         'WConf_lastState_in'],
                             output_keys=['WConf_lastWord_out',
                                         'WConf_lastState_out'])
        self.word = ""
        self.state = "WaitingConfirmation"
        self.lastWord = ''
        self.pub = rospy.Publisher('SaraVoice', String, queue_size=10)
 
    def execute(self, userdata):
        global RECOGNIZER_CALLBACK
        RECOGNIZER_CALLBACK = self.callback
        rospy.loginfo('Executing state WaitingConfirmation')
        userdata.WConf_lastState_out = self.state 
        self.lastWord = userdata.WConf_lastWord_in
        self.SayX('Did you say')
        self.SayX(self.lastWord)
        self.word = ""
        timeout = time.time() + 15  # 15 sec
        while True:
            if self.word == 'stop':
                userdata.WConf_lastWord_out = self.word
                return 'Stop'

            if self.word == 'No':
                userdata.WConf_lastWord_out = self.word
                self.SayX('Sorry, can you repeat your command please')
                return 'No'

            if self.word == 'yes':
                userdata.WConf_lastWord_out = self.word
                self.SayX('I will now execute your order')
                return 'Yes'

            if time.time() > timeout:
                return 'Timeout'

    def callback(self,data): 
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

# define state DoSomething
class DoSomething(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Done'],
                             input_keys=['DSome_lastWord_in',
                                         'DSome_lastState_in',
                                         'DSome_lastCommand_in'],
                             output_keys=['DSome_lastWord_out',
                                          'DSome_lastState_out'])

        self.pub = rospy.Publisher('SaraVoice', String, queue_size=10)
        self.pubFollow = rospy.Publisher('voice_follow_flag', String, queue_size=10)
        #self.pubEmo = rospy.Publisher('control_emo', int, queue_size=10)

        self.nbObj = ''
        self.lastWord = ""
        self.lastState = ""
        self.lastCommand = ""
        self.state = "DoSomething"
        self.str_follow = "stop"
 
    def execute(self, userdata):
        global RECOGNIZER_CALLBACK2
        RECOGNIZER_CALLBACK2 = self.callback
        rospy.loginfo('-- Executing state DoSomething --')

        self.lastWord = userdata.DSome_lastWord_in
        self.lastState = userdata.DSome_lastState_in
        self.lastCommand = userdata.DSome_lastCommand_in
        userdata.DSome_lastState_out = self.state
        self.nbObj = ''
        
        '''if self.lastCommand == "be happy":
                userdata.DSome_lastState_out = self.state
                self.pubEmo.publish(1)
                return 'Done'

        if self.lastCommand == "be sad":
                userdata.DSome_lastState_out = self.state
                self.pubEmo.publish(2)
                return 'Done' '''

        if self.lastCommand == "what do you see":
                userdata.DSome_lastState_out = self.state
                rospy.loginfo('DSomm - Waiting for object table !!')

                while self.nbObj == '':
                    continue

                rospy.loginfo('DSomm - object table received !!')

                if self.nbObj == 0:
                    self.SayX('i see nothing')

                else:
                    phrase = "i see"
                    for i in range(0, self.nbObj):

                        if i == self.nbObj - 1 and self.nbObj >= 2 :
                            phrase += " and "

                        if self.objectTable[i*12] == 3:
                            phrase += " a can, "

                        if self.objectTable[i*12] == 4:
                            phrase += " some mexican food, "

                        if self.objectTable[i*12] == 5:
                            phrase += " a videogame controller, "

                    self.SayX(phrase)

                return 'Done'
        rospy.loginfo(self.lastCommand)
        if self.lastCommand == "follow me":
                rospy.loginfo('publishing follow')
                self.str_follow = 'follow'
                userdata.DSome_lastState_out = self.state
                self.pubFollow.publish(self.str_follow)
                return 'Done'

        if self.lastCommand == "stop":
                rospy.loginfo('publishing stop')
                self.SayX('stopping')
                self.str_follow = 'stop'
                userdata.DSome_lastState_out = self.state
                self.pubFollow.publish(self.str_follow)
                return 'Done'

        if self.lastCommand == "go foward":
                userdata.DSome_lastState_out = self.state
                return 'Done'

        if self.lastCommand == "go backward":
                userdata.DSome_lastState_out = self.state
                return 'Done'
  
        if self.lastCommand == "rotate left":
                userdata.DSome_lastState_out = self.state
                return 'Done'

        if self.lastCommand == "rotate right":
                userdata.DSome_lastState_out = self.state
                return 'Done'
        else:
            return 'Idle'

    def SayX(self, ToSay_str):
        rospy.loginfo(ToSay_str)
        self.pub.publish(ToSay_str)

    def callback(self,data):
        self.nbObj = len(data.data)/12
        self.objectTable = data.data


# main
def main():

    rospy.init_node('interpreter')

    rospy.Subscriber("/objects", Float32MultiArray, handleRecognizerMessage2, queue_size=1)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])

    with sm:
        # Add states to the container
        smach.StateMachine.add('Idle', Idle(),
                               transitions={'Sarah':'WaitingCommand',
                                            'Stop':'DoSomething'},
                               remapping={'Idle_lastWord_in':'lastWord',
                                          'Idle_lastState_in':'lastState',
                                          'Idle_lastWord_out':'lastWord',
                                          'Idle_lastState_out':'lastState',
                                          'Idle_lastCommand_out':'lastCommand'})

        smach.StateMachine.add('WaitingCommand', WaitingCommand(),
                               transitions={'Stop':'DoSomething',
                                            'DoIt':'DoSomething',
                                            'Sarah':'WaitingCommand',
                                            'Command':'WaitingConfirmation',
                                            'Timeout':'Idle'},
                               remapping={'WComm_lastWord_in':'lastWord',
                                          'WComm_lastState_in':'lastState',
                                          'WComm_lastWord_out':'lastWord',
                                          'WComm_lastState_out':'lastState',
                                          'WComm_lastCommand_out':'lastCommand'})

        smach.StateMachine.add('WaitingConfirmation', WaitingConfirmation(),
                               transitions={'Timeout':'Idle',
                                            'Yes':'DoSomething',
                                            'No':'Idle',
                                            'Stop':'DoSomething',
                                            'Sarah':'WaitingCommand'},
                               remapping={'WConf_lastWord_in':'lastWord',
                                          'WConf_lastState_in':'lastState',
                                          'WConf_lastWord_out':'lastWord',
                                          'WConf_lastState_out':'lastState'})

        smach.StateMachine.add('DoSomething', DoSomething(),
                               transitions={'Done': 'Idle'},
                               remapping={'DSome_lastWord_in': 'lastWord',
                                          'DSome_lastState_in': 'lastState',
                                          'DSome_lastCommand_in': 'lastCommand',
                                          'DSome_lastWord_out': 'lastWord',
                                          'DSome_lastState_out': 'lastState',
                                          'DSome_result_out': 'result'})

    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    smach_thread.join()

    rospy.spin()

    # Request the container to preempt
    sm.request_preempt()

if __name__ == '__main__':
    main()
