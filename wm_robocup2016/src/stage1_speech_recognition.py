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

TIMEOUT_LENGTH = 10

# define state WaitingQuestion
class WaitingQuestion(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['NotUnderstood', 'Question', 'Timeout'],
                             input_keys=[],
                             output_keys=['WQ_question_out'])

        self.RecoString = []
        self.state = "WaitingQuestion"
        self.QUESTIONS = []
        self.QUESTIONS.append(["Who are the inventors of the C programming language",
                               "Who is the inventor of the Python programming language",
                               "Which robot was the star in the movie Wall-E",
                               "Where does the term computer bug come from",
                               "What is the name of the round robot in the new Star Wars movie",
                               "How many curry sausages are eaten in Germany each year",
                               "Who is president of the galaxy in The Hitchhiker Guide to the Galaxy",
                               "Which robot is the love interest in Wall-E",
                               "Which company makes ASIMO",
                               "What company makes Big Dog",
                               "What is the funny clumsy character of the Star Wars prequels",
                               "How many people live in the Germany",
                               "What are the colours of the German flag",
                               "What city is the capital of the Germany",
                               "How many arms do you have",
                               "What is the heaviest element",
                               "what did Alan Turing create",
                               "Who is the helicopter pilot in the A-Team",
                               "What Apollo was the last to land on the moon",
                               "Who was the last man to step on the moon",
                               "In which county is the play of Hamlet set",
                               "What are names of Donald Duck nephews",
                               "How many metres are in a mile",
                               "Name a dragon in The Lord of the Rings",
                               "Who is the Chancellor of Germany",
                               "Who developed the first industrial robot",
                               "What's the difference between a cyborg and an android",
                               "Do you know any cyborg",
                               "In which city is this year's RoboCup hosted",
                               "Which city hosted last year's RoboCup",
                               "In which city will next year's RoboCup be hosted",
                               "Name the main rivers surrounding Leipzig",
                               "Where is the zoo of this city located",
                               "Where did the peaceful revolution of 1989 start",
                               "Where is the world's oldest trade fair hosted",
                               "Where is one of the world's largest dark music festivals hosted",
                               "Where is Europe's oldest continuous coffee shop hosted",
                               "Name one of the greatest German composers",
                               "Where is Johann Sebastian Bach buried",
                               "Do you have dreams",
                               "Hey what's up",
                               "There are seven days in a week. True or false",
                               "There are eleven days in a week. True or false",
                               "January has 31 days. True or false",
                               "January has 28 days. True or false",
                               "February has 28 days. True or false",
                               "February has 31 days. True or false",
                               "What city are you from",
                               "Who used first the word Robot",
                               "What origin has the word Robot"])
        self.QUESTIONS.append([0, 0])

        self.tts_pub = rospy.Publisher('sara_tts', String, queue_size=1, latch=True)
        self.face_cmd = rospy.Publisher('/face_mode', UInt8, queue_size=1, latch=True)
        self.sub = rospy.Subscriber("/recognizer_1/output", String, self.callback, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state WaitingQuestion')

        self.face_cmd.publish(3)

        userdata.WQ_question_out = self.state

        timeout = time.time() + TIMEOUT_LENGTH  # 10 sec
        while True:
            if min(self.QUESTIONS[1]) > 1:
                userdata.WQ_lastCommand_out = self.QUESTIONS[0][max(self.QUESTIONS[1])]
                return 'Question'

            if time.time() > timeout:
                return 'Timeout'

    def callback(self, data):
        self.RecoString = data.data.split()
        for idx in self.QUESTIONS[1]:
            self.QUESTION[1][idx] = 0

        for RecoWord in self.RecoString:
            for idx in self.QUESTIONS[1]:
                if self.QUESTIONS[idx].lower().find(RecoWord) != -1:
                    self.QUESTIONS[1][idx] += 1


    def SayX(self, ToSay_str):
        rospy.loginfo(ToSay_str)
        self.pub.publish(ToSay_str)

    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        smach.State.request_preempt(self)
        rospy.logwarn("Preempted!")


# define state AnswerQuestion
class AnswerQuestion(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Done'],
                             input_keys=['AQ_question_in'])

        self.ANSWERS = {"Who are the inventors of the C programming language": "The inventors of the C programming language is Ken Thompson and Dennis Ritchie",
                               "Who is the inventor of the Python programming language": "The inventors of the Python programming language Guido van Rossum",
                               "Which robot was the star in the movie Wall-E": "The robot star int he movie Wall-E was Wall-E",
                               "Where does the term computer bug come from": "The term computer bug come from a moth trapped in a relay",
                               "What is the name of the round robot in the new Star Wars movie": " The name of the round robot in the new Star Wars movie is B B 8",
                               "How many curry sausages are eaten in Germany each year": "About 800 million currywurst every year",
                               "Who is president of the galaxy in The Hitchhiker Guide to the Galaxy": "The president of the galaxy in The Hitchhiker's Guide to the Galaxy is Zaphod Beeblebrox",
                               "Which robot is the love interest in Wall-E": "The robot that is the love interest in Wall-E is Eve",
                               "Which company makes ASIMO": "The company that makes ASIMO is Honda",
                               "What company makes Big Dog": "The company that makes Big Dog is Boston Dynamics",
                               "What is the funny clumsy character of the Star Wars prequels": "The funny clumsy character of the Star Wars prequels is Jar-Jar Binks",
                               "How many people live in the Germany": "A little over 80 million people live in the Germany",
                               "What are the colours of the German flag": "The colours of the German flag are black red and yellow",
                               "What city is the capital of the Germany": "The capital of the Germany is Berlin",
                               "How many arms do you have": "I only have one arm for now. Ask me again next year",
                               "What is the heaviest element": "the heaviest element is plutonium when measured by the mass of the element but Osmium is densest",
                               "what did Alan Turing create": "Alan Turing created many things like Turing machines and the Turing test",
                               "Who is the helicopter pilot in the A-Team": "The helicopter pilot in the A-Team is Captain Howling Mad Murdock",
                               "What Apollo was the last to land on the moon": "The last to land on the moon was Apollo 17",
                               "Who was the last man to step on the moon": "The last man to step on the moon was Gene Cernan",
                               "In which county is the play of Hamlet set": "The play of Hamlet set is in Denmark",
                               "What are names of Donald Duck nephews": "The names of Donald Duck's nephews is Huey Dewey and Louie Duck",
                               "How many metres are in a mile": "There is about 1609 metres metres are in a mile",
                               "Name a dragon in The Lord of the Rings": "A dragon name in The Lord of the Rings is Smaug",
                               "Who is the Chancellor of Germany": "The Chancellor of Germany is Angela Merkel",
                               "Who developed the first industrial robot": "The first to develope a industrial robot are the American physicist Joseph Engelberg. He is also considered the father of robotics.",
                               "What's the difference between a cyborg and an android": "The difference between a cyborg and an android",
                               "Do you know any cyborg": "Professor Kevin Warwick. He implanted a chip in in his left arm to remotely operate doors an artificial hand and an electronic wheelchair",
                                "In which city is this year's RoboCup hosted": "Robocup 2016 is hosted in Leipzig Germany",
                                "Which city hosted last year's RoboCup": "robocup 2015 was hosted in Hefei China",
                                "In which city will next year's RoboCup be hosted": "robocup 2017 will be in Nagoya in Japan",
                                "Name the main rivers surrounding Leipzig": "he Parthe Pleisse and the White Elster",
                                "Where is the zoo of this city located": "the zoo is located Near the central station",
                                "Where did the peaceful revolution of 1989 start": "The peaceful revolution started in September 4 1989 in Leipzig at the Saint Nicholas Church",
                                "Where is the worlds oldest trade fair hosted": "The worlds oldest trade fair is in Leipzig",
                                "Where is one of the worlds largest dark music festivals hosted": "Leipzig hosts one of the worlds largest dark music festivals",
                                "Where is Europes oldest continuous coffee shop hosted": "Europes oldest continuous coffee shop is in Leipzig",
                                "Name one of the greatest German composers": "Johann Sebastian Bach",
                                "Where is Johann Sebastian Bach buried": "Johann Sebastian Bach is buried in Saint Thomas Church here in Leipzig",
                                "Do you have dreams": "I dream of Electric Sheeps",
                                "Hey what's up": "I don't know since I've never been there",
                                "There are seven days in a week. True or false": "True there are seven days in a week",
                                "There are eleven days in a week. True or false": "False there are seven days in a week not eleven",
                                "January has 31 days. True or false": "True January has 31 days",
                                "January has 28 days. True or false": "False January has 31 days not 28",
                                "February has 28 days. True or false": "True but in leap-years has 29",
                                "February has 31 days. True or false": "False February has either 28 or 29 days. Depend on the year",
                                "What city are you from": "I am from Montreal",
                                "Who used first the word Robot": "The word robot was first used by tchek writer Karel Capek",
                                "What origin has the word Robot": "The tchek word robota that means forced work or labour"}

        self.tts_pub = rospy.Publisher('sara_tts', String, queue_size=1, latch=True)
 
    def execute(self, userdata):
        rospy.loginfo('-- Executing state WaitingConfirmation --')

        self.SayX(self.ANSWERS(userdata.AQ_question_in))

    def SayX(self, ToSay_str):
        rospy.loginfo(ToSay_str)
        self.pub.publish(ToSay_str)

    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        smach.State.request_preempt(self)
        rospy.logwarn("Preempted!")

# define state AskToRepeat
class AskToRepeat(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Done'])

        self.tts_pub = rospy.Publisher('sara_tts', String, queue_size=1, latch=True)

    def execute(self, userdata):
        rospy.loginfo('-- Executing state AskRepeat --')

        self.SayX("Can you repeat the question please?")

    def SayX(self, ToSay_str):
        rospy.loginfo(ToSay_str)
        self.pub.publish(ToSay_str)

    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        smach.State.request_preempt(self)
        rospy.logwarn("Preempted!")

# main
def main():

    outcomes = ""

    rospy.init_node('interpreter')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'aborted', 'preempted'],
                            output_keys=['result'])

    with sm:
        # Add states to the container
        smach.StateMachine.add('WaitingQuestion', WaitingQuestion(),
                               transitions={'Question': 'AnswerQuestion',
                                            'NotUnderstood': 'AskToRepeat',
                                            'Timeout': 'WaitingQuestion'},
                               remapping={'WQ_question_out': 'question'})

        smach.StateMachine.add('AnswerQuestion', AnswerQuestion(),
                               transitions={'Done': 'WaitingQuestion'},
                               remapping={'AQ_question_in': 'question'})

        smach.StateMachine.add('AskToRepeat', AskToRepeat(),
                               transitions={'Done': 'WaitingQuestion'},
)




    '''sis = smach_ros.IntrospectionServer('server_name', asw.wrapped_container, '/ASW_ROOT')'''

    # Execute SMACH plan
    sm.execute()

    rospy.spin()

    # Request the container to preempt
    sm.request_preempt()

if __name__ == '__main__':
    main()
