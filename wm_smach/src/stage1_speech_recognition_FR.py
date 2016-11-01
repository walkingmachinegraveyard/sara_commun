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
from std_msgs.msg import String, Float64, UInt8, Bool
from wm_interpreter.msg import *
from collections import Counter


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
        self.QUESTIONS.append(["What is your name",
                               "Do a little presentation",
                               "Who are the inventors of the C programming language",
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
        self.QUESTIONS.append([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        self.tts_pub = rospy.Publisher('sara_tts', String, queue_size=1, latch=True)
        self.face_cmd = rospy.Publisher('/face_mode', UInt8, queue_size=1, latch=True)
        self.sub = rospy.Subscriber("/recognizer_1/output", String, self.callback, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state WaitingQuestion')

        self.face_cmd.publish(3)

        timeout = time.time() + TIMEOUT_LENGTH  # 10 sec
        while True:
            if max(self.QUESTIONS[1]) > 70:
                userdata.WQ_question_out = self.QUESTIONS[0][self.QUESTIONS[1].index(max(self.QUESTIONS[1]))]
                for idx in range(len(self.QUESTIONS[1])):
                    self.QUESTIONS[1][idx] = 0
                return 'Question'
            '''else:
                if len(self.RecoString) < 2:
                    return 'NotUnderstood' '''

            '''if time.time() > timeout:
                return 'Timeout' '''

    def callback(self, data):
        self.RecoString = data.data.split()
        for idx in range(len(self.QUESTIONS[1])):
            self.QUESTIONS[1][idx] = 0

        for RecoWord in self.RecoString:
            for idx in range(len(self.QUESTIONS[1])):
                if self.QUESTIONS[0][idx].lower().find(RecoWord) != -1:
                    self.QUESTIONS[1][idx] += 1

        for idx in range(len(self.QUESTIONS[1])):
            self.QUESTIONS[1][idx] = self.QUESTIONS[1][idx]*100/len(self.QUESTIONS[0][idx].split())


    def SayX(self, ToSay_str):
        rospy.loginfo(ToSay_str)
        self.tts_pub.publish(ToSay_str)

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

        self.ANSWERS = {"What is your name":"Mon nom est Sara, ce qui signifie Systeme dassistance robotiser autonome",
                               "Do a little presentation":"Je suis un robot dassistance robotiser autonome. Jai eter concu par le club Walking Machine de ler-cole de technologie superieure specialement pour la comper-tition Robocup at Home.",
                               "Who are the inventors of the C programming language": "Les inventeur du language de programmation C sont Ken Thompson et Dennis Ritchie",
                               "Who is the inventor of the Python programming language": "Linventeur du language de programation python est Guido van Rossum",
                               "Which robot was the star in the movie Wall-E": "Le robot qui est lacteur principale dans le film Wall-E est Wall-E",
                               "Where does the term computer bug come from": "Le terme bogue informatique vient dun papillon de nuit coince dans un relais",
                               "What is the name of the round robot in the new Star Wars movie": "Le nom du petit robot rond dans le nouveau film de Star Wars est B B 8",
                               "How many curry sausages are eaten in Germany each year": "Environ 800 million currywurst par anner",
                               "Who is president of the galaxy in The Hitchhiker Guide to the Galaxy": "Le president de la galaxie dans le film Le Guide du voyageur galactique est Zaphod Beeblebrox",
                               "Which robot is the love interest in Wall-E": "Le robot companion de Wall-E est Eve",
                               "Which company makes ASIMO": "La compagnie qui fabrique ASIMO est Honda",
                               "What company makes Big Dog": "La compagnie qui fabrique Big Dog est Boston Dynamics",
                               "What is the funny clumsy character of the Star Wars prequels": "Le personnage drole mais maladroit des prelude de Star Wars est Jar-Jar Binks",
                               "How many people live in the Germany": "Il y a 80 millions dhabitant en Allemagne ",
                               "What are the colours of the German flag": "Les couleurs du drapeau de lAllemagne sont rouge, noir et jaune",
                               "What city is the capital of the Germany": "La capital de lAllemagne est Berlin",
                               "How many arms do you have": "Jai seulement un bras pour le moment. Veuillez me le redemander lannnee prochain",
                               "What is the heaviest element": "Lelement le plus lourd est le plutonium lorsquil est mesure par la masse de lelement mais lOsmium est plus dense",
                               "What did Alan Turing create": "Alan Turing a cree plusieurs choses comme les machines de Turing et le test de Turing",
                               "Who is the helicopter pilot in the A-Team": "Le pilote dhelicoptere dans A-Team est le capitaine Howling Mad Murdock",
                               "What Apollo was the last to land on the moon": "Le dernier a avoir atteris sur la lune etait Apollo 17",
                               "Who was the last man to step on the moon": "Le dernier homme a avoir marcher sur la lune etait Gene Cernan",
                               "In which county is the play of Hamlet set": "Il etait au Danemark",
                               "What are names of Donald Duck nephews": "The nom des neveux de Donald Duck etaient Huey Dewey et Louie Duck",
                               "How many metres are in a mile": "Il y a environ 1609 metres dans un mile",
                               "Name a dragon in The Lord of the Rings": "Le nom du dragon dans le Seigneur des anneaux etait Smaug",
                               "Who is the Chancellor of Germany": "La chancelliere de lAllemagne est Angela Merkel",
                               "Who developed the first industrial robot": "Le premier a developper un robot industriel etait le physicien americain Joseph Engelberg. Il est aussi considere comme le pere de la robotique.",
                               "What's the difference between a cyborg and an android": "Les cyborgs sont des etres biologiques avec des ameliorations electromecaniques. Les androids sont des robots avec une apparence humaine.",
                               "Do you know any cyborg": "Le professeur Kevin Warwick. Il a implemente un circuit dans son avant-bras gauche.",
                                "In which city is this year's RoboCup hosted": "La Robocup 2016 etait a Leipzig en Allemagne",
                                "Which city hosted last year's RoboCup": "La robocup 2015 etait a Heifei en Chine.",
                                "In which city will next year's RoboCup be hosted": "Robocup 2017 sera a Nagoya au Japon.",
                                "Name the main rivers surrounding Leipzig": "La Parthe Pleisse et la White Elster",
                                "Where is the zoo of this city located": "Le zoo est situe pres de la gare centrale.",
                                "Where did the peaceful revolution of 1989 start": "La revolution tranquille commenca le 4 septembre 1989 a Leipzig a la leglise Saint Nicholas.",
                                "Where is the world's oldest trade fair hosted": "La Foire de Leipzig est la plus ancienne du monde",
                                "Where is one of the world's largest dark music festivals hosted": "La ville de Leipzig accueille lun des plus grand festival de musique gothique du monde",
                                "Where is Europe's oldest continuous coffee shop hosted": "Le plus ancien cafe deurope ce trouve a Leipzig",
                                "Name one of the greatest German composers": "Jean Sebastien Bach est le plus grand compositeur dAllemagne",
                                "Where is Johann Sebastian Bach buried": "La sepulture de Jean Sebastien Bach se trouve a leglise Saint Thomas a  Leipzig",
                                "Do you have dreams": "Je reve de moutons electriques.",
                                "Hey what's up": "Comment le saurai-je?",
                                "There are seven days in a week. True or false": "Cest vrais, il y a bel et bien sept jours dans une semaine.",
                                "There are eleven days in a week. True or false": "Cest faux, il y a plutot sept jours dans une semaine.",
                                "January has 31 days. True or false": "Cest vrai, le mois de Janvier compte 31 jours.",
                                "January has 28 days. True or false": "Faux, Janvier contient 31 jours, pas 28",
                                "February has 28 days. True or false": "Vrai, sauf dans une annee bissextile qui en contient 29",
                                "February has 31 days. True or false": "Faux, Fevrier a soit 28 jours, ou 29 selon lannee.",
                                "What city are you from": "Je viens de Mont-rer al",
                                "Who used first the word Robot": "Le mot robot fut utilise pour la premiere fois par lecrivain tcheque Karel Capek",
                                "What origin has the word Robot": "Il provient du mot tcheque Robota qui signifie travail force ou esclavage"}

        self.tts_pub = rospy.Publisher('sara_tts', String, queue_size=1, latch=True)
 
    def execute(self, userdata):
        rospy.loginfo('-- Executing state WaitingConfirmation --')
        self.SayX(self.ANSWERS[userdata.AQ_question_in])

        return 'Done'

    def SayX(self, ToSay_str):
        rospy.loginfo(ToSay_str)
        self.tts_pub.publish(ToSay_str)

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
        rospy.sleep(5)
        return 'Done'

    def SayX(self, ToSay_str):
        rospy.loginfo(ToSay_str)
        self.tts_pub.publish(ToSay_str)

    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        smach.State.request_preempt(self)
        rospy.logwarn("Preempted!")

# main
def main():
    rospy.init_node('interpreter')
    rospy.sleep(5)
    tts_pub = rospy.Publisher('sara_tts', String, queue_size=1, latch=True)
    neck_pub = rospy.Publisher('neckHead_controller/command', Float64, queue_size=1, latch=True)

    neck_cmd = Float64()
    neck_cmd.data = 0
    neck_pub.publish(neck_cmd)

    tts_pub.publish("Bonjour, je suis maintenant prete a repondre a vos questions")

    outcomes = ""

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'aborted', 'preempted'],
                            output_keys=[])

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

