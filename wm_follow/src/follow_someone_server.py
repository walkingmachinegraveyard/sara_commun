#!/usr/bin/env python

from wm_follow.srv import *
import rospy
from math import *
import tf.transformations
import time
import actionlib
from people_msgs.msg import *
from move_base_msgs.msg import *

class Person:

    def __init__(self):
        self.newX = 0
        self.newY = 0
        self.lastX = 0
        self.lastY = 0

class Follower:

    def __init__(self):
        self.person = Person()

        self.state = False

        #Action client move_base
        self.moveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # init le goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'base_link'

    def sendGoal(self):
        self.goal.target_pose.header.frame_id = 'base_link'
        self.moveBaseClient.send_goal(self.goal)


def callbackPeople(data):

    if follower.state == True:
		
        shortestDistance = -1

        shortestX = 0
        shortestY = 0

        #rospy.loginfo(str(len(data.people)))

        for people in data.people:
            
            x = people.pos.x
            y = people.pos.y

            distance = hypot(x, y)

            if shortestDistance == -1:
                shortestDistance = distance
                shortestX = x
                shortestY = y
            else :
                if distance < shortestDistance:
                    shortestDistance = distance
                    shortestX = x
                    shortestY = y

        follower.person.lastX = follower.person.newX
        follower.person.newX = shortestX
        follower.person.newY = shortestY

def handle_FollowSomeone(req):
    if req.state == req.START_FOLLOWING:
        follower.state = True
        rospy.logwarn("SARA : START FOLLOWING")

    elif req.state == req.STOP_FOLLOWING:
        follower.state = False
        rospy.logwarn("SARA : STOP FOLLOWING")

def createNavGoal(x, y):
    rospy.loginfo("Position personne : (" + str(x) + " " + str(y) + ")")

    # calcule angle par rapport a la position de la personne
    angleRad = atan2(y, x)
    rospy.loginfo("angleRad : " + str(angleRad))

    # calcule la distance par rapport a la personne
    distancePersonne = hypot(x, y)
    rospy.loginfo("distance : " + str(distancePersonne))

    # calcul distance a parcourir, si distance < 1 : 0, sinon
    if distancePersonne > 1.5:
        move = True
        distanceBouger = distancePersonne - 1.5

    else:
        distanceBouger = 0

    rospy.loginfo("distance a parcourir : " + str(distanceBouger))

    # nouvelle position a atteindre
    x = cos(angleRad) * distanceBouger
    y = sin(angleRad) * distanceBouger

    rospy.loginfo("x : " + str(x))
    rospy.loginfo("y : " + str(y))

    # calcul l'angle de euler(roll, pitch, yaw) a quaternion
    quaternion = tf.transformations.quaternion_from_euler(0, 0, angleRad)

    goal = MoveBaseGoal()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    return goal


def startFollow():
    while not rospy.is_shutdown():
        time.sleep(0.5)
		
        if follower.state:
            follower.goal = createNavGoal(follower.person.newX, follower.person.newY)
            follower.sendGoal()


def followSomeone():
    rospy.init_node('followSomeoneServer')
    rospy.Service('follow_someone', setState, handle_FollowSomeone)
    rospy.loginfo("Ready to follow someone")
    peopleSub = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, callbackPeople)
    startFollow()

    # Subscriber detection personnes
    

    rospy.spin()


follower = Follower()

if __name__ == "__main__":
    followSomeone()
    
