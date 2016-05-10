#!/usr/bin/env python

import rospy
from cob_perception_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Interpreter:
    def __init__(self):

        self.repub = rospy.Publisher("/geometry_msgs/PoseStamped", PoseStamped, queue_size=100)
        self.cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=100)

        self.s = rospy.Subscriber("/leg_tracker_measurements", PositionMeasurementArray, self.arraycallback)
        self.flw_cmd = rospy.Subscriber("/voice_follow_flag", String, self.flw_cmd_callback)

        self.min_y_ = -1.0
        self.max_y_ = 1.0
        self.min_x_ = -0.5
        self.max_x_ = 3.0
        self.max_z_ = 2.0
        self.stop_flag = True
        
        self.people = None
        self.geo = None
        self.vel_cmd = None
        self.person_to_follow = None

        self.DEBUG = False

    def flw_cmd_callback(self, string):
         rospy.loginfo(string.data)
         if string.data == "stop":
             rospy.loginfo(string.data)
             self.stop_flag = True
         elif string.data == "follow":
             rospy.loginfo(string.data)
             self.stop_flag = False
         elif self.DEBUG:
             self.stop_flag = False

    def arraycallback(self, msg):

        if self.stop_flag:
             #STOP MODE ACTIVATED, DO NOT RUN
            return

        if not msg.people:
            # NO PEOPLE DETECTED
            return

        self.people = sorted(msg.people, key=lambda person: person.reliability)  # sort is ascend
        self.person_to_follow = self.people[-1]  # take the more reliable

        print self.person_to_follow.reliability, [p.reliability for p in msg.people]

        self.geo = PoseStamped()
        self.vel_cmd = Twist()
        self.geo.pose.position = self.person_to_follow.pos
        self.geo.header = self.person_to_follow.header

        if (self.person_to_follow.pos.x > self.min_y_) and (self.person_to_follow.pos.y < self.max_y_) and \
                (self.person_to_follow.pos.x < self.max_x_) and (self.person_to_follow.pos.x > self.min_x_) and \
                (self.person_to_follow.pos.z < self.max_z_):
            rospy.loginfo("yolo")              # objective is within detection area
            # publish data to geometry PoseStamped
            # objective is within detection zone
            self.repub.publish(self.geo)


            # moves following a vector toward objective

            if self.geo.pose.position.x < 0.5:
                self.vel_cmd.linear.x = -0.5
                self.vel_cmd.linear.y = 0
                self.vel_cmd.linear.z = 0
                self.vel_cmd.angular.x = 0
                self.vel_cmd.angular.y = 0
                self.vel_cmd.angular.z = 0

            elif self.geo.pose.position.x < 1.0 and 0.5 > self.geo.pose.position.y >= -0.5:
                self.vel_cmd.linear.x = 0
                self.vel_cmd.linear.y = 0
                self.vel_cmd.linear.z = 0
                self.vel_cmd.angular.x = 0
                self.vel_cmd.angular.y = 0
                self.vel_cmd.angular.z = 0

            else:

                if 2.0 > self.geo.pose.position.y >= 1.0:
                    # turn left if objective is left
                    self.vel_cmd.linear.x = 0
                    self.vel_cmd.linear.y = 0
                    self.vel_cmd.linear.z = 0
                    self.vel_cmd.angular.x = 0
                    self.vel_cmd.angular.y = 0
                    self.vel_cmd.angular.z = 0.5
                    rospy.loginfo("Left")

                elif 1.0 > self.geo.pose.position.y > 0.2:
                    # turn left if objective is left
                    self.vel_cmd.linear.x = 0.5
                    self.vel_cmd.linear.y = 0 
                    self.vel_cmd.linear.z = 0
                    self.vel_cmd.angular.x = 0
                    self.vel_cmd.angular.y = 0
                    self.vel_cmd.angular.z = 0.5
                    rospy.loginfo("Front and Left")

                elif 0.2 > self.geo.pose.position.y > -0.2:
                    # goes straight if within straight cone
                    self.vel_cmd.linear.x = 0.75
                    self.vel_cmd.linear.y = 0
                    self.vel_cmd.linear.z = 0
                    self.vel_cmd.angular.x = 0
                    self.vel_cmd.angular.y = 0
                    self.vel_cmd.angular.z = 0
                    rospy.loginfo("Straight")

                elif -0.2 > self.geo.pose.position.y > -0.5:
                    # turn right if objective is right
                    self.vel_cmd.linear.x = 0.5
                    self.vel_cmd.linear.y = 0
                    self.vel_cmd.linear.z = 0
                    self.vel_cmd.angular.x = 0
                    self.vel_cmd.angular.y = 0
                    self.vel_cmd.angular.z = -0.5
                    rospy.loginfo("Front and Right")

                elif -0.5 > self.geo.pose.position.y >= -1.5:
                    # turn left if objective is left
                    self.vel_cmd.linear.x = 0
                    self.vel_cmd.linear.y = 0
                    self.vel_cmd.linear.z = 0
                    self.vel_cmd.angular.x = 0
                    self.vel_cmd.angular.y = 0
                    self.vel_cmd.angular.z = -0.5
                    rospy.loginfo("Left")



                else:
                    self.vel_cmd.linear.x = 0
                    self.vel_cmd.linear.y = 0
                    self.vel_cmd.linear.z = 0
                    self.vel_cmd.angular.x = 0
                    self.vel_cmd.angular.y = 0
                    self.vel_cmd.angular.z = 0
                self.cmd.publish(self.vel_cmd)


if __name__ == '__main__':
    rospy.init_node('position_interpreter_node', log_level=rospy.DEBUG)
    interpreter = Interpreter()
    rospy.spin()
