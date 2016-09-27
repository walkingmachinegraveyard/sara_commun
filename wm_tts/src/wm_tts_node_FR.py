#!/usr/bin/env python

# Get espeak at http://espeak.sourceforge.net/

import rospy
from std_msgs.msg import String
from subprocess import check_call, CalledProcessError
import os


class wm_tts:

    def __init__(self):
        self.sub = rospy.Subscriber('sara_tts', String, self.callback)

    def callback(self, msg):

        try:
            os.system("pico2wave -l=fr-FR -w=/tmp/test.wav " + '"'+str(msg.data)+'"')
            os.system("aplay /tmp/test.wav")
            os.system("rm /tmp/test.wav")
            rospy.loginfo('SARA said: %s', msg.data)

        except CalledProcessError:
            rospy.logwarn('Last subprocess call was not valid.')


if __name__ == '__main__':

    try:
        rospy.init_node('wm_tts_node')

        wm_tts()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass











