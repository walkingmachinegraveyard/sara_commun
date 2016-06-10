#!/usr/bin/env python

# Get espeak at http://espeak.sourceforge.net/

import rospy
from std_msgs.msg import String
from subprocess import check_call, CalledProcessError


class wm_tts:

    def __init__(self):
        # get parameters
        self.engine = rospy.get_param('/wm_tts_node/tts_engine', "espeak")
        str(self.engine)
        # voice file must be in /espeak-data/voices directory
        self.voice = rospy.get_param('/wm_tts_node/voice_file', "en+f2")
        str(self.voice)
        # words/minute (80...500), default value = 175
        self.speech_rate = rospy.get_param('/wm_tts_node/speech_rate', 140)
        # amplitude (0...200), default value = 100
        self.speech_volume = rospy.get_param('/wm_tts_node/speech_volume', 100)
	# pitch (0...99), default value = 50
	self.pitch = rospy.get_param('/wm_tts_node/pitch',60)
	

        self.sub = rospy.Subscriber('sara_tts', String, self.callback)

    def callback(self, msg):

        try:
            # str() typecasting is necessary for check_call to work
            check_call([str(self.engine), "-s", str(self.speech_rate), "-v", str(self.voice),
                        "-a", str(self.speech_volume), "-p", str(self.pitch), str(msg.data)])
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











