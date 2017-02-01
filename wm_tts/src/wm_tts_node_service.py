#!/usr/bin/env python

import pyttsx
import rospy
import time
import threading
from std_msgs.msg import String
from wm_tts.srv import *


class wm_tts():
    def __init__(self, node_name):
        # get parameters
        rospy.init_node(node_name)
        s = rospy.Service('wm_say', say_service, self.say)
        self.engine = pyttsx.init()
        self.engine_thread = threading.Thread(target=self.loop)
        self.engine_thread.start()
        rate = rospy.get_param('rate', 150)
        self.engine.setProperty('rate', rate)
        self.engine.setProperty('volume', 0.5)

    def loop(self):
        self.engine.startLoop(False)
        while not rospy.is_shutdown():
            self.engine.iterate()
            time.sleep(0.1)
        self.engine.endLoop()

    def say(self, req):
        rospy.loginfo(req.say.sentence)
        self.engine.say(req.say.sentence)
        while self.engine.isBusy():
            time.sleep(0.1)

        return True


if __name__ == '__main__':

    try:

        wm_tts('wm_tts_node')

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
