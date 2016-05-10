#!/usr/bin/env python

import pyttsx
import rospy
import time
import threading
from std_msgs.msg import String


class wm_tts():
    def __init__(self,node_name):
        # get parameters
        rospy.init_node(node_name)
        self.engine = pyttsx.init()
        self.engine_thread = threading.Thread(target=self.loop)
        self.engine_thread.start()
        rate = rospy.get_param('rate', 150)
        self.engine.setProperty('rate', rate)
        self.engine.setProperty('volume', 0.5)

        self.sub = rospy.Subscriber('SaraVoice', String, self.callback)

    def loop(self):
        self.engine.startLoop(False)
        while not rospy.is_shutdown():
            self.engine.iterate()
            time.sleep(0.1)
        self.engine.endLoop()

    def callback(self, msg):

        rospy.loginfo(msg.data)
        self.engine.say(msg.data)
        while self.engine.isBusy():
            time.sleep(0.1)



if __name__ == '__main__':
    
    try:

        wm_tts('wm_tts_node')

        rospy.spin()

    except rospy.ROSInterruptException:
        pass











