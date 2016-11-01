#!/usr/bin/env python

import rospy
import smach
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String, Float64, Bool, UInt8
from geometry_msgs.msg import PoseStamped
from wm_people_follower.srv import peopleFollower, peopleFollowerRequest, peopleFollowerResponse
from tf2_ros import Buffer, TransformListener
from smach import StateMachine
import wm_supervisor.srv
import threading
from math import sqrt

GREEN_FACE = 3
YELLOW_FACE = 4
RED_FACE = 5


class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_done'])
        self.tts_pub = rospy.Publisher('sara_tts', String, queue_size=1, latch=True)
        self.neck_pub = rospy.Publisher('neckHead_controller/command', Float64, queue_size=1, latch=True)
        self.face_cmd = rospy.Publisher('/face_mode', UInt8, queue_size=1, latch=True)

    def execute(self, ud):

        self.face_cmd.publish(GREEN_FACE)

        neck_cmd = Float64()

        neck_cmd.data = -2.0
        self.neck_pub.publish(neck_cmd)
        rospy.sleep(rospy.Duration(2))
        neck_cmd.data = 0.0
        self.neck_pub.publish(neck_cmd)

        tts_msg = String()
        tts_msg.data = "I am ready to begin the following"
        self.tts_pub.publish(tts_msg)

        return 'init_done'


class TellInstructions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['target_locked'])

        # TODO
        self.tts_pub = rospy.Publisher('sara_tts', String, queue_size=1, latch=True)
        self.people_follower_srv = rospy.ServiceProxy('wm_people_follow', peopleFollower)

    def execute(self, ud):
        rospy.logdebug("Entered 'TELL_INSTRUCTIONS' state.")

        # TODO

        tts_msg = String()
        tts_msg.data = "I will follow you once I am ready."
        self.tts_pub.publish(tts_msg)
        tts_msg.data = "Please stand still, approximately 1 meter in front of me, facing me, while I memorize your you."
        self.tts_pub.publish(tts_msg)
        rospy.sleep(5.0)

        loop_again = True

        while loop_again:
            try:
                res = self.people_follower_srv.call(request=peopleFollowerRequest.ACQUIRE_TARGET)
                if res.response == peopleFollowerResponse.SUCCESS:
                    loop_again = False
                else:
                    tts_msg.data = "I was not able to get your features. Please stand still, approximately 1 meter in front of me, facing me."
                    self.tts_pub.publish(tts_msg)

            except rospy.ServiceException:
                rospy.sleep(rospy.Duration(8))
                pass

        tts_msg.data = "When you want me to stop following you, say 'SARA stop'"
        self.tts_pub.publish(tts_msg)
        tts_msg.data = "You must start your instructions by calling my name."
        self.tts_pub.publish(tts_msg)
        tts_msg.data = "I am now ready to follow you."
        self.tts_pub.publish(tts_msg)

        try:
            self.people_follower_srv.call(request=peopleFollowerRequest.START_FOLLOWING)
        except rospy.ServiceException:
            pass

        return 'target_locked'


class MonitorFollowing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop_following'])
        self.audio_input = rospy.Subscriber('recognizer1/output', String, self.audio_cb)
        self.tts_pub = rospy.Publisher('sara_tts', String, queue_size=1, latch=True)
        self.people_follower_srv = rospy.ServiceProxy('wm_people_follow', peopleFollower)

        self.mutex = threading.Lock()
        self.stop_following = False

    def audio_cb(self, msg):

        self.mutex.acquire()

        if msg.data.lower().find('sara') != -1 or msg.data.lower().find('sarah') != -1:
            if msg.data.find('stop') != -1 and msg.data.find('follow') != -1:
                self.stop_following = True

        self.mutex.release()

        return

    def execute(self, ud):
        rospy.logdebug("Entered 'MONITOR_FOLLOWING' state.")

        while True:
            self.mutex.acquire()

            if self.stop_following:
                self.people_follower_srv.call(request=peopleFollowerRequest.STOP_FOLLOWING)
                self.mutex.release()
                break

            self.mutex.release()
            rospy.sleep(rospy.Duration(1))

        return 'stop_following'


class AskContinueFollowing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_following', 'success'])
        self.audio_input = rospy.Subscriber('recognizer1/output', String, self.audio_cb)
        self.tts_pub = rospy.Publisher('sara_tts', String, queue_size=1, latch=True)
        self.people_follower_srv = rospy.ServiceProxy('wm_people_follow', peopleFollower)
        self.face_cmd = rospy.Publisher('/face_mode', UInt8, queue_size=1, latch=True)

        self.mutex = threading.Lock()

        self.question_asked = False

        self.confirmation_received = False
        self.continue_following = False

    def audio_cb(self, msg):

        self.mutex.acquire()

        if self.question_asked:
            if msg.data.lower.find('yes'):
                self.confirmation_received = True
                self.continue_following = True
            elif msg.data.lower.find('no'):
                self.confirmation_received = True
                self.continue_following = False
            else:
                tts_msg = String()
                tts_msg.data = "I am sorry. I did not understand. Can you please repeat?"
                self.face_cmd.publish(YELLOW_FACE)
                self.tts_pub.publish(tts_msg)

        self.mutex.release()

        return

    def execute(self, ud):

        tts_msg = String()
        tts_msg.data = 'Do you want me to continue following you?'
        self.tts_pub.publish(tts_msg)

        self.question_asked = True

        return_param = ''

        while True:
            self.mutex.acquire()
            if self.confirmation_received:
                if self.continue_following:
                    tts_msg.data = "I will continue following you."
                    self.tts_pub.publish(tts_msg)
                    try:
                        self.people_follower_srv.call(request=peopleFollowerRequest.START_FOLLOWING)
                    except rospy.ServiceException:
                        pass
                    return_param = 'continue_following'
                else:
                    tts_msg.data = "I will stop following you."
                    self.tts_pub.publish(tts_msg)
                    return_param = 'success'

                self.mutex.release()
                break
            self.mutex.release()

        return return_param


class Following(StateMachine):

    def __init__(self):
        super(Following, self).__init__(outcomes=['success', 'aborted', 'preempted'],
                                        output_keys=['result'])

        starting_location = PoseStamped()
        starting_location.header.frame_id = 'map'
        starting_location.pose.position.x = 1.0
        starting_location.pose.position.y = 0.0
        starting_location.pose.position.z = 0.0
        starting_location.pose.orientation.x = 0.0
        starting_location.pose.orientation.y = 0.0
        starting_location.pose.orientation.z = 0.0
        starting_location.pose.orientation.w = 1.0

        checkpoint_1 = PoseStamped()
        checkpoint_1.header.frame_id = 'map'
        checkpoint_1.pose.position.x = 3.0
        checkpoint_1.pose.position.y = 0.0
        checkpoint_1.pose.position.z = 0.0
        checkpoint_1.pose.orientation.x = 0.0
        checkpoint_1.pose.orientation.y = 0.0
        checkpoint_1.pose.orientation.z = 0.0
        checkpoint_1.pose.orientation.w = 1.0

        checkpoint_2 = PoseStamped()
        checkpoint_2.header.frame_id = 'map'
        checkpoint_2.pose.position.x = 3.0
        checkpoint_2.pose.position.y = 1.0
        checkpoint_2.pose.position.z = 0.0
        checkpoint_2.pose.orientation.x = 0.0
        checkpoint_2.pose.orientation.y = 0.0
        checkpoint_2.pose.orientation.z = 0.0
        checkpoint_2.pose.orientation.w = 1.0

        checkpoint_3 = PoseStamped()
        checkpoint_3.header.frame_id = 'map'
        checkpoint_3.pose.position.x = 4.0
        checkpoint_3.pose.position.y = 1.0
        checkpoint_3.pose.position.z = 0.0
        checkpoint_3.pose.orientation.x = 0.0
        checkpoint_3.pose.orientation.y = 0.0
        checkpoint_3.pose.orientation.z = 0.0
        checkpoint_3.pose.orientation.w = 1.0

        goal_location = PoseStamped()
        goal_location.header.frame_id = 'map'
        goal_location.pose.position.x = 4.0
        goal_location.pose.position.y = 1.0
        goal_location.pose.position.z = 0.0
        goal_location.pose.orientation.x = 0.0
        goal_location.pose.orientation.y = 0.0
        goal_location.pose.orientation.z = 0.0
        goal_location.pose.orientation.w = 1.0

        self.userdata.target_wp = 0
        self.userdata.waypoints = [starting_location, checkpoint_1, checkpoint_2, checkpoint_3, goal_location]
        self.userdata.wp_str = ["starting location", "checkpoint 1", "checkpoint 2", "checkpoint 3"]

        with self:

            smach.StateMachine.add('INIT_STATE',
                                   InitState(),
                                   transitions={'init_done': 'TELL_INSTRUCTIONS'})

            smach.StateMachine.add('TELL_INSTRUCTIONS',
                                   TellInstructions(),
                                   transitions={'target_locked': 'MONITOR_FOLLOWING'})

            smach.StateMachine.add('MONITOR_FOLLOWING',
                                   MonitorFollowing(),
                                   transitions={'stop_following': 'ASK_CONTINUE_FOLLOWING'})

            smach.StateMachine.add('ASK_CONTINUE_FOLLOWING',
                                   AskContinueFollowing(),
                                   transitions={'continue_following': 'MONITOR_FOLLOWING',
                                                'success': 'success'})

if __name__ == '__main__':
    rospy.init_node('stage1_following_guiding_node')
    Following().execute()
    while not rospy.is_shutdown():
        rospy.spin()
