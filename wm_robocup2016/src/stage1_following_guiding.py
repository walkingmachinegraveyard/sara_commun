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
        tts_msg.data = "I am ready to begin the following and guiding test."
        self.tts_pub.publish(tts_msg)

        return 'init_done'


class WaitForStart(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_cmd_received'])
        self.start_button_sub = rospy.Subscriber('start_button_msg', Bool, self.start_button_cb, queue_size=4)
        self.voice_recognizer_sub = rospy.Subscriber('output', String, self.voice_recognizer_cb, queue_size=4)

        self.mutex = threading.Lock()
        self.proceed = False

    def voice_recognizer_cb(self, msg):

        self.mutex.acquire()

        if msg.data.lower().find('sara') != -1 or msg.data.lower().find('sarah') != -1:
            if msg.data.find('begin') != -1 or msg.data.find('start'):
                self.proceed = True

        self.mutex.release()

        return

    def start_button_cb(self, msg):

        self.mutex.acquire()

        if msg.data:
            self.proceed = True

        self.mutex.release()

        return

    def execute(self, ud):
        rospy.logdebug("Entered 'WAIT_FOR_START' state.")

        while True:
            self.mutex.acquire()

            if self.proceed:
                self.mutex.release()
                break

            self.mutex.release()
            rospy.sleep(rospy.Duration(1))

        return 'start_cmd_received'


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
        tts_msg.data = "Hello, my name is SARA. I will follow you to the next waypoint once I am ready."
        self.tts_pub.publish(tts_msg)
        tts_msg.data = "Please stand still, approximately 1 meter in front of me, facing me, while I memorize your features."
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

        tts_msg.data = "When you want me to stop following you, say 'SARA go back home'"
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
        smach.State.__init__(self, outcomes=['continue_following', 'ask_guiding'])
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
                    return_param = 'ask_guiding'

                self.mutex.release()
                break
            self.mutex.release()

        return return_param


class AskStartGuiding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_guiding', 'do_not_guide'])
        self.audio_input = rospy.Subscriber('recognizer1/output', String, self.audio_cb)
        self.tts_pub = rospy.Publisher('sara_tts', String, queue_size=1, latch=True)
        self.people_follower_srv = rospy.ServiceProxy('wm_people_follow', peopleFollower)
        self.face_cmd = rospy.Publisher('/face_mode', UInt8, queue_size=1, latch=True)

        self.mutex = threading.Lock()

        self.question_asked = False

        self.confirmation_received = False
        self.start_guiding = False

    def audio_cb(self, msg):

        self.mutex.acquire()

        if self.question_asked:
            if msg.data.lower.find('yes'):
                self.confirmation_received = True
                self.start_guiding = True
            elif msg.data.lower.find('no'):
                self.confirmation_received = True
                self.start_guiding = False
            else:
                tts_msg = String()
                tts_msg.data = "I am sorry. I did not understand. Can you please repeat?"
                self.tts_pub.publish(tts_msg)
                self.face_cmd.publish(YELLOW_FACE)

        self.mutex.release()

        return

    def execute(self, ud):

        tts_msg = String()
        tts_msg.data = 'Do you want me to start guiding you back to the arena?'
        self.tts_pub.publish(tts_msg)

        self.question_asked = True

        return_param = ''

        while True:
            self.mutex.acquire()
            if self.confirmation_received:
                if self.start_guiding:
                    tts_msg.data = "I will guide you back to the arena."
                    self.tts_pub.publish(tts_msg)
                    try:
                        self.people_follower_srv.call(request=peopleFollowerRequest.RELEASE_TARGET)
                    except rospy.ServiceException:
                        pass
                    return_param = 'start_guiding'
                else:
                    return_param = 'do_not_guide'

                self.mutex.release()
                break
            self.mutex.release()

        return return_param


class StartGuiding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_back'],
                             input_keys=['sg_waypoints'],
                             output_keys=['sg_target_wp'])
        self.tts_pub = rospy.Publisher('sara_tts', String, queue_size=1, latch=True)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.face_cmd = rospy.Publisher('/face_mode', UInt8, queue_size=1, latch=True)

    def execute(self, ud):

        tts_msg = String()

        rospy.sleep(rospy.Duration(5))

        self.face_cmd.publish(YELLOW_FACE)
        tts_msg.data = "I can not guide you back. I will go back to the stating location on my own."
        self.tts_pub.publish(tts_msg)

        tf_stamped = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))

        lst = []

        for i in range(len(ud.sg_waypoints)):
            distance = sqrt((tf_stamped.transform.translation.x - ud.sg_waypoints[i].pose.position.x)**2 +
                            (tf_stamped.transform.translation.y - ud.sg_waypoints[i].pose.position.y)**2)
            lst.append([distance, i])

        lst.sort()

        if lst[0][1] == 0:
            ud.sg_target_wp = 0
        else:
            ud.sg_target_wp = lst[0][1] - 1

        return 'go_back'


class AnnounceAction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['announcement_done'], input_keys=['aa_target_wp', 'aa_wp_str'])

        self.tts_pub = rospy.Publisher('sara_tts', String, queue_size=1, latch=True)

    def execute(self, ud):
        rospy.logdebug("Entered 'ANNOUNCE_ACTION' state.")
        tts_msg = String()
        tts_msg.data = "I am moving toward " + ud.aa_wp_str[ud.aa_target_wp] + "."
        self.tts_pub.publish(tts_msg)
        return 'announcement_done'


class MoveSupervisor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move_ok', 'move_estop'])
        self.status_service = rospy.ServiceProxy('robot_status', wm_supervisor.srv.robotStatus)
        self.face_cmd = rospy.Publisher('/face_mode', UInt8, queue_size=1, latch=True)

    def execute(self, ud):
        rospy.logdebug("Entered 'ROBOT_STATUS' state.")

        try:
            res = self.status_service()

        except rospy.ServiceException:
            self.face_cmd.publish(RED_FACE)
            return 'move_estop'

        if res.status == wm_supervisor.srv.robotStatusResponse.STATUS_OK:
            return 'move_ok'

        rospy.sleep(5.0)
        self.face_cmd.publish(YELLOW_FACE)

        return 'move_estop'


class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wp_reached', 'wp_not_reached', 'test_finished'],
                             input_keys=['move_waypoints', 'move_target_wp', 'move_wp_str'],
                             output_keys=['move_target_wp'])
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        self.tts_pub = rospy.Publisher('sara_tts', String, queue_size=1, latch=True)

    def execute(self, ud):

        self.move_base_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose = ud.move_waypoints[ud.move_target_wp]
        goal.target_pose.header.stamp = rospy.Time.now()

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

        status = self.move_base_client.get_state()

        tts_msg = String()

        if status == GoalStatus.SUCCEEDED:
            tts_msg.data = "I have reached " + ud.move_wp_str[ud.move_target_wp] + "."
            self.tts_pub.publish(tts_msg)

            if ud.move_target_wp == 0:
                return 'test_finished'
            else:
                ud.move_target_wp -= 1
                return 'succeeded'
        else:
            return 'wp_not_reached'


if __name__ == '__main__':

    rospy.init_node('stage1_following_guiding_node')
    sm = smach.StateMachine(outcomes=['test_succeeded'])

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

    sm.userdata.target_wp = 0
    sm.userdata.waypoints = [starting_location, checkpoint_1, checkpoint_2, checkpoint_3, goal_location]
    sm.userdata.wp_str = ["starting location", "checkpoint 1", "checkpoint 2", "checkpoint 3"]

    with sm:

        smach.StateMachine.add('INIT_STATE',
                               InitState(),
                               transitions={'init_done': 'WAIT_FOR_START'})

        smach.StateMachine.add('WAIT_FOR_START',
                               WaitForStart(),
                               transitions={'start_cmd_received': 'TELL_INSTRUCTIONS'})

        smach.StateMachine.add('TELL_INSTRUCTIONS',
                               TellInstructions(),
                               transitions={'target_locked': 'MONITOR_FOLLOWING'})

        smach.StateMachine.add('MONITOR_FOLLOWING',
                               MonitorFollowing(),
                               transitions={'stop_following': 'ASK_CONTINUE_FOLLOWING'})

        smach.StateMachine.add('ASK_CONTINUE_FOLLOWING',
                               AskContinueFollowing(),
                               transitions={'continue_following': 'MONITOR_FOLLOWING',
                                            'ask_guiding': 'ASK_START_GUIDING'})

        smach.StateMachine.add('ASK_START_GUIDING',
                               AskStartGuiding(),
                               transitions={'start_guiding': 'START_GUIDING',
                                            'do_not_guide': 'ASK_CONTINUE_FOLLOWING'})

        smach.StateMachine.add('START_GUIDING',
                               StartGuiding(),
                               transitions={'go_back': 'MOVE_SUPERVISOR'},
                               remapping={'sg_target_wp': 'target_wp',
                                          'sg_waypoints': 'waypoints'})

        smach.StateMachine.add('ANNOUNCE_ACTION',
                               AnnounceAction(),
                               transitions={'announcement_done': 'MOVE_SUPERVISOR'},
                               remapping={'aa_target_wp': 'target_wp',
                                          'aa_wp_str': 'wp_str'})

        smach.StateMachine.add('MOVE_SUPERVISOR',
                               MoveSupervisor(),
                               transitions={'move_ok': 'MOVE',
                                            'move_estop': 'MOVE_SUPERVISOR'})

        smach.StateMachine.add('MOVE',
                               Move(),
                               transitions={'wp_reached': 'ANNOUNCE_ACTION',
                                            'wp_not_reached': 'MOVE_SUPERVISOR',
                                            'test_finished': 'test_succeeded'},
                               remapping={'move_waypoints': 'waypoints',
                                          'move_target_wp': 'target_wp',
                                          'move_wp_str': 'wp_str'})

    outcome = sm.execute()
