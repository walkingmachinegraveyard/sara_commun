#!/usr/bin/env python

import rospy
import smach
from smach_ros import SimpleActionState
import wm_supervisor.srv
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8, String
import threading
from std_msgs.msg import Float64


class InitRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_done'])
        self.neck_pub = rospy.Publisher('neckHead_controller/command', Float64, queue_size=1, latch=True)

    def execute(self, ud):

        cmd = Float64()
        cmd.data = 0.0
        self.neck_pub.publish(cmd)

        return 'init_done'


class WaitForStart(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['begin_inspection'])
        # TODO subscribe to start button topic
        self.start_sub = rospy.Subscriber('topic_name', Int8, self.sub_cb, queue_size=4)

        self.mutex = threading.Lock()
        self.start_signal_received = False

    def sub_cb(self, msg):

        self.mutex.acquire()

        if msg.data == 1:
            self.start_signal_received = True

        self.mutex.release()

    def execute(self, ud):

        while True:
            self.mutex.acquire()
            if self.start_signal_received:
                self.mutex.release()
                break

            self.mutex.release()
            rospy.sleep(rospy.Duration(1))

        return 'begin_inspection'


class InspectionPoseSupervisor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['inspection_pose_estop', 'inspection_pose_ok'])
        self.supervisor_srv = rospy.ServiceProxy('robot_status', wm_supervisor.srv.robotStatus)

    def execute(self, ud):

        try:
            res = self.supervisor_srv()
            if res.status == wm_supervisor.srv.robotStatusResponse.STATUS_OK:
                return 'inspection_pose_ok'

        except rospy.ServiceException:
            rospy.logerr("Failed to connect to the supervising service.")

        rospy.sleep(5.0)

        return 'inspection_pose_estop'


class WaitForContinue(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_inspection'])
        self.barcode_sub = rospy.Subscriber('barcode', String, self.sub_cb, queue_size=4)

        self.mutex = threading.Lock()
        self.continue_code_received = False

    def sub_cb(self, msg):

        self.mutex.acquire()

        if msg.data.lower().find('continue') != -1:
            self.continue_code_received = True

        self.mutex.release()

    def execute(self, ud):

        while True:
            self.mutex.acquire()
            if self.continue_code_received:
                self.mutex.release()
                break

            self.mutex.release()
            rospy.sleep(rospy.Duration(1))

        return 'continue_inspection'


class ExitPoseSupervisor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit_pose_estop', 'exit_pose_ok'])
        self.supervisor_srv = rospy.ServiceProxy('robot_status', wm_supervisor.srv.robotStatus)

    def execute(self, ud):

        try:
            res = self.supervisor_srv()
            if res.status == wm_supervisor.srv.robotStatusResponse.STATUS_OK:
                return 'exit_pose_ok'

        except rospy.ServiceException:
            rospy.logerr("Failed to connect to the supervising service.")

        rospy.sleep(5.0)

        return 'exit_pose_estop'

if __name__ == '__main__':

    rospy.init_node('stage1_navigation_node')
    sm = smach.StateMachine(outcomes=['exit'])

    sm.userdata.inspection_pose = MoveBaseGoal()
    sm.userdata.inspection_pose.target_pose.header.frame_id = 'map'
    sm.userdata.inspection_pose.target_pose.header.stamp = rospy.Time.now()
    sm.userdata.inspection_pose.target_pose.pose.position.x = 3.0
    sm.userdata.inspection_pose.target_pose.pose.position.y = -1.0
    sm.userdata.inspection_pose.target_pose.pose.position.z = 0.0
    sm.userdata.inspection_pose.target_pose.pose.orientation.x = 0.0
    sm.userdata.inspection_pose.target_pose.pose.orientation.y = 0.0
    sm.userdata.inspection_pose.target_pose.pose.orientation.z = 0.0
    sm.userdata.inspection_pose.target_pose.pose.orientation.w = 1.0

    sm.userdata.exit_pose = MoveBaseGoal()
    sm.userdata.exit_pose.target_pose.header.frame_id = 'map'
    sm.userdata.exit_pose.target_pose.header.stamp = rospy.Time.now()
    sm.userdata.exit_pose.target_pose.pose.position.x = 0.0
    sm.userdata.exit_pose.target_pose.pose.position.y = 0.0
    sm.userdata.exit_pose.target_pose.pose.position.z = 0.0
    sm.userdata.exit_pose.target_pose.pose.orientation.x = 0.0
    sm.userdata.exit_pose.target_pose.pose.orientation.y = 0.0
    sm.userdata.exit_pose.target_pose.pose.orientation.z = 0.0
    sm.userdata.exit_pose.target_pose.pose.orientation.w = 1.0

    with sm:
        smach.StateMachine.add('INIT',
                               InitRobot(),
                               transitions={'init_done': 'WAIT_FOR_START'})

        smach.StateMachine.add('WAIT_FOR_START',
                               WaitForStart(),
                               transitions={'begin_inspection': 'INSPECTION_POSE_SUPERVISOR'})

        smach.StateMachine.add('INSPECTION_POSE_SUPERVISOR',
                               InspectionPoseSupervisor(),
                               transitions={'inspection_pose_estop': 'INSPECTION_POSE_SUPERVISOR',
                                            'inspection_pose_ok': 'MOVE_INSPECTION_POSE'})

        smach.StateMachine.add('MOVE_INSPECTION_POSE',
                               SimpleActionState('move_base',
                                                 MoveBaseAction,
                                                 goal_slots=['target_pose']),
                               transitions={'succeeded': 'WAIT_FOR_CONTINUE',
                                            'aborted': 'INSPECTION_POSE_SUPERVISOR',
                                            'preempted': 'INSPECTION_POSE_SUPERVISOR'},
                               remapping={'target_pose': 'inspection_pose'})

        smach.StateMachine.add('WAIT_FOR_CONTINUE',
                               WaitForContinue(),
                               transitions={'continue_inspection': 'EXIT_POSE_SUPERVISOR'})

        smach.StateMachine.add('EXIT_POSE_SUPERVISOR',
                               ExitPoseSupervisor(),
                               transitions={'exit_pose_estop': 'EXIT_POSE_SUPERVISOR',
                                            'exit_pose_ok': 'MOVE_EXIT_POSE'})

        smach.StateMachine.add('MOVE_EXIT_POSE',
                               SimpleActionState('move_base',
                                                 MoveBaseAction,
                                                 goal_slots=['target_pose']),
                               transitions={'succeeded': 'exit',
                                            'aborted': 'EXIT_POSE_SUPERVISOR',
                                            'preempted': 'EXIT_POSE_SUPERVISOR'},
                               remapping={'target_pose': 'exit_pose'})

    outcome = sm.execute()

