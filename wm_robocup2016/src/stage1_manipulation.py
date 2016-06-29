#!/usr/bin/env python

import rospy
import smach
from smach_ros import SimpleActionState, IntrospectionServer
import wm_supervisor.srv
from move_base_msgs.msg import MoveBaseAction
from wm_arm_msgs.msg import executePlanAction, executePlanGoal
from wm_arm_msgs.srv import computePlan, computePlanResponse, computePlanRequest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory, CollisionObject
from shape_msgs.msg import SolidPrimitive
import tf_conversions
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionGoal, RecognizedObjectArray, TableArray
from object_recognition_msgs.srv import GetObjectInformation, GetObjectInformationRequest
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from robotiq_c_model_control.msg import CModel_robot_output as eef_cmd
from robotiq_c_model_control.msg import CModel_robot_input as eef_status
import threading
from std_msgs.msg import Float64, Bool, String, UInt8
from actionlib_msgs.msg import GoalStatus
from actionlib import SimpleActionClient
from subprocess import check_call, CalledProcessError

GREEN_FACE = 3
YELLOW_FACE = 4
RED_FACE = 5
PATH_TO_PDF_CREATOR = '~/sara_ws/src/sara_commun/wm_ork/Rviz_Capture/open_rviz.sh'


class WaitForStart(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['do_init'])
        self.start_button_sub = rospy.Subscriber('start_button_msg', Bool, self.start_button_cb, queue_size=4)
        self.voice_recognizer_sub = rospy.Subscriber('output', String, self.voice_recognizer_cb, queue_size=4)
        self.face_cmd = rospy.Publisher('/face_mode', UInt8, queue_size=1, latch=True)

        self.mutex = threading.Lock()
        self.proceed = False

    def voice_recognizer_cb(self, msg):

        self.face_cmd.publish(GREEN_FACE)

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

        return 'do_init'


class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_move_arm', 'init_scan_objects'],
                             input_keys=['init_r_pos'],
                             output_keys=['init_arm_plan'])
        self.eef_pub = rospy.Publisher('/CModelRobotOutput', eef_cmd, queue_size=1, latch=True)
        self.neck_pub = rospy.Publisher('neckHead_controller/command', Float64, queue_size=1, latch=True)
        self.make_plan_srv = rospy.ServiceProxy('/wm_arm_driver_node/compute_manipulator_plan', computePlan)

    def execute(self, ud):
        rospy.logdebug("Entered 'INIT_STATE' state.")

        hand_cmd = eef_cmd()
        hand_cmd.rACT = 1  # activate gripper
        hand_cmd.rGTO = 1  # request to go to position
        hand_cmd.rSP = 200  # set activation speed (0[slowest]-255[fastest])
        hand_cmd.rFR = 0  # set force limit (0[min] - 255[max])
        hand_cmd.rPR = 0  # request to open

        self.eef_pub.publish(hand_cmd)

        neck_cmd = Float64()
        neck_cmd.data = -1.3
        self.neck_pub.publish(neck_cmd)

        rospy.sleep(rospy.Duration(5))

        try:
            res = self.make_plan_srv(targetPose=None, jointPos=ud.init_r_pos,
                                     planningSpace=computePlanRequest.JOINT_SPACE, collisionObject=None)
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to the planning service.")
            return 'init_scan_objects'

        if res.planningResult == computePlanResponse.PLANNING_FAILURE:
            rospy.logwarn("Arm planning failed.")
            return 'init_scan_objects'

        else:
            ud.init_arm_plan = res.trajectory
            return 'init_move_arm'


class InitSupervisor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_arm_estop',
                                             'init_arm_ok',
                                             'init_arm_error'])
        self.supervisor_srv = rospy.ServiceProxy('robot_status', wm_supervisor.srv.robotStatus)
        self.face_cmd = rospy.Publisher('/face_mode', UInt8, queue_size=1, latch=True)

    def execute(self, ud):
        rospy.logdebug("Entered 'INIT_SUPERVISOR' state.")

        try:
            res = self.supervisor_srv()
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to the supervising service.")
            self.face_cmd.publish(RED_FACE)
            return 'init_arm_error'

        if res.status == wm_supervisor.srv.robotStatusResponse.STATUS_OK:
            return 'init_arm_ok'

        rospy.sleep(5.0)
        self.face_cmd.publish(YELLOW_FACE)
        return 'init_arm_estop'


class GetDropPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['drop_pose_acquired'],
                             output_keys=['drop_pose'])

        self.tabletop_sub = rospy.Subscriber('/object_recognition/table_array', TableArray, self.tabletop_cb, queue_size=10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.msg_received = False

        self.mutex = threading.Lock()

        self.tables = TableArray()

    def tabletop_cb(self, table_array):

        self.mutex.acquire()
        self.msg_received = True
        self.tables = table_array
        self.mutex.release()

    def execute(self, ud):

        loop_again = True

        while loop_again:
            self.mutex.acquire()
            if self.msg_received:
                loop_again = False
            self.mutex.release()
            rospy.sleep(rospy.Duration(1))

        transform = self.tf_buffer.lookup_transform('odom', self.tables.header.frame_id, rospy.Time(0))

        drop_pose_z = 1.5
        tmp = PoseStamped()
        lst = []

        for t in range(len(self.tables.tables)):
            tmp.pose = self.tables.tables[t].pose
            tmp.header.frame_id = self.tables.header.frame_id
            tmp.header.stamp = rospy.Time.now()

            eval_pose = do_transform_pose(tmp, transform)

            rpy = tf_conversions.transformations.euler_from_quaternion(eval_pose.pose.orientation)

            if rpy[2]**2 > 0.90:
                lst.append([(eval_pose.pose.position.z - drop_pose_z)**2, eval_pose])

        lst.sort()

        ud.drop_pose = lst[0][1]

        return 'drop_pose_acquired'


class SetObjectTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['target_set', 'no_new_object'],
                             input_keys=['sot_object_array',
                                         'sot_ork_frame',
                                         'sot_picked_objects',
                                         'sot_target_object',
                                         'sot_grasp_target_pose'],
                             output_keys=['sot_target_object',
                                          'sot_grasp_target_pose'])
        self.ork_srv = rospy.ServiceProxy('get_object_info', GetObjectInformation)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

    def execute(self, ud):
        rospy.logdebug("Entered 'SET_OBJECT_TARGET' state.")

        new_target = ''
        is_new_obj = False

        # loop through all found objects
        for i in range(len(ud.sot_object_array)):

            # get object info, mainly its name
            try:
                req = GetObjectInformationRequest()
                req.type = ud.sot_object_array[i].type
                res = self.ork_srv(req)
                # if the object hasn't been grasped yet, make the object the current grasp target
                is_new_obj = True
                if any(res.information.name in s for s in ud.sot_picked_objects):
                    is_new_obj = False

                if is_new_obj:
                    new_target = res.information.name

            except rospy.ServiceException:
                rospy.logerr("GetObjectInformation service request failed.")

            # if a new object has been detected
            if new_target and is_new_obj:

                # get the transform from ork's frame to odom

                transform = self.tf_buffer.lookup_transform('odom', ud.sot_ork_frame, rospy.Time(0))
                # get the object's pose in odom frame
                ud.sot_grasp_target_pose = do_transform_pose(ud.sot_object_array[i].pose.pose, transform)
                ud.sot_target_object = new_target
                ud.sot_grasp_target_pose.pose.position.z += 0.06
                ud.sot_grasp_target_pose.pose.position.x -= 0.1

                print "OBJECT NAME : " + new_target
                print "POSE : "
                print ud.sot_grasp_target_pose
                return 'target_set'
                # rospy.logerr("Could not get transform from " + ud.sot_ork_frame + " to 'odom'.")

        return 'no_new_object'


class ArmPlanGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grasp_arm_plan_succeeded',
                                             'grasp_arm_plan_failed',
                                             'grasp_arm_error'],
                             input_keys=['grasp_target_pose'],
                             output_keys=['grasp_arm_plan'])
        self.make_plan_srv = rospy.ServiceProxy('/wm_arm_driver_node/compute_manipulator_plan', computePlan)

    def execute(self, ud):
        rospy.logdebug("Entered 'GRASP_ARM_PLAN' state.")

        collision_object = CollisionObject()
        collision_object.header.frame_id = 'odom'
        collision_object.id = 'coffee'
        primitive = SolidPrimitive()
        primitive.type = primitive.CYLINDER
        primitive.dimensions.append(0.15)
        primitive.dimensions.append(0.06)
        collision_object.primitives.append(primitive)

        collision_object.operation = collision_object.ADD

        ta = PoseStamped()
        ta.header.frame_id = 'odom'
        ta.header.stamp = rospy.Time.now()
        ta.pose.position.x = 0.843708
        ta.pose.position.y = -0.0977909
        ta.pose.position.z = 1.10764
        ta.pose.orientation.x = -0.126284
        ta.pose.orientation.y = -0.368197
        ta.pose.orientation.z = -0.525026
        ta.pose.orientation.w = 0.756856

        collision_object.primitive_poses.append(ud.grasp_target_pose.pose)

        try:
            res = self.make_plan_srv(targetPose=ud.grasp_target_pose, jointPos=[], planningSpace=computePlanRequest.CARTESIAN_SPACE, collisionObject=collision_object)
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to the planning service.")
            return 'grasp_arm_error'

        if res.planningResult == computePlanResponse.PLANNING_FAILURE:
            rospy.logwarn("Arm planning failed. Moving the base is probably required to reach target pose.")
            return 'grasp_arm_plan_failed'

        else:
            ud.grasp_arm_plan = res.trajectory
            rospy.sleep(rospy.Duration(10))
            return 'grasp_arm_plan_succeeded'


class BasePlanGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grasp_base_plan_succeeded',
                                             'grasp_base_plan_failed',
                                             'grasp_base_error'],
                             input_keys=['grasp_target_pose'],
                             output_keys=['grasp_move_base_odom'])
        self.make_plan_srv = rospy.ServiceProxy('/wm_arm_driver_node/compute_base_plan', computePlan)

    def execute(self, ud):
        rospy.logdebug("Entered 'GRASP_BASE_PLAN' state.")

        try:
            res = self.make_plan_srv(targetPose=ud.grasp_target_pose, jointPos=[], planningSpace=computePlanRequest.CARTESIAN_SPACE)
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to the planning service.")
            return 'grasp_base_error'

        if res.planningResult == computePlanResponse.PLANNING_FAILURE:
            rospy.logwarn("Base planning failed. Target pose is probably unreachable.")
            return 'grasp_base_plan_failed'

        else:
            # get the last point in the trajectory
            last_traj_point = res.trajectory.joint_trajectory.points[-1]

            # plug the joints positions in move_base pose
            grasp_move_base_odom = PoseStamped()
            grasp_move_base_odom.header.frame_id = 'odom'
            grasp_move_base_odom.pose.position.x = last_traj_point.positions[0]
            grasp_move_base_odom.pose.position.y = last_traj_point.positions[1]
            q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, last_traj_point.positions[2])
            grasp_move_base_odom.pose.orientation.x = q[0]
            grasp_move_base_odom.pose.orientation.y = q[1]
            grasp_move_base_odom.pose.orientation.z = q[2]
            grasp_move_base_odom.pose.orientation.w = q[3]
            ud.grasp_move_base_odom = grasp_move_base_odom

            print grasp_move_base_odom

            rospy.sleep(rospy.Duration(10))

            return 'grasp_base_plan_succeeded'


class GraspBaseSupervisor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grasp_base_estop',
                                             'grasp_base_ok',
                                             'grasp_base_error'])
        self.supervisor_srv = rospy.ServiceProxy('robot_status', wm_supervisor.srv.robotStatus)

    def execute(self, ud):
        rospy.logdebug("Entered 'GRASP_BASE_SUPERVISOR' state.")

        try:
            res = self.supervisor_srv()
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to the supervising service.")
            return 'grasp_base_error'

        if res.status == wm_supervisor.srv.robotStatusResponse.STATUS_OK:
            return 'grasp_base_ok'

        rospy.sleep(5.0)

        return 'grasp_base_estop'


class GraspArmSupervisor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grasp_arm_estop',
                                             'grasp_arm_ok',
                                             'grasp_arm_error'])
        self.supervisor_srv = rospy.ServiceProxy('robot_status', wm_supervisor.srv.robotStatus)

    def execute(self, ud):
        rospy.logdebug("Entered 'GRASP_ARM_SUPERVISOR' state.")

        try:
            res = self.supervisor_srv()
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to the supervising service.")
            return 'grasp_arm_error'

        if res.status == wm_supervisor.srv.robotStatusResponse.STATUS_OK:
            return 'grasp_arm_ok'

        rospy.sleep(5.0)

        return 'grasp_arm_estop'


class GraspPlan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grasp_plan_succeeded',
                                             'grasp_plan_failed'],
                             input_keys=['grasp_target_pose'],
                             output_keys=['grasp_arm_plan'])
        self.make_plan_srv = rospy.ServiceProxy('/wm_arm_driver_node/compute_manipulator_plan', computePlan)

    def execute(self, ud):
        rospy.logdebug("Entered 'GRASP_PLAN' state.")

        tp = ud.grasp_target_pose
        tp.pose.position.x += 0.1

        try:
            res = self.make_plan_srv(targetPose=tp, jointPos=[], planningSpace=computePlanRequest.CARTESIAN_SPACE, collisionObject=None)
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to the planning service.")
            return 'grasp_plan_failed'

        if res.planningResult == computePlanResponse.PLANNING_FAILURE:
            return 'grasp_plan_failed'

        else:
            ud.grasp_arm_plan = res.trajectory
            rospy.sleep(rospy.Duration(10))
            return 'grasp_plan_succeeded'


class GraspSupervisor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grasp_ok', 'grasp_estop'])
        self.supervisor_srv = rospy.ServiceProxy('robot_status', wm_supervisor.srv.robotStatus)

    def execute(self, ud):
        rospy.logdebug("Entered 'GRASP_SUPERVISOR' state.")

        try:
            res = self.supervisor_srv()
            if res.status == wm_supervisor.srv.robotStatusResponse.STATUS_OK:
                return 'grasp_ok'

        except rospy.ServiceException:
            rospy.sleep(5.0)
            rospy.logerr("Failed to connect to the supervising service.")

        return 'grasp_estop'


class CloseEef(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['close_eef_cmd_sent'])
        self.eef_pub = rospy.Publisher('/CModelRobotOutput', eef_cmd, queue_size=1, latch=True)

    def execute(self, ud):
        rospy.logdebug("Entered 'CLOSE_EEF' state.")

        cmd = eef_cmd()

        cmd.rACT = 1  # activate gripper
        cmd.rGTO = 1  # request to go to position
        cmd.rSP = 200  # set activation speed (0[slowest]-255[fastest])
        cmd.rFR = 0  # set force limit (0[min] - 255[max])
        cmd.rPR = 250  # request to close

        self.eef_pub.publish(cmd)

        return 'close_eef_cmd_sent'


class MonitorEef(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['close_eef_ok',
                                             'close_eef_error'],
                             input_keys=['eef_picked_objects', 'eef_target_object'],
                             output_keys=['eef_picked_objects', 'eef_target_object'])
        self.eef_sub = rospy.Subscriber('/CModelRobotInput', eef_status, self.eef_cb, queue_size=10)
        self.tts_pub = rospy.Publisher('sara_tts', String, queue_size=1, latch=True)

        self.mutex = threading.Lock()

        self.eef_closed = False
        self.eef_error = False

        self.in_progress = False

    def eef_cb(self, status):
        self.mutex.acquire()

        if not self.in_progress:
            if status.gSTA == 1:  # activation in progress
                self.in_progress = True
                self.mutex.release()
                return

        if self.in_progress:
            if status.gOBJ == 2 and status.gPO != 255:
                self.eef_closed = True
                self.mutex.release()
                return

            elif status.gOBJ == 2:
                self.eef_error = True
                self.mutex.release()

        return

    def execute(self, ud):
        rospy.logdebug("Entered 'MONITOR_EEF' state.")

        while True:
            self.mutex.acquire()
            if self.eef_error or self.eef_closed:
                self.mutex.release()
                break
            self.mutex.release()
            rospy.sleep(rospy.Duration(1))

        self.mutex.acquire()

        if self.eef_closed:
            ud.eef_picked_objects.append(ud.eef_target_object)
            tts_msg = String()
            tts_msg.data = "I picked up the " + ud.eef_target_object + "."
            self.tts_pub.publish(tts_msg)
            ud.eef_target_object = ''
            return 'close_eef_ok'
        else:
            return 'close_eff_error'


class ResetPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reset_plan_ok', 'reset_plan_error'],
                             input_keys=['reset_r_pos'],
                             output_keys=['reset_arm_plan'])
        self.make_plan_srv = rospy.ServiceProxy('/wm_arm_driver_node/compute_manipulator_plan', computePlan)

    def execute(self, ud):

        try:
            res = self.make_plan_srv(targetPose=None, jointPos=ud.reset_r_pos,
                                     planningSpace=computePlanRequest.JOINT_SPACE, collisionObject=None)
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to the planning service.")
            return 'reset_plan_error'

        if res.planningResult == computePlanResponse.PLANNING_SUCCESS:
            ud.reset_arm_plan = res.trajectory
            return 'reset_plan_ok'
        else:
            rospy.logwarn("Arm planning failed.")
            return 'reset_plan_error'


class ResetSupervisor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reset_ok', 'reset_estop'])
        self.supervisor_srv = rospy.ServiceProxy('robot_status', wm_supervisor.srv.robotStatus)

    def execute(self, ud):
        rospy.logdebug("Entered 'RESET_SUPERVISOR' state.")

        try:
            res = self.supervisor_srv()
            if res.status == wm_supervisor.srv.robotStatusResponse.STATUS_OK:
                return 'reset_ok'

        except rospy.ServiceException:
            rospy.sleep(5.0)
            rospy.logerr("Failed to connect to the supervising service.")

        return 'reset_estop'


class DoReset(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['reset_plan'])
        self.eef_pub = rospy.Publisher('/CModelRobotOutput', eef_cmd, queue_size=1, latch=True)
        self.move_arm_client = SimpleActionClient('/wm_arm_driver_node/execute_plan', executePlanAction)

    def execute(self, ud):
        rospy.logdebug("Entered 'RO_RESET' state.")

        hand_cmd = eef_cmd()
        hand_cmd.rACT = 1  # activate gripper
        hand_cmd.rGTO = 1  # request to go to position
        hand_cmd.rSP = 200  # set activation speed (0[slowest]-255[fastest])
        hand_cmd.rFR = 0  # set force limit (0[min] - 255[max])
        hand_cmd.rPR = 0  # request to open

        self.eef_pub.publish(hand_cmd)

        self.move_arm_client.wait_for_server()
        goal = executePlanGoal()
        goal.trajectory = ud.reset_plan
        self.move_arm_client.send_goal(goal)

        self.move_arm_client.wait_for_result()

        status = self.move_arm_client.get_state()
        if status == GoalStatus.SUCCEEDED:
            return 'succeeded'
        elif status == GoalStatus.PREEMPTED:
            return 'preempted'
        else:
            return 'aborted'


class ArmDropPlan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['drop_arm_plan_succeeded',
                                             'drop_arm_plan_failed',
                                             'drop_arm_error'],
                             input_keys=['drop_target_pose'],
                             output_keys=['drop_arm_plan'])
        self.make_plan_srv = rospy.ServiceProxy('/wm_arm_driver_node/compute_manipulator_plan', computePlan)

    def execute(self, ud):
        rospy.logdebug("Entered 'DROP_ARM_PLAN' state.")

        try:
            res = self.make_plan_srv(targetPose=ud.drop_target_pose, jointPos=[], planningSpace=computePlanRequest.CARTESIAN_SPACE)
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to the planning service.")
            return 'drop_arm_error'

        if res.planningResult == computePlanResponse.PLANNING_FAILURE:
            rospy.logwarn("Planning failed. Target pose is probably unreachable.")
            return 'drop_arm_plan_failed'

        else:
            ud.arm_plan = res.trajectory
            return 'drop_arm_plan_succeeded'


class DropArmSupervisor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['drop_arm_estop',
                                             'drop_arm_ok',
                                             'drop_arm_error'])
        self.supervisor_srv = rospy.ServiceProxy('robot_status', wm_supervisor.srv.robotStatus)

    def execute(self, ud):
        rospy.logdebug("Entered 'DROP_ARM_SUPERVISOR' state.")

        try:
            res = self.supervisor_srv()
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to the supervising service.")
            return 'drop_arm_error'

        if res.status == wm_supervisor.srv.robotStatusResponse.STATUS_OK:
            return 'drop_arm_ok'

        rospy.sleep(5.0)

        return 'drop_arm_estop'


class DropBasePlan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['drop_base_plan_succeeded',
                                         'drop_base_plan_failed',
                                         'drop_base_error'],
                             input_keys=['drop_target_pose'],
                             output_keys=['drop_move_base_odom'])
        self.make_plan_srv = rospy.ServiceProxy('/wm_arm_driver_node/compute_base_plan', computePlan)

    def execute(self, ud):
        rospy.logdebug("Entered 'DROP_BASE_PLAN' state.")

        try:
            res = self.make_plan_srv(targetPose=ud.drop_target_pose, jointPos=[], planningSpace=computePlanRequest.CARTESIAN_SPACE)
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to the planning service.")
            return 'drop_base_error'

        if res.planningResult == computePlanResponse.PLANNING_FAILURE:
            rospy.logwarn("Base planning failed. Target pose is probably unreachable.")
            return 'drop_base_plan_failed'

        else:
            # get the last point in the trajectory
            last_traj_point = res.trajectory.joint_trajectory.points[-1]

            # plug the joints positions in move_base pose
            drop_move_base_odom = PoseStamped()
            drop_move_base_odom.header.frame_id = 'odom'
            drop_move_base_odom.pose.position.x = last_traj_point.positions[0]
            drop_move_base_odom.pose.position.y = last_traj_point.positions[1]
            q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, last_traj_point.positions[2])
            drop_move_base_odom.pose.orientation.x = q[0]
            drop_move_base_odom.pose.orientation.y = q[1]
            drop_move_base_odom.pose.orientation.z = q[2]
            drop_move_base_odom.pose.orientation.w = q[3]
            ud.grasp_move_base_odom = drop_move_base_odom

            return 'drop_base_plan_succeeded'


class RetractArmPlan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['retract_plan_succeeded',
                                             'retract_plan_failed',
                                             'retract_plan_error'],
                             input_keys=['retract_joint_pos'],
                             output_keys=['retract_plan'])
        self.make_plan_srv = rospy.ServiceProxy('/wm_arm_driver_node/compute_manipulator_plan', computePlan)

    def execute(self, ud):
        rospy.logdebug("Entered 'RETRACT_ARM_PLAN' state.")

        try:
            req = computePlanRequest()
            req.planningSpace = computePlanRequest.JOINT_SPACE
            req.jointPos = ud.retract_joint_pos
            res = self.make_plan_srv(targetPose=None, jointPos = ud.retract_joint_pos, planningSpace=computePlanRequest.JOINT_SPACE)
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to the planning service.")
            return 'retract_error'

        if res.status == computePlanResponse.PLANNING_FAILURE:
            rospy.logwarn("Planning failed. Target pose is probably unreachable.")
            return 'retract_plan_failed'

        else:
            ud.arm_plan = res.trajectory
            return 'retract_plan_succeeded'


class RetractArmSupervisor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['retract_arm_estop',
                                             'retract_arm_ok',
                                             'retract_arm_error'])
        self.supervisor_srv = rospy.ServiceProxy('robot_status', wm_supervisor.srv.robotStatus)

    def execute(self, ud):
        rospy.logdebug("Entered 'RETRACT_ARM_SUPERVISOR' state.")

        try:
            res = self.supervisor_srv()
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to the supervising service.")
            return 'retract_arm_error'

        if res.status == wm_supervisor.srv.robotStatusResponse.STATUS_OK:
            return 'retract_arm_ok'

        rospy.sleep(5.0)

        return 'retract_arm_estop'


class DropBaseSupervisor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['drop_base_estop',
                                             'drop_base_ok',
                                             'drop_base_error'])
        self.supervisor_srv = rospy.ServiceProxy('robot_status', wm_supervisor.srv.robotStatus)

    def execute(self, ud):
        rospy.logdebug("Entered 'DROP_BASE_SUPERVISOR' state.")

        try:
            res = self.supervisor_srv()
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to the supervising service.")
            return 'drop_base_error'

        if res.status == wm_supervisor.srv.robotStatusResponse.STATUS_OK:
            return 'drop_base_ok'

        rospy.sleep(5.0)

        return 'drop_base_estop'


class OpenEef(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['open_eef_ok', 'open_eef_error'])
        self.eef_pub = rospy.Publisher('/CModelRobotOutput', eef_cmd, queue_size=1, latch=True)

    def execute(self, ud):
        rospy.logdebug("Entered 'OPEN_EEF' state.")

        hand_cmd = eef_cmd()
        hand_cmd.rACT = 1  # activate gripper
        hand_cmd.rGTO = 1  # request to go to position
        hand_cmd.rSP = 200  # set activation speed (0[slowest]-255[fastest])
        hand_cmd.rFR = 0  # set force limit (0[min] - 255[max])
        hand_cmd.rPR = 0  # request to open

        self.eef_pub.publish(hand_cmd)

        return 'open_eef_ok'


class FailTest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit'])
        # TODO
        # connect to service to turn face red, announce the robot cannot recover autonomously

    def execute(self, ud):
        # TODO
        return 'exit'


if __name__ == '__main__':

    rospy.init_node('stage1_manipulation_node')
    sm = smach.StateMachine(outcomes=['test_failed', 'test_success'])

    # target object pose in odom frame
    sm.userdata.grasp_target_pose = PoseStamped()

    # drop location pose in odom frame
    sm.userdata.drop_target_pose = PoseStamped()
    sm.userdata.drop_target_pose.header.frame_id = 'odom'

    # move_base goal pose in odom frame
    sm.userdata.move_base_odom = PoseStamped()

    # move arm trajectory
    sm.userdata.arm_plan = RobotTrajectory()

    # joints' retracted position in joint space (arm joints only)
    sm.userdata.retract_pos = [0.0, 0.0, 0.80, -1.80, 0.0, 0.0]

    # array of RecognizedObject
    sm.userdata.object_array = RecognizedObjectArray()
    # define a object_recognition goal
    sm.userdata.ork_goal = ObjectRecognitionGoal()
    sm.userdata.ork_goal.use_roi = False
    sm.userdata.ork_frame = ''

    sm.userdata.picked_objects = ['apple']
    sm.userdata.target_object = ''

    with sm:

        def ork_result_cb(userdata, status, result):

            if status == GoalStatus.SUCCEEDED:
                if len(result.recognized_objects.objects) > 0:
                    userdata.ork_object_array = result.recognized_objects.objects
                    userdata.ork_action_frame = result.recognized_objects.header.frame_id
                    bash_str = 'sh ' + PATH_TO_PDF_CREATOR
                    check_call([bash_str])
                    return 'succeeded'

            return 'aborted'

        smach.StateMachine.add('WAIT_FOR_START',
                               WaitForStart(),
                               transitions={'do_init': 'INIT_STATE'})

        smach.StateMachine.add('INIT_STATE',
                               InitState(),
                               transitions={'init_move_arm': 'INIT_SUPERVISOR',
                                            'init_scan_objects': 'SCAN_FOR_OBJECTS'},
                               remapping={'init_arm_plan': 'arm_plan',
                                          'init_r_pos': 'retract_pos'})

        smach.StateMachine.add('INIT_SUPERVISOR',
                               InitSupervisor(),
                               transitions={'init_arm_estop': 'INIT_SUPERVISOR',
                                            'init_arm_ok': 'INIT_ARM',
                                            'init_arm_error': 'INIT_SUPERVISOR'})

        smach.StateMachine.add('INIT_ARM',
                               SimpleActionState('/wm_arm_driver_node/execute_plan',
                                                 executePlanAction,
                                                 goal_slots=['trajectory']),
                               transitions={'succeeded': 'SCAN_FOR_OBJECTS',
                                            'aborted': 'SCAN_FOR_OBJECTS',
                                            'preempted': 'SCAN_FOR_OBJECTS'},
                               remapping={'trajectory': 'arm_plan'})

        smach.StateMachine.add('GET_DROP_POSE',
                               GetDropPose(),
                               transitions={'drop_pose_acquired': 'SCAN_FOR_OBJECTS'},
                               remapping={'drop_pose': 'drop_target_pose'})

        smach.StateMachine.add('SCAN_FOR_OBJECTS',
                               SimpleActionState('/object_recognition/recognize_objects',
                                                 ObjectRecognitionAction,
                                                 result_cb=ork_result_cb,
                                                 output_keys={'ork_object_array',
                                                              'ork_action_frame'}),
                               transitions={'succeeded': 'SET_OBJECT_TARGET',
                                            'aborted': 'SCAN_FOR_OBJECTS',
                                            'preempted': 'SCAN_FOR_OBJECTS'},
                               remapping={'ork_object_array': 'object_array',
                                          'ork_action_frame': 'ork_frame'})

        smach.StateMachine.add('SET_OBJECT_TARGET',
                               SetObjectTarget(),
                               transitions={'target_set': 'GRASP_ARM_PLAN',
                                            'no_new_object': 'SCAN_FOR_OBJECTS'},
                               remapping={'sot_object_array': 'object_array',
                                          'sot_ork_frame': 'ork_frame',
                                          'sot_picked_objects': 'picked_objects',
                                          'sot_target_object': 'target_object',
                                          'sot_grasp_target_pose': 'grasp_target_pose'})

        smach.StateMachine.add('GRASP_ARM_PLAN',
                               ArmPlanGrasp(),
                               transitions={'grasp_arm_plan_succeeded': 'GRASP_ARM_SUPERVISOR',
                                            'grasp_arm_plan_failed': 'GRASP_BASE_PLAN',
                                            'grasp_arm_error': 'TEST_FAILED'},
                               remapping={'grasp_arm_plan': 'arm_plan',
                                          'grasp_target_pose': 'grasp_target_pose'})

        smach.StateMachine.add('GRASP_BASE_PLAN',
                               BasePlanGrasp(),
                               transitions={'grasp_base_plan_succeeded': 'GRASP_BASE_SUPERVISOR',
                                            'grasp_base_plan_failed': 'GRASP_ARM_PLAN',  # TODO SKIP_OBJECT
                                            'grasp_base_error': 'TEST_FAILED'},
                               remapping={'grasp_target_pose': 'grasp_target_pose',
                                          'grasp_move_base_odom': 'move_base_odom'})

        smach.StateMachine.add('GRASP_BASE_SUPERVISOR',
                               GraspBaseSupervisor(),
                               transitions={'grasp_base_estop': 'GRASP_BASE_SUPERVISOR',
                                            'grasp_base_ok': 'GRASP_MOVE_BASE',
                                            'grasp_base_error': 'TEST_FAILED'})

        smach.StateMachine.add('GRASP_ARM_SUPERVISOR',
                               GraspArmSupervisor(),
                               transitions={'grasp_arm_estop': 'GRASP_ARM_SUPERVISOR',
                                            'grasp_arm_ok': 'GRASP_MOVE_ARM',
                                            'grasp_arm_error': 'TEST_FAILED'})

        smach.StateMachine.add('GRASP_MOVE_BASE',
                               SimpleActionState('move_base',
                                                 MoveBaseAction,
                                                 goal_slots=['target_pose']),
                               transitions={'succeeded': 'GRASP_ARM_PLAN',
                                            'aborted': 'GRASP_ARM_PLAN',
                                            'preempted': 'GRASP_ARM_PLAN'},
                               remapping={'target_pose': 'move_base_odom'})

        smach.StateMachine.add('GRASP_MOVE_ARM',
                               SimpleActionState('/wm_arm_driver_node/execute_plan',
                                                 executePlanAction,
                                                 goal_slots=['trajectory']),
                               transitions={'succeeded': 'GRASP_PLAN',
                                            'aborted': 'GRASP_ARM_PLAN',
                                            'preempted': 'GRASP_ARM_PLAN'},
                               remapping={'trajectory': 'arm_plan'})

        smach.StateMachine.add('GRASP_PLAN',
                               GraspPlan(),
                               transitions={'grasp_plan_succeeded': 'GRASP_SUPERVISOR',
                                            'grasp_plan_failed': 'RESET_POSITION'},
                               remapping={'grasp_arm_plan': 'arm_plan',
                                          'grasp_target_pose': 'grasp_target_pose'})

        smach.StateMachine.add('GRASP_SUPERVISOR',
                               GraspSupervisor(),
                               transitions={'grasp_ok': 'EXECUTE_GRASP',
                                            'grasp_estop': 'GRASP_SUPERVISOR'})

        smach.StateMachine.add('EXECUTE_GRASP',
                               SimpleActionState('/wm_arm_driver_node/execute_plan',
                                                 executePlanAction,
                                                 goal_slots=['trajectory']),
                               transitions={'succeeded': 'CLOSE_EEF',
                                            'aborted': 'GRASP_PLAN',
                                            'preempted': 'GRASP_PLAN'},
                               remapping={'trajectory': 'arm_plan'})

        smach.StateMachine.add('CLOSE_EEF',
                               CloseEef(),
                               transitions={'close_eef_cmd_sent': 'MONITOR_EEF'})

        smach.StateMachine.add('MONITOR_EEF',
                               MonitorEef(),
                               transitions={'close_eef_ok': 'RETRACT_ARM_PLAN',
                                            'close_eef_error': 'RESET_POSITION'},
                               remapping={'eef_picked_objects': 'picked_objects',
                                          'eef_target_object': 'target_object'})

        smach.StateMachine.add('RESET_POSITION',
                               ResetPosition(),
                               transitions={'reset_plan_ok': 'RESET_SUPERVISOR',
                                            'reset_plan_error': 'RESET_POSITION'},
                               remapping={'reset_r_pos': 'retract_pos',
                                          'reset_trajectory': 'arm_plan'})

        smach.StateMachine.add('RESET_SUPERVISOR',
                               ResetSupervisor(),
                               transitions={'reset_ok': 'DO_RESET',
                                            'reset_estop': 'RESET_SUPERVISOR'})

        smach.StateMachine.add('DO_RESET',
                               DoReset(),
                               transitions={'succeeded': 'SET_OBJECT_TARGET',
                                            'aborted': 'RESET_POSITION',
                                            'preempted': 'RESET_POSITION'},
                               remapping={'reset_plan': 'arm_plan'})

        smach.StateMachine.add('DROP_ARM_PLAN',
                               ArmDropPlan(),
                               transitions={'drop_arm_plan_succeeded': 'DROP_ARM_SUPERVISOR',
                                            'drop_arm_plan_failed': 'DROP_BASE_PLAN',
                                            'drop_arm_error': 'TEST_FAILED'},
                               remapping={'drop_arm_plan': 'arm_plan',
                                          'drop_arm_target_pose': 'drop_target_pose'})

        smach.StateMachine.add('DROP_ARM_SUPERVISOR',
                               DropArmSupervisor(),
                               transitions={'drop_arm_estop': 'DROP_ARM_SUPERVISOR',
                                            'drop_arm_ok': 'DROP_MOVE_ARM',
                                            'drop_arm_error': 'TEST_FAILED'})

        smach.StateMachine.add('DROP_MOVE_ARM',
                               SimpleActionState('/wm_arm_driver_node/execute_plan',
                                                 executePlanAction,
                                                 goal_slots=['trajectory']),
                               transitions={'succeeded': 'OPEN_EEF',
                                            'aborted': 'DROP_ARM_PLAN',
                                            'preempted': 'DROP_ARM_PLAN'},
                               remapping={'trajectory': 'arm_plan'})

        smach.StateMachine.add('DROP_BASE_PLAN',
                               DropBasePlan(),
                               transitions={'drop_base_plan_succeeded': 'RETRACT_ARM_PLAN',
                                            'drop_base_plan_failed': 'DROP_ARM_PLAN',
                                            'drop_base_error': 'TEST_FAILED'},
                               remapping={'drop_target_pose': 'drop_target_pose',
                                          'drop_move_base_odom': 'move_base_odom'})

        smach.StateMachine.add('RETRACT_ARM_PLAN',
                               RetractArmPlan(),
                               transitions={'retract_plan_succeeded': 'RETRACT_ARM_SUPERVISOR',
                                            'retract_plan_failed': 'RETRACT_ARM_PLAN',
                                            'retract_plan_error': 'TEST_FAILED'},
                               remapping={'retract_joint_pos': 'retract_pos',
                                          'retract_plan': 'arm_plan'})

        smach.StateMachine.add('RETRACT_ARM_SUPERVISOR',
                               RetractArmSupervisor(),
                               transitions={'retract_arm_estop': 'RETRACT_ARM_SUPERVISOR',
                                            'retract_arm_ok': 'RETRACT_MOVE_ARM',
                                            'retract_arm_error': 'TEST_FAILED'})

        smach.StateMachine.add('RETRACT_MOVE_ARM',
                               SimpleActionState('/wm_arm_driver_node/execute_plan',
                                                 executePlanAction,
                                                 goal_slots=['trajectory']),
                               transitions={'succeeded': 'DROP_BASE_SUPERVISOR',
                                            'aborted': 'RETRACT_ARM_PLAN',
                                            'preempted': 'RETRACT_ARM_PLAN'},
                               remapping={'trajectory': 'arm_plan'})

        smach.StateMachine.add('DROP_BASE_SUPERVISOR',
                               DropBaseSupervisor(),
                               transitions={'drop_base_estop': 'DROP_BASE_SUPERVISOR',
                                            'drop_base_ok': 'DROP_MOVE_BASE',
                                            'drop_base_error': 'TEST_FAILED'})

        smach.StateMachine.add('DROP_MOVE_BASE',
                               SimpleActionState('/wm_arm_driver_node/execute_plan',
                                                 executePlanAction,
                                                 goal_slots=['trajectory']),
                               transitions={'succeeded': 'DROP_ARM_PLAN',
                                            'aborted': 'DROP_ARM_PLAN',
                                            'preempted': 'DROP_ARM_PLAN'},
                               remapping={'trajectory': 'arm_plan'})

        smach.StateMachine.add('OPEN_EEF',
                               OpenEef(),
                               transitions={'open_eef_ok': 'RESET_POSITION',
                                            'open_eef_error': 'OPEN_EEF'})

        smach.StateMachine.add('TEST_FAILED',
                               FailTest(),
                               transitions={'exit': 'test_failed'})

    sis = IntrospectionServer('smach_introspection_server', sm, 'manipulation_smach')
    sis.start()

    outcome = sm.execute()

    while not rospy.is_shutdown():
        rospy.spin()

    sis.stop()
