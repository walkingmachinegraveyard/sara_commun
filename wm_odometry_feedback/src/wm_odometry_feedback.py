#!/usr/bin/env python

import rospy
from roboteq_msgs.msg import Feedback
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
import message_filters
import tf2_ros
import tf_conversions
import approxsync
from math import cos, sin


class OdometryFeedback:

    def __init__(self):

        # get parameters
        # x axis distance between wheel axis and the robot's centroid
        self.alpha = rospy.get_param('alpha', 0.31)    # in meter
        # y axis distance between wheel radial median and the robot'S centroid
        self.beta = rospy.get_param('beta', 0.30)    # in meter
        self.wheel_radius = rospy.get_param('wheel_radius', 0.075)    # wheel radius, in meter
        self.child_id = rospy.get_param('/wm_odometry_feedback_node/child_frame_id', "base_link")
        self.frame_id = rospy.get_param('/wm_odometry_feedback_node/frame_id', "odom")
        # gearbox ratio
        self.gb_ratio = rospy.get_param('gearbox_ratio', 15.0)

        self.prev_time = rospy.Time()

        # direct kinematics matrix
        self.dk_x = [1.0/4, -1.0/4, 1.0/4, -1.0/4]
        self.dk_y = [-1.0/4, -1.0/4, 1.0/4, 1.0/4]
        self.dk_yaw = [-1.0/(4*(self.alpha+self.beta)), -1.0/(4*(self.alpha+self.beta)),
                       -1.0/(4*(self.alpha+self.beta)), -1.0/(4*(self.alpha+self.beta))]

        # ApproximateTimeSynchronizer setup
        self.FLW_fb = message_filters.Subscriber('roboteq_driver_FLW/feedback', Feedback, queue_size=2)
        self.FRW_fb = message_filters.Subscriber('roboteq_driver_FRW/feedback', Feedback, queue_size=2)
        self.RLW_fb = message_filters.Subscriber('roboteq_driver_RLW/feedback', Feedback, queue_size=2)
        self.RRW_fb = message_filters.Subscriber('roboteq_driver_RRW/feedback', Feedback, queue_size=2)

        self.async = approxsync.ApproximateSynchronizer(1, [self.FLW_fb, self.FRW_fb,
                                                            self.RLW_fb, self.RRW_fb], 1)

        self.async.registerCallback(self.callback)

        # variable to save updated position and orientation
        self.pose = Pose()

        # delta time between callbacks
        self.dt = 1.0/50      # 50Hz, default publish rate of Feedback messages

        self.pub = rospy.Publisher('odom', Odometry, queue_size=1)

        self.tf_br = tf2_ros.TransformBroadcaster()

        self.last_cb = rospy.Time.now()

    def callback(self, flw, frw, rlw, rrw):

        odom = Odometry()

        tf = TransformStamped()

        current_time = rospy.Time.now()

        dt = current_time.to_sec() - self.last_cb.to_sec()
        self.last_cb = current_time

        rospy.loginfo("Time between callbacks = %f", dt)

        odom.header.stamp = current_time
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_id

        tf.header.stamp = current_time
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = self.child_id

        # x axis linear velocity
        # multiply by -1.0 because of wiring
        vX = -1.0 / self.gb_ratio * self.wheel_radius * (self.dk_x[0]*flw.measured_velocity +
                                                         self.dk_x[1]*frw.measured_velocity +
                                                         self.dk_x[2]*rlw.measured_velocity +
                                                         self.dk_x[3]*rrw.measured_velocity)

        odom.twist.twist.linear.x = vX

        # y axis linear velocity
        # multiply by -1.0 because of wiring
        vY = -1.0 / self.gb_ratio * self.wheel_radius * (self.dk_y[0]*flw.measured_velocity +
                                                         self.dk_y[1]*frw.measured_velocity +
                                                         self.dk_y[2]*rlw.measured_velocity +
                                                         self.dk_y[3]*rrw.measured_velocity)

        odom.twist.twist.linear.y = vY

        # yaw angular velocity
        # multiply by -1.0 because of wiring
        vYaw = -1.0 / self.gb_ratio * self.wheel_radius * (self.dk_yaw[0]*flw.measured_velocity +
                                                           self.dk_yaw[1]*frw.measured_velocity +
                                                           self.dk_yaw[2]*rlw.measured_velocity +
                                                           self.dk_yaw[3]*rrw.measured_velocity)

        odom.twist.twist.angular.z = vYaw

        # update pose orientation
        # convert orientation to RPY
        # euler = [roll, pitch, yaw]
        euler = tf_conversions.transformations.euler_from_quaternion([self.pose.orientation.x,
                                                                      self.pose.orientation.y,
                                                                      self.pose.orientation.z,
                                                                      self.pose.orientation.w])

        # update x position
        self.pose.position.x += self.dt * (vX * cos(euler[2]) - vY * sin(euler[2]))
        # update y position
        self.pose.position.y += self.dt * (vX * sin(euler[2]) + vY * cos(euler[2]))

        # add yaw variation to RPY and convert orientation back to quaternion
        q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, euler[2] + vYaw * self.dt)
        self.pose.orientation.x = q[0]
        self.pose.orientation.y = q[1]
        self.pose.orientation.z = q[2]
        self.pose.orientation.w = q[3]

        odom.pose.pose = self.pose

        # update tf
        tf.transform.translation.x = self.pose.position.x
        tf.transform.translation.y = self.pose.position.y
        tf.transform.translation.z = 0.0

        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]

        # publish odom and send transform
        self.pub.publish(odom)

        self.tf_br.sendTransform(tf)


if __name__ == '__main__':

    try:
        rospy.init_node('wm_odometry_feedback_node')

        OdometryFeedback()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass


