#!/usr/bin/env python

# SARA teleoperation
# input: sensor_msgs/Joy
# output: geometry_msgs/Twist

# tutorial on how to setup joy_node http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

# axes: [LeftJoystickEastWest LeftJoystickNorthSouth LeftTrigger RightJoystickEastWest  ...
#        RightJoystickNorthSouth RightTrigger PadEastWest PadNorthSouth]
# For joysticks and directional pad: West==1.0, East==-1.0, North==1.0, South==-1
# For triggers: fully pushed==-1.0, at rest==1.0
# joysticks and triggers can take any value between [-1.0, 1.0], 0.0 is at rest state
# directional pad values are 0.0 or 1.0
# buttons: [a, b, x, y, LeftBumper, RightBumper, back, start, XboxButton, LeftJoystickPress, RightJoystickPress]
# all buttons values are either 1.0 (button is pushed) or 0.0 (button is not pushed)

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from math import pi, sin, cos, sqrt, atan2


class MecanumTeleop:

    def __init__(self):

        self.sub = rospy.Subscriber('joy', Joy, self.callback)

        self.pubFLW = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.maxLinearVelocity = float(rospy.get_param('max_linear_vel', 1))
        # max angular velocity, in rad/s
        divisor = rospy.get_param('angular_vel_div', 6)
        self.maxAngularVelocity = pi/divisor

    def callback(self, joy):

        twist = Twist()

        # linear velocity
        # button A must be pressed to allow movement
        vLinear = float(joy.buttons[0]) * sqrt(joy.axes[0]**2 + joy.axes[1]**2)

        # movement orientation
        Heading = atan2(joy.axes[0], joy.axes[1])

        # x axis linear velocity
        twist.linear.x = self.maxLinearVelocity * vLinear * cos(Heading)
        # y axis linear velocity
        twist.linear.y = self.maxLinearVelocity * vLinear * sin(Heading)

        # YAW axis rotational velocity
        twist.angular.z = self.maxAngularVelocity * joy.buttons[0] * (joy.axes[5] - joy.axes[2]) / 2.0

        self.pubFLW.publish(twist)


if __name__ == '__main__':

    try:
        rospy.init_node('wm_mecanum_teleop_node')

        MecanumTeleop()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
