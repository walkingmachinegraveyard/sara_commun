#!/usr/bin/env python

# SARA arm teleoperation
# input: sensor_msgs/Joy

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
import os, sys
import time

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from math import pi, sin, cos, sqrt, atan2
from subprocess import Popen, PIPE

import roslib; roslib.load_manifest('robotiq_c_model_control')
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
from time import sleep


try:
    controler_type = sys.argv[1]
except Exception:
    print("no controler type defined")


if( controler_type == "ps3" ):
    B1 = 13
    B2 = 15
    B3 = 12
    B4 = 10
    B5 = 11
    B6 = 0
    B7 = 5
    B8 = 8
    B9 = 9
    B0 = 14
else:
    B1 = 1
    B2 = 2
    B3 = 3
    B4 = 4
    B5 = 5
    B6 = 6
    B7 = 7
    B8 = 8
    B9 = 9
    B0 = 0
   

def genCommand(char, command):
    """Update the command according to the character entered by the user."""    
        
    if char == 'a':
        command = outputMsg.CModel_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 255
        command.rFR  = 150

    if char == 'r':
        command = outputMsg.CModel_robot_output();
        command.rACT = 0

    if char == 'c':
        command.rPR = 255

    if char == 'o':
        command.rPR = 0   

    #If the command entered is a int, assign this value to rPRA
    try: 
        command.rPR = int(char)
        if command.rPR > 255:
            command.rPR = 255
        if command.rPR < 0:
            command.rPR = 0
    except ValueError:
        pass                    
        
    if char == 'f':
        command.rSP += 25
        if command.rSP > 255:
            command.rSP = 255
            
    if char == 'l':
        command.rSP -= 25
        if command.rSP < 0:
            command.rSP = 0
            
    if char == 'i':
        command.rFR += 25
        if command.rFR > 255:
            command.rFR = 255
            
    if char == 'd':
        command.rFR -= 25
        if command.rFR < 0:
            command.rFR = 0

    return command


 
class MecanumTeleop:

    def __init__(self):

        self.sub = rospy.Subscriber('joy', Joy, self.callback)
        self.hand_pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output)
        self.command = outputMsg.CModel_robot_output();
        #print "To use the wireless ps3 controler, execute ' $ sixad --start '"
        os.system('cd /opt/kinova/GUI ; ./DevelopmentCenter')


    def callback(self, joy):

        global B1
        global B2
        global B3
        global B4
        global B5
        global B6
        global B7
        global B8
        global B9
        global B0

        # movement Forearm Vectical (Motor4)
        if joy.buttons[B0]*joy.axes[1] >= 0.5: # joy.buttons[B0]=button A        
            os.system("xte 'keydown 8'")

        if joy.buttons[B0]*joy.axes[1] <= -0.5: # joy.buttons[B0]=button A        
            #print "A+v"
            os.system("xte 'keydown 7'")
            
        # movement Forearm Horizontal (Motor3)
        if joy.buttons[B0]*joy.axes[0] >= 0.5: # joy.buttons[B0]=button A
            os.system("xte 'keydown 5'")

        if joy.buttons[B0]*joy.axes[0] <= -0.5: # joy.buttons[B0]=button A
            os.system("xte 'keydown 6'")

        # movement Arm Vectical (Motor4)
        if joy.buttons[B1]*joy.axes[1] >= 0.5: # joy.buttons[B1]=button B
            os.system("xte 'keydown 4'")
            
        if joy.buttons[B1]*joy.axes[1] <= -0.5: # joy.buttons[B1]=button B
            os.system("xte 'keydown 3'")

        # movement Arm Horizontal (Motor3)
        if joy.buttons[B1]*joy.axes[0] >= 0.5: # joy.buttons[B1]=button B
            os.system("xte 'keydown 2'")
            
        if joy.buttons[B1]*joy.axes[0] <= -0.5: # joy.buttons[B1]=button B
            os.system("xte 'keydown 1'")
            
        # movement Wrist Roll (Motor5)
        if joy.buttons[B5] == 1: # joy.buttons[B1]=button B
            os.system("xte 'keydown 9'")

        if joy.buttons[B4] == 1: # joy.buttons[B1]=button B
            os.system("xte 'keydown 0'")
                        
        # Gripper open/close
        if joy.buttons[B6] == 1:
            self.command = genCommand('r', self.command)
            self.hand_pub.publish(self.command)
            rospy.sleep(0.1)
            self.command = genCommand('a', self.command)
            self.hand_pub.publish(self.command)
            rospy.sleep(0.1)
            
        time.sleep(0.05)

        os.system("xte 'keyup 1'")
        os.system("xte 'keyup 2'")
        os.system("xte 'keyup 3'")
        os.system("xte 'keyup 4'")
        os.system("xte 'keyup 5'")
        os.system("xte 'keyup 6'")
        os.system("xte 'keyup 7'")
        os.system("xte 'keyup 8'")
        os.system("xte 'keyup 9'")
        os.system("xte 'keyup 0'")

if __name__ == '__main__':

    try:
        rospy.init_node('wm_mecanum_teleop_node')
        MecanumTeleop()                
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
        
        
        
        
        
        
        
        
        
        
        
        
        

        
        
        
