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

import os

import time


from sensor_msgs.msg import Joy

from geometry_msgs.msg import Twist

from math import pi, sin, cos, sqrt, atan2

from subprocess import Popen, PIPE






class MecanumTeleop:



    def __init__(self):


        self.sub = rospy.Subscriber('joy', Joy, self.callback)
        os.system('cd /opt/kinova/GUI ; ./DevelopmentCenter')
        #print( "The controler is ready. You can go and start the Kinova DevelopementCenter at:\n/opt/kinova/GUI/DevelopmentCenter" )



    def callback(self, joy):



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






        # movement Forearm Vectical (Motor4)
        if joy.buttons[0]*joy.axes[1] >= 0.5: # joy.buttons[0]=button A
        
            os.system("xte 'keydown 8'")


        if joy.buttons[0]*joy.axes[1] <= -0.5: # joy.buttons[0]=button A
        
            #print "A+v"

            os.system("xte 'keydown 7'")
            


        # movement Forearm Horizontal (Motor3)
        if joy.buttons[0]*joy.axes[0] >= 0.5: # joy.buttons[0]=button A

            os.system("xte 'keydown 5'")


        if joy.buttons[0]*joy.axes[0] <= -0.5: # joy.buttons[0]=button A

            os.system("xte 'keydown 6'")







        # movement Arm Vectical (Motor4)
        if joy.buttons[1]*joy.axes[1] >= 0.5: # joy.buttons[1]=button B

            os.system("xte 'keydown 4'")
            

        if joy.buttons[1]*joy.axes[1] <= -0.5: # joy.buttons[1]=button B

            os.system("xte 'keydown 3'")



        # movement Arm Horizontal (Motor3)
        if joy.buttons[1]*joy.axes[0] >= 0.5: # joy.buttons[1]=button B

            os.system("xte 'keydown 2'")
            

        if joy.buttons[1]*joy.axes[0] <= -0.5: # joy.buttons[1]=button B

            os.system("xte 'keydown 1'")
            






        # movement Wrist Roll (Motor5)
        if joy.buttons[5] == 1: # joy.buttons[1]=button B

            os.system("xte 'keydown 9'")


        if joy.buttons[4] == 1: # joy.buttons[1]=button B

            os.system("xte 'keydown 0'")
            










if __name__ == '__main__':



    try:

        rospy.init_node('wm_mecanum_teleop_node')


        MecanumTeleop()
        
        
        rospy.spin()



    except rospy.ROSInterruptException:

        pass
        
        
        
        
        
        
        
        
        
        
        
        
        

        
        
        
