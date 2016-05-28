#!/bin/sh
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"
sleep 0.5
roslaunch roboteq_driver wm_drive1.launch
sleep 0.5
roslaunch roboteq_driver wm_drive2.launch
sleep 0.5
roslaunch roboteq_driver wm_drive3.launch
sleep 0.5
roslaunch roboteq_driver wm_drive4.launch
