#!/bin/bash
cd ~/sara_ws/src/sara_commun/wm_ork/Rviz_Capture/
Xvfb :1 & export DISPLAY=:1
(./snapshot.sh) & (DISPLAY=:1 rosrun rviz rviz -d camera_object_preset.rviz)






