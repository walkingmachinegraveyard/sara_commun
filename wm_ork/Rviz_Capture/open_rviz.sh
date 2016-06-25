#!/bin/bash
cd ~/sara_ws/src/sara_commun/wm_ork/Rviz_Capture/
(./snapshot.sh) & (rosrun rviz rviz -d camera_object_preset.rviz)






