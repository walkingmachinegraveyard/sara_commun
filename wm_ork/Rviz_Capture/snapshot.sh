#!/bin/bash

sleep 5
ID_RVIZ=$(pidof rviz)
cd /home/samuel/Pictures/snapshots_rviz

name=camera
if [[ -e $name.png ]] ; then
    i=1
    while [[ -e $name-$i.png ]] ; do
        let i++
    done
    name=$name-$i
fi
touch $name.png  

DISPLAY=":0.0"; export DISPLAY
import -window root -display $DISPLAY -screen /home/samuel/Pictures/snapshots_rviz/$name.png
kill $ID_RVIZ
exit




