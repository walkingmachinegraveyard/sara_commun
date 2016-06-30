#!/bin/bash

sleep 5
ID_RVIZ=$(pidof rviz)
mkdir /tmp/snpsht_tmp
cd /tmp/snpsht_tmp
mkdir snapshots
cd snapshots

name=camera
if [[ -e $name.png ]] ; then
    i=1
    while [[ -e $name-$i.png ]] ; do
        let i++
    done
    name=$name-$i
fi
touch $name.png  

export DISPLAY=:1 
import -window root -display $DISPLAY -screen /tmp/snpsht_tmp/snapshots/$name.png
kill $ID





