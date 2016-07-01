#!/bin/bash

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

import -window root -display $DISPLAY -screen /tmp/snpsht_tmp/snapshots/$name.png
exit




