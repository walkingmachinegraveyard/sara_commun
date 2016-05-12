#!/bin/sh

# Adding source and key
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116

# To get the latest package lists
apt-get update

# ROS installation
apt-get install ros-indigo-desktop-full

# Package installation
apt-get install ros-indigo-openni2-launch ros-indigo-urg-node ros-indigo-xsens-driver \
ros-indigo-joystick-drivers ros-indigo-navigation ros-indigo-pocketshinx ros-indigo-rosserial \
ros-indigo-roboteq-driver ros-indigo-roboteq-diagnostics ros-indigo-roboteq-msgs ros-indigo-smach \
ros-indigo-rtabmap-ros ros-indigo-gazebo-ros ros-indigo-slam-gmapping ros-indigo-map-laser \
ros-indigo-cob-perception-common ros-indigo-moveit-full ros-indigo-geographic-info \
ros-indigo-zbar-ros ros-indigo-dynamixel-motor
