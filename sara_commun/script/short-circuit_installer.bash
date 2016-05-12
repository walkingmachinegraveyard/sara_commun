#!/bin/sh

# Adding source and key
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116

# To get the latest package lists
apt-get update

# ROS installation
apt-get install ros-indigo-desktop-full -y

# Package installation
apt-get install ros-indigo-openni2-launch -y
apt-get install ros-indigo-urg-node -y
apt-get install ros-indigo-xsens-driver -y
apt-get install ros-indigo-joystick-drivers -y
apt-get install ros-indigo-navigation -y
apt-get install ros-indigo-pocketshinx -y
apt-get install ros-indigo-rosserial -y
apt-get install ros-indigo-roboteq-driver -y
apt-get install ros-indigo-roboteq-diagnostics -y
apt-get install ros-indigo-roboteq-msgs -y
apt-get install ros-indigo-smach -y
apt-get install ros-indigo-rtabmap-ros -y
apt-get install ros-indigo-gazebo-ros -y
apt-get install ros-indigo-slam-gmapping -y
apt-get install ros-indigo-map-laser -y
apt-get install ros-indigo-cob-perception-common -y
apt-get install ros-indigo-moveit-full -y
apt-get install ros-indigo-geographic-info -y
apt-get install ros-indigo-zbar-ros -y

