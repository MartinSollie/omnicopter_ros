#!/bin/bash

if [ "$(whoami)" != "root" ]; then
	echo "Run as root!"
	exit 1
fi


# ROS
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

apt-get update
nice apt-get upgrade

apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
rosdep init
sudo -u $USER rosdep update

mkdir -p ~/ros_catkin_ws
cd ~/ros_catkin_ws

rosinstall_generator ros_comm geometry_msgs sensor_msgs --rosdistro kinetic --deps --wet-only --exclude collada_parser collada_urdf --tar > kinetic-ros_comm-wet.rosinstall
wstool init src kinetic-ros_comm-wet.rosinstall

# Eigen

# RTImulib