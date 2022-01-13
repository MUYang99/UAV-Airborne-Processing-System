#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.sh
export ROS_HOSTNAME=tx2
export ROS_MASTER_URI=http://192.168.2.12:11311

roslaunch darknet_ros darknet_ros.launch
