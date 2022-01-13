#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.sh
export ROS_HOSTNAME=tx2
export ROS_MASTER_URI=http://192.168.2.12:11311
sleep 10
roslaunch dji_sdk_demo dji_sdk_client.launch | tee M100.txt
