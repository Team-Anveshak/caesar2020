#!/bin/bash
export ROS_MASTER_URI=http://192.168.0.10:11311/
export ROS_IP=192.168.0.6
rosparam set joy_node/dev "/dev/input/js2"
rosrun joy joy_node 

