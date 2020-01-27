#!/bin/bash
#export ROS_MASTER_URI=http://192.168.0.10:11311/
#export ROS_IP=192.168.0.4
#rosparam set joy_arm/dev "/dev/input/js3"
#rosrun joy joy_node __name:=joy_arm
#cd caesar/
. ./export_source.sh
roslaunch joy.launch
