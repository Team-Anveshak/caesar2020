#!/bin/bash
export ROS_MASTER_URI=http://192.168.0.10:11311/
export ROS_IP=192.168.0.6
. ./devel/setup.bash 
rqt
