#!/bin/bash
. ~/caesar2020/devel/setup.bash
export ROS_MASTER_URI='http://192.168.1.11:11311' 
export ROS_IP='192.168.1.11'
rosparam load ~/caesar2020/ports.yaml
rosparam load ~/caesar2020/traversal/config/traversal_param.yaml
