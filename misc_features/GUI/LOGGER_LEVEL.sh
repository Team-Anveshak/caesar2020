#!/bin/bash
export ROS_MASTER_URI=http://192.168.0.10:11311/
export ROS_IP=192.168.0.2
rosrun rqt_logger_level rqt_logger_level
