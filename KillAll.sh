#!/bin/bash
#sshpass -p "anveshak" ssh -Y -o StrictHostKeyChecking=no anveshak@192.168.0.10 'bash -c "source /opt/ros/melodic/setup.bash && export ROS_IP=192.168.0.10 && gedit caesar2020/ports.yaml"'
sshpass -p "anveshak" ssh -Y -o StrictHostKeyChecking=no anveshak@192.168.0.10 'bash -c "source /opt/ros/melodic/setup.bash && export ROS_IP=192.168.0.10 && rosnode kill --all"'
