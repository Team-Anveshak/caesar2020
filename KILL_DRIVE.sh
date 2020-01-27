#!/bin/bash
sshpass -p "anveshak" ssh -o StrictHostKeyChecking=no anveshak@192.168.0.10 'bash -c "source /opt/ros/melodic/setup.bash && rosnode kill /drive"'
