#!/bin/bash
cd ${HOME}/ros2_ws/
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${HOME}/ros2_ws/install/setup.bash
echo "launching application, please wait..."
ros2 launch whi_rc_bridge launch.py robot_name:='whi'
