#!/bin/bash
cd ${HOME}/catkin_workspace/
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${HOME}/catkin_workspace/devel/setup.bash
echo "launching application, please wait..."
roslaunch whi_rc_bridge whi_rc_bridge_ns.launch robot_name:=whi

