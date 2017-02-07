#!/bin/bash
source /opt/ros/indigo/setup.bash
source devel/setup.bash
roscore > /dev/null &
rosrun treeD_control_pkg treeD_control_node.py

