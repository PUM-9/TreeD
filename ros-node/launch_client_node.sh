#!/bin/bash
args="$@"
set -- ""
source /opt/ros/indigo/setup.bash
source devel/setup.bash
rosrun treeD_client_pkg treeD_client_node.py $args
