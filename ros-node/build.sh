#!/bin/bash
source /opt/ros/indigo/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3.4 -DPYTHON_INCLUDE_DIR=/usr/include/python3.4 -DPYTHON_LIBRARY=/usr/lib/libpython3.4.so
