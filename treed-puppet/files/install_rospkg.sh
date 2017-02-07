#!/bin/bash
git clone https://github.com/ros-infrastructure/catkin_pkg /root/catkin_pkg -b 0.2.10
cd /root/catkin_pkg 
python3 setup.py install

