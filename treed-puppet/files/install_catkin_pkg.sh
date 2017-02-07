#!/bin/bash
git clone https://github.com/ros-infrastructure/rospkg.git /root/rospkg -b 1.0.38
cd /root/rospkg
python3 setup.py install

