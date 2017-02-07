#/bin/bash
PUPFILES=/etc/puppet/files
OPTPATH=/opt/treed

# Install linear drive
cd $PUPFILES/treeD/hardware/technosoft-linear-drive
python3 setup.py install

# Install network-communication
cd $PUPFILES/treeD/network-communication
python3 setup.py install

# Build and install ros thingies
cd $OPTPATH/ros-node
source /opt/ros/indigo/setup.bash
catkin_make\
    -DPYTHON_EXECUTABLE=/usr/bin/python3.4\
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.4\
    -DPYTHON_LIBRARY=/usr/lib/libpython3.4.so\
