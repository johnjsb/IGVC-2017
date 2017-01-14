#!/bin/bash

#######################################################################
# STOP: If the dependency you want to add is required for the project # 
#       to build, it should be added as a rosdep. This script should  #
#       only contain other dependecies, like those required for gazebo#
#######################################################################

######################################################################
# This script will download and install dependencies for the project #
######################################################################

echo "================================================================"
echo "Installing other ROS dependencies..."
echo "================================================================"

sudo apt-get install -y\
    libserial-dev \
    ros-kinetic-xacro \
    ros-kinetic-controller-manager \
    ros-kinetic-gazebo-ros-control \
    ros-kinetic-ros-controllers \
    ros-kinetic-ros-control \
    ros-kinetic-effort-controllers \
    ros-kinetic-hector-gazebo

echo "================================================================"
echo "Finished installing other ROS dependencies."
echo "================================================================"
