#!/bin/bash

# if foxy is installed, load the mpa_to_ros2 setup script which will
# also source the Foxy setup script

FOXYFILE="/opt/ros/foxy/local_setup.bash"
MPA_TO_ROS2_FILE="/opt/ros/foxy/mpa_to_ros2/install/setup.bash"

if [ -f $FOXYFILE ]; then
	source $FOXYFILE
	source $MPA_TO_ROS2_FILE
fi

export ROS_HOME=/opt/ros/foxy
