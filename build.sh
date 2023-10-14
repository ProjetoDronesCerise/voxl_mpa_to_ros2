#!/bin/bash
#
# builds everything without installing
#
# Modal AI Inc. 2023
# author: zachary.lowell@ascendengineer.com


set -e

current_dir=$(pwd)
cd $current_dir/colcon_ws

AVAILABLE_PLATFORMS="qrb5165"

print_usage(){
    echo ""
    echo " Build the current project based on platform target."
    echo ""
    echo " Usage:"
    echo ""
    echo "  ./build.sh qrb5165"
    echo "        Build 64-bit binaries for qrb5165"
    echo ""
    echo ""
}



case "$1" in
    qrb5165)
        ROS_DIST="foxy"
        ;;

    *)
        print_usage
        exit 1
        ;;
esac

. /opt/ros/${ROS_DIST}/setup.bash

colcon build
cd $current_dir

mkdir -p misc_files/opt/ros/${ROS_DIST}/mpa_to_ros2
cp -r colcon_ws/install/  misc_files/opt/ros/${ROS_DIST}/mpa_to_ros2
cp -r colcon_ws/build/ misc_files/opt/ros/${ROS_DIST}/mpa_to_ros2
chmod -R 777 misc_files
