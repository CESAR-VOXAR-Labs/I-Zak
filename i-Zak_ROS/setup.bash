#!/usr/bin/env bash
# generated from catkin/cmake/templates/setup.bash.in

CATKIN_SHELL=bash

# source setup.sh from same directory as this file
_CATKIN_SETUP_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
. "$_CATKIN_SETUP_DIR/setup.sh"

export ROS_PACKAGE_PATH=$HOME/catkin_ws/install/share:$HOME/catkin_ws/install/stacks:$HOME/ros:/opt/ros/indigo/share:/opt/ros/indigo/stacks
