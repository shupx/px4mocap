#!/usr/bin/bash
source /opt/ros/noetic/setup.bash

source /home/orangepi/px4mocap/devel/setup.bash

roslaunch px4mocap mocap2px4.launch
