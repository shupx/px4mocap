#!/usr/bin/bash
source /opt/ros/noetic/setup.bash

source /home/cm/px4mocap/devel/setup.bash

sleep 5 # waiting for roscore to finish launching

roslaunch px4mocap oled.launch
