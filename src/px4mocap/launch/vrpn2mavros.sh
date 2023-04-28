#!/usr/bin/bash

source /opt/ros/noetic/setup.bash

source /home/cm/px4mocap/devel/setup.bash

while true
do
	ping -c 2 192.168.31.105 >/dev/null
	if [ $? -eq 0 ];then
		roslaunch px4mocap vrpn2mavros.launch
		break
	else
		echo "could not ping success"
	fi
	sleep 2
done
