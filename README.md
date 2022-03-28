### Introduction

This ros package is used to read mocap pose from vrpn and send the pose data to px4 flight controller through mavros at a frequency of 50Hz.

### Note!!!
This package is for motion capture vrpn system with distance unit 'mm', for systems with distance unit 'm', you should change the code by yourself!!


### Build:

This ros package should be built on the onboard computer of the drone.


```C
git clone https://gitee.com/shu-peixuan/px4mocap.git
cd px4mocap
catkin_make
source devel/setup.bash
```


Please install or update vrpn client to the newest version like: 

```C
sudo apt install ros-melodic-vrpn-client-ros
```

### Launch:

To launch the px4mocap, please connect to the same WIFI with mocap server. Change the  _fcu_url_ (onboard computer port),  _server IP_  and desired  _rigidpose_topic_  in mocap2px4.launch. Then 

```C
roslaunch px4mocap mocap2px4.launch
```
Then the xyz position and yaw angle (quaternion) will be sent to px4 flight controller(pixhawk).

Remember to check that the mocap pose data were transferred to the pixhawk properly:

```C
rostopic echo /mavros/vision_pose/pose
rostopic echo /mavros/local_position/pose
```


### Attention:


Be sure to properly set AID_MASK and HGT parameters of PX4 on QgroundControl to reveive these pose messages.

Be sure to align body x axis to the heading of the drone when creating rigid body in Seeker(mocap software). Otherwise, the yaw angle calculated by mocap system will not be the angle from world x to the heading of the drone(Mavros use ENU world frame and FLU body frame).

Please feel free to ask questions in Issue


