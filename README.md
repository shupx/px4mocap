### Introduction

This ros package is used to read mocap pose from vrpn and send the pose data to px4 flight controller through mavros at a frequency of 60Hz.

### Note!!!
This package is default for motion capture vrpn system with distance unit 'm'. For systems with distance unit 'mm', you should change the `mocap_unit` into "mm" in `mocap2px4.launch`.


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

Or you can use the shell script to launch quickly:

```bash
cd src/px4mocap/launch
./mocap2px4.sh
```

Remember to check that the mocap pose data were transferred to the pixhawk properly:

```C
rostopic echo /mavros/vision_pose/pose
rostopic echo /mavros/local_position/pose
```
Note that `rostopic echo` is cpu and RAM consuming. Do not keep `rostopic echo` on resource-limited onboard computer like raspberrypi.

### Attention:


Be sure to properly set AID_MASK and HGT parameters of PX4 on QgroundControl to receive these pose messages.

Be sure to align body x axis to the heading of the drone when creating rigid body in Seeker(mocap software). Otherwise, the yaw angle calculated by mocap system will not be the angle from world x to the heading of the drone(Mavros use ENU world frame and FLU body frame).

Please feel free to ask questions in Issue


