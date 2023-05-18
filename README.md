### Introduction

This ros package is used to read mocap pose from vrpn and send the pose data to px4 flight controller through mavros at a frequency of 60Hz.

### Note!!!
This package is default for motion capture vrpn system with distance unit 'm'. For systems with distance unit 'mm', you should change the `mocap_unit` into "mm" in `mocap2px4.launch`.


### Build:

This ros package should be built on the onboard computer of the drone.


```C
git clone https://gitee.com/shu-peixuan/px4mocap.git
cd px4mocap/
catkin_make
source devel/setup.bash
```


Please install or update vrpn client to the newest version like (`melodic` for ubuntu18 and `noetic` for ubuntu20):

```C
sudo apt install ros-melodic-vrpn-client-ros
```

If you have not installed mavros, then you can follow the command in `./ROS-install-command` to install mavros first (`melodic` for ubuntu18 and `noetic` for ubuntu20):

```bash
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras -y

cd ROS-install-command

sudo chmod +x install_geographiclib_datasets.sh

sudo ./install_geographiclib_datasets.sh
```

### Launch:

To launch the px4mocap, please connect to the same WIFI with mocap server. Change the  `fcu_url` (onboard computer port), `gcs_url` (QGroundControl IP),  `server` IP (motion capture system vrpn IP)  and desired  `rigidpose_topic` (vrpn rigid body pose topic)  in `mocap2px4.launch`. Then 

```C
roslaunch px4mocap mocap2px4.launch
```
Then the xyz position and yaw angle (quaternion) will be sent to px4 flight controller(pixhawk).

Or you can use the shell script to launch quickly:

```bash
cd src/px4mocap/launch
# change the `source` path in mocap2px4.sh first
./mocap2px4.sh
```

Similarly, you can launch `mavros.launch`|`mavros.sh` and `vrpn2mavros.launch`|`vrpn2mavros.sh` separately in your need. 


### Result:

Remember to check that the mocap pose data were transferred to the pixhawk properly:

```C
rostopic echo /mavros/vision_pose/pose
rostopic echo /mavros/local_position/pose
```
Note that `rostopic echo` is cpu and RAM consuming. Do not keep `rostopic echo` on resource-limited onboard computer like raspberrypi.

### Attention:

Be sure to properly set `EKF2_AID_MASK` (`EKF2_EV_CTRL` in px4 v1.14.0 and newer version) and `EKF2_HGT_REF` parameters of PX4 on QgroundControl to receive these pose messages.

Be sure to align body x axis to the heading of the drone when creating rigid body in Seeker(mocap software). Otherwise, the yaw angle calculated by mocap system will not be the angle from world x to the heading of the drone(Mavros use ENU world frame and FLU body frame).


### Auto Start:

Furthur more, you can set `mavros` or `vrpn2mavros` as auto-startup service on boot using `systemctl`. **It is worth noting that these ROS nodes should be launched a few seconds after `roscore.service` has been launched.** Otherwise, if the first ROS node which auto-start roscore by default is shut down occassionally, other ROS nodes will be shut down as well, which is unwanted.

```bash
cd px4mocap/src/px4mocap/launch
# change `ExecStart` path in **.service and **.sh first (must use absolute path).
# service文件中不要出现中文和注释！
sudo cp roscore.service /etc/systemd/system/roscore.service
sudo cp mavros.service /etc/systemd/system/mavros.service 
sudo cp vrpn2mavros.service /etc/systemd/system/vrpn2mavros.service 

sudo systemctl enable roscore.service mavros.service vrpn2mavros.service # enable service on boot
sudo systemctl daemon-reload # reload service

# other operations:
sudo systemctl status **.service # echo the output of the service
sudo systemctl disable **.service # disable service on boot
sudo systemctl start **.service # start service once
sudo systemctl stop **.service # stop the active service
```

Please feel free to ask questions in Issue.

### 附注

`vncserver-command/`中附有vnc远程桌面配置说明，`ROS-install-command`中附有安装ROS、mavros、vrpn等说明，详情请进入对应文件夹浏览。
