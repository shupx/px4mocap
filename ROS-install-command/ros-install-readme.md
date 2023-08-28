## 安装ROS1极简教程

1. 添加镜像源：

```bash
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```

2. **这步可以不做**，因为ros.key已经下载好了放在`px4mocap/ROS-install-command/`文件夹下了。

```bash
sudo apt install curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

wget http://packages.ros.org/ros.key
```

3. 添加ros.key：

```bash
# git clone https://gitee.com/shu-peixuan/px4mocap.git
# cd px4mocap/ROS-install-command/
# 如果还没有clone px4mocap仓库，可以直接用wget下载px4mocap仓库中的这个ros.key文件
wget https://gitee.com/shu-peixuan/px4mocap/raw/85b46df9912338f775949903841160c873af4a1d/ROS-install-command/ros.key
sudo apt-key add ros.key
sudo apt-get update --fix-missing
```

4. 安装ROS（ubuntu20安装ros-noetic，ubuntu18安装ros-melodic）：

```bash
# 如果安装melodic版本，下面noetic对应改成melodic
sudo apt install ros-noetic-desktop # 桌面端建议安装这个更全的版本
# sudo apt install ros-noetic-ros-base # 机载端安装这个最简单的版本
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc # 添加环境路径
```



## 安装mavros

如果安装melodic版本，下面noetic对应改成melodic：

```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras -y
```

```bash
# cd px4mocap/ROS-install-command/
# 如果还没有clone px4mocap仓库，可以直接用wget下载px4mocap仓库中的这个install_geographiclib_datasets.sh文件
wget https://gitee.com/shu-peixuan/px4mocap/raw/85b46df9912338f775949903841160c873af4a1d/ROS-install-command/install_geographiclib_datasets.sh
sudo chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh #这步要联网下载，科学上网更快
```



## 安装vrpn的ROS包

```bash
sudo apt install ros-noetic-vrpn-client-ros -y
```
