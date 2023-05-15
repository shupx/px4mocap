## 为xubuntu安装和配置VNCviewer远程桌面

树莓派cm4安装了ubuntu20.04（xubuntu桌面，不是gnome桌面），需要实现vnc远程桌面登录树莓派。此处选择tigervnc，其他的vnc比如tightvnc和vnc4server貌似不好用。

### VNC server

1. 在树莓派上安装tigerVNC的server端。

    ```bash
    sudo apt install xfce4
    sudo apt install tigervnc-standalone-server tigervnc-common
    ```

2. 第一次启动vncserver，需要根据提示配置密码(123456)，不要sudo

    ```bash
    vncserver
    ```
    此时也会在本地`~/.vnc`文件夹下创建ubuntu:1.pid和passwd等文件

3. 建立~/.vnc/xstartup配置文件：

    ```bash
    cd px4mocap/vncserver-command
    sudo cp xstartup ~/.vnc/xstartup
    ```

4. 常用操作：

    ```bash
    vncserver -list # 列出所有server
    vncserver -kill :1 # 杀掉1号server
    vncserver -geometry 1200x800 -localhost no #设置分辨率和非本地启动（局域网内其他client才能连接）
    ```

5. 设置开机自启动：

   把`px4mocap/vncserver-command/vncserver.service`中的用户名cm改成实际的用户名（注意shell命令中vncserver必须要指定用户名后启动），然后用systemctl的开机自启动管理：
   
   ```bash
   cd px4mocap/vncserver-command
   sudo cp vncserver.service /etc/systemd/system/vncserver.service
   sudo systemctl daemon-reload
   sudo systemctl enable vncserver.service # 加入启动项
   # sudo systemctl start vncserver.service # 一次性启动
   # sudo systemctl disable vncserver.service # 删除启动项

### VNC viewer

ubuntu下好像可以直接用apt安装`tigervnc-viewer`。在windows端建议安装一些第三方的vncviewer，比如realVNC的[vncviewer](https://www.realvnc.com/en/connect/download/viewer/)。但我建议安装的是功能更全，远程桌面更流畅的[MobaXterm](https://mobaxterm.mobatek.net/)。它不仅支持vnc远程桌面，还支持ssh，telnet等。

点击MobaXterm的左上角Sessions -> New Session。输入vncserver的IP以及端口5901，输入vnc密码123456进行连接。如果连接多个vnc桌面，部分功能如WIFI设置和firefox浏览器功能只在某一个vnc桌面有用。



VNCserver在画面静止时占用cpu很小，但画面有变化时占用服务器端cpu很大。其实相较于远程视频桌面，MobaXterm的ssh功能就可以满足大部分的远程需求了，它左方的文件浏览器支持用本地电脑的文件编辑器打开服务器端的文件进行编辑。
