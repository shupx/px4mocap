#!/usr/bin/python3
# coding=utf-8

# Peixuan Shu 
# 2023.5.30

#################################### ROS init ############################################
import rospy
from mavros_msgs.msg import State
from mavros_msgs.msg import GPSRAW
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
import numpy as np
from tf.transformations import euler_from_quaternion

ENUpos = np.zeros(3,dtype=float) # ENU pos (m)
yaw = 0.0 # from east (deg)
px4_mode = "" # POSCTL ...
px4_state = "" # READY ...
gps_fix = "" # 3Dfix ...
gps_satellites = 0 # visible satellites
bat_volt = 0.0 # V
bat_cur = 0.0 # A
bat_perct = 0.0 # %

def mavros_state_cb(data):
    # refer to https://mavlink.io/zh/messages/common.html
    # refer to https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/State.html
    global px4_mode, px4_state
    px4_mode = data.mode
    system_status = data.system_status
    if system_status == 0:
        px4_state = "UNINIT"
    elif system_status == 1:
        px4_state = "BOOT"
    elif system_status == 2:
        px4_state = "CALIBRATING"
    elif system_status == 3:
        px4_state = "STANDBY" # ready to launch
    elif system_status == 4:
        px4_state = "ACTIVE" # flying
    elif system_status == 5:
        px4_state = "CRITICAL" # non-normal but still navigate
    elif system_status == 6:
        px4_state = "EMERGENCY"   
    elif system_status == 7:
        px4_state = "POWEROFF"   
    elif system_status == 8:
        px4_state = "TERMINATE"
    rospy.sleep(0.2)

def mavros_gps_cb(data):
    # refer to https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/GPSRAW.html
    global gps_fix, gps_satellites
    gps_satellites = data.satellites_visible
    fix_type = data.fix_type
    if fix_type == 0:
        gps_fix = "NO_GPS" # No GPS connected
    elif fix_type == 1:
        gps_fix = "NO_FIX" # No position information, GPS is connected
    elif fix_type == 2:
        gps_fix = "2D_FIX"  # 2D position
    elif fix_type == 3:
        gps_fix = "3D_FIX"  # 3D position. This one is the good one!
    elif fix_type == 4:
        gps_fix = "DGPS"  # DGPS/SBAS aided 3D position
    elif fix_type == 5:
        gps_fix = "RTK_FLOATR"  # TK float, 3D position
    elif fix_type == 6:
        gps_fix = "RTK_FIXEDR"  # TK Fixed, 3D position
    elif fix_type == 7:
        gps_fix = "STATIC"  # Static fixed, typically used for base stations
    elif fix_type == 8:
        gps_fix = "PPP"  # PPP, 3D position
    rospy.sleep(0.2)

def mavros_pose_cb(data):
    global ENUpos, yaw
    ENUpos[0] = data.pose.position.x
    ENUpos[1] = data.pose.position.y
    ENUpos[2] = data.pose.position.z
    qx = data.pose.orientation.x
    qy = data.pose.orientation.y
    qz = data.pose.orientation.z
    qw = data.pose.orientation.w
    euler = euler_from_quaternion([qx, qy, qz, qw])
    yaw = euler[2] * 180 / 3.1415926 # deg from East (ccw)
    rospy.sleep(0.2)

def mavros_battery_cb(data):
    global bat_volt, bat_cur, bat_perct
    bat_volt = data.voltage
    bat_cur = -data.current
    bat_perct = data.percentage
    rospy.sleep(0.2)

rospy.init_node("oled_node", anonymous=False)
rospy.Subscriber('mavros/state', State, mavros_state_cb, queue_size=1)
rospy.Subscriber('mavros/gpsstatus/gps1/raw', GPSRAW, mavros_gps_cb, queue_size=1)
rospy.Subscriber('mavros/local_position/pose', PoseStamped, mavros_pose_cb, queue_size=1)
rospy.Subscriber('mavros/battery', BatteryState, mavros_battery_cb, queue_size=1)


##################################### oled init ############################################
import time

import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess

# Raspberry Pi pin configuration:
RST = None     # on the PiOLED this pin isnt used
# Note the following are only used with SPI:
DC = 23
SPI_PORT = 0
SPI_DEVICE = 0

# Beaglebone Black pin configuration:
# RST = 'P9_12'
# Note the following are only used with SPI:
# DC = 'P9_15'
# SPI_PORT = 1
# SPI_DEVICE = 0

# 128x32 display with hardware I2C:
# disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)

# 128x64 display with hardware I2C:
disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST)

# Note you can change the I2C address by passing an i2c_address parameter like:
# disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST, i2c_address=0x3C)

# Alternatively you can specify an explicit I2C bus number, for example
# with the 128x32 display you would use:
# disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST, i2c_bus=2)

# 128x32 display with hardware SPI:
# disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST, dc=DC, spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE, max_speed_hz=8000000))

# 128x64 display with hardware SPI:
# disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST, dc=DC, spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE, max_speed_hz=8000000))

# Alternatively you can specify a software SPI implementation by providing
# digital GPIO pin numbers for all the required display pins.  For example
# on a Raspberry Pi with the 128x32 display you might use:
# disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST, dc=DC, sclk=18, din=25, cs=22)

# Initialize library.
disp.begin()

# Clear display.
disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
oled_width = disp.width
oled_height = disp.height
image = Image.new('1', (oled_width, oled_height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0,0,oled_width,oled_height), outline=0, fill=0)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = -2
top = padding
bottom = oled_height-padding
# Move left to right keeping track of the current x position for drawing shapes.
left = 0

# Load default font.
font = ImageFont.load_default()

# Alternatively load a TTF font.  Make sure the .ttf font file is in the same directory as the python script!
# Some other nice fonts to try: http://www.dafont.com/bitmap.php
# font = ImageFont.truetype('Minecraftia.ttf', 8)

page = 1 
max_page = 3

##################################### key init ############################################
import RPi.GPIO as GPIO
import time


key_id = 21 # GPIO number

GPIO.setmode(GPIO.BCM)
# GPIO.setup(16,GPIO.OUT)
GPIO.setup(key_id,GPIO.IN,pull_up_down=GPIO.PUD_UP) # 上拉，默认为高电平，按下为低
# GPIO.setup(key,GPIO.IN)


keystate = 1 # 上拉，默认为高电平，按下为低
keydown_time = 0.0
press_state = ""
press_count = 0

def confirm():
    ### 确认函数？
    # TODO
    global page
    page = 1 # 回到主页
    print("page: {}/{}".format(page, max_page))
    pass

def short_press():
    ### 短按按钮的函数（翻页）
    global page, max_page
    global press_state, press_count
    press_state = "short pressed"
    press_count = press_count + 1
    print("short pressed")    
    page = page + 1
    if page > max_page:
        page = 1
    print("page: {}/{}".format(page, max_page))

def long_press():
    # 长按按钮的函数（确认）
    global press_state, press_count
    press_state = "long pressed"
    press_count = press_count + 1
    print("long pressed")   
    confirm()

def key_cb(chn):
    #### 按下和回弹按键的回调函数callback ###
    global key_id, keystate, keydown_time
    time.sleep(0.05)
    keystate = GPIO.input(key_id) # 读按键状态
    # print(keystate)
    if keystate == False: # 按下按键
        keydown_time = time.time()
    else: # 回弹按键
        timenow = time.time()
        if timenow - keydown_time > 0.7: # 长按
            keydown_time = timenow # 防止按键抖动连续产生两次long press
            long_press()
        else: # 短按
            short_press()
    # GPIO.output(16,state)
    time.sleep(0.1)    # 这个语句的作用是防止按键电平不稳定，造成多次触发
                       # 可以在add_event_detect()中添加bouncetime=200来代替

GPIO.add_event_detect(key_id,GPIO.BOTH,callback=key_cb)


################################### oled page #######################################

def oled_page_px4():
    global draw, disp, image, font
    global oled_height, oled_width
    global padding, top, bottom, left
    global page, max_page
    global keystate, press_count, press_state

    # Draw a black filled box to clear the image.
    draw.rectangle((0,0,oled_width,oled_height), outline=0, fill=0)

    h = 10
    draw.text((left+oled_width/2-28, top),  "MiniDrone",  font=font, fill=255)
    draw.text((oled_width-20, top),  "{}/{}".format(page, max_page),  font=font, fill=255)

    draw.text((left, top+h),  "PX4: " + px4_mode,  font=font, fill=255)
    draw.text((left+oled_width/2, top+h),  ' | ' + px4_state,  font=font, fill=255)

    draw.text((left, top+2*h),  "GPS: " + gps_fix,  font=font, fill=255)
    draw.text((left+oled_width/2, top+2*h),  ' | ' + str(gps_satellites) + ' sat',  font=font, fill=255)
    
    draw.text((left, top+3*h),  "X: {:.2f} m".format(ENUpos[0]),  font=font, fill=255)
    draw.text((left+oled_width/2+2, top+3*h),  "Y: {:.2f} m".format(ENUpos[1]),  font=font, fill=255)
    
    draw.text((left, top+4*h),  "Z: {:.2f} m".format(ENUpos[2]),  font=font, fill=255)
    draw.text((left+oled_width/2+2, top+4*h),  "Yaw: {:.1f}°".format(yaw),  font=font, fill=255)
    
    draw.text((left, top+5*h),  "Bat: {:.2f}V {:.0f}% {:.1f}A".format(bat_volt, bat_perct*100, bat_cur),  font=font, fill=255)

    # Display image.
    disp.image(image)
    disp.display()


def oled_page_monitor():
    global draw, disp, image, font
    global oled_height, oled_width
    global padding, top, bottom, left
    global page, max_page
    global keystate, press_count, press_state

    # Draw a black filled box to clear the image.
    draw.rectangle((0,0,oled_width,oled_height), outline=0, fill=0)

    # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
    cmd = "hostname -I | cut -d\' \' -f1"
    # IP = subprocess.check_output(cmd, shell = True )
    IP = subprocess.getoutput(cmd)
    
    cmd = "top -bn1 | grep load | awk '{printf \"CPU: %.1f%% \", $(NF-2)*100/4}'" # "load in last one minutes"/4 (4核cpu)
    # CPU = subprocess.check_output(cmd, shell = True )
    CPU = subprocess.getoutput(cmd)

    cmd = "sensors |grep temp1 |awk '{printf $(NF)}'"
    CPU_temp = subprocess.getoutput(cmd)
    
    cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%sMB %.1f%%\", $3,$2,$3*100/$2 }'"
    # MemUsage = subprocess.check_output(cmd, shell = True )
    MemUsage = subprocess.getoutput(cmd)

    cmd = "df -h | awk '$NF==\"/\"{printf \"Disk: %d/%dGB %s\", $3,$2,$5}'"
    # Disk = subprocess.check_output(cmd, shell = True )
    Disk = subprocess.getoutput(cmd)

    # Write two lines of text.
    h = 10
    draw.text((left+oled_width/2-26, top),  "Monitor",  font=font, fill=255)
    draw.text((oled_width-20, top),  "{}/{}".format(page, max_page),  font=font, fill=255)
    draw.text((left, top+h),       "IP: " + str(IP),  font=font, fill=255)
    draw.text((left, top+2*h),     str(CPU) + str(CPU_temp), font=font, fill=255)
    draw.text((left, top+3*h),    str(MemUsage),  font=font, fill=255)
    draw.text((left, top+4*h),    str(Disk),  font=font, fill=255)
    draw.text((left, top+5*h),       "key: "  + str(press_count) + ' ' + str(press_state),  font=font, fill=255)

    # Display image.
    disp.image(image)
    disp.display()

def oled_page_network():
    global draw, disp, image, font
    global oled_height, oled_width
    global padding, top, bottom, left
    global page, max_page
    global keystate, press_count, press_state

    # Draw a black filled box to clear the image.
    draw.rectangle((0,0,oled_width,oled_height), outline=0, fill=0)

    # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
    cmd = "hostname -I | cut -d\' \' -f1"
    # IP = subprocess.check_output(cmd, shell = True )
    IP = subprocess.getoutput(cmd)
    
    cmd = "top -bn1 | grep load | awk '{printf \"CPU: %.1f%% \", $(NF-2)*100/4}'" # "load in last one minutes"/4 (4核cpu)
    # CPU = subprocess.check_output(cmd, shell = True )
    CPU = subprocess.getoutput(cmd)

    cmd = "sensors |grep temp1 |awk '{printf $(NF)}'"
    CPU_temp = subprocess.getoutput(cmd)
    
    cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%sMB %.1f%%\", $3,$2,$3*100/$2 }'"
    # MemUsage = subprocess.check_output(cmd, shell = True )
    MemUsage = subprocess.getoutput(cmd)

    cmd = "df -h | awk '$NF==\"/\"{printf \"Disk: %d/%dGB %s\", $3,$2,$5}'"
    # Disk = subprocess.check_output(cmd, shell = True )
    Disk = subprocess.getoutput(cmd)

    cmd = "cat /proc/net/wireless |grep wlan0 |awk '{print $3}'"
    Link_quality = float(subprocess.getoutput(cmd)) / 70  # 70. /70

    cmd = "cat /proc/net/wireless |grep wlan0 |awk '{print $4}'"
    Signal_level = float(subprocess.getoutput(cmd)) # -32 dBm

    cmd = "iwconfig wlan0 |grep ESSID |awk '{printf $(NF)}'"
    wlan0_ssid = subprocess.getoutput(cmd) # ESSID:"Xiaomi-C105"

    # Write two lines of text.
    h = 11
    draw.text((left+oled_width/2-26, top),  "Network",  font=font, fill=255)
    draw.text((oled_width-20, top),  "{}/{}".format(page, max_page),  font=font, fill=255)
    draw.text((left, top+h),    "IP: " + str(IP),  font=font, fill=255)
    draw.text((left, top+2*h),  wlan0_ssid,  font=font, fill=255)
    draw.text((left, top+3*h),  "Link Quality: {:.0f}%".format(Link_quality*100),  font=font, fill=255)
    draw.text((left, top+4*h),  "Signal Level: {:.0f}dBm".format(Signal_level),  font=font, fill=255)

    # Display image.
    disp.image(image)
    disp.display()


################################### main loop #######################################

while not rospy.is_shutdown():
    try:
        ######################## oled ######################
        if page == 1:
            oled_page_px4()
        elif page == 2:
            oled_page_monitor()
        elif page == 3:
            oled_page_network()
        else:
            pass
        time.sleep(1.0) # sleep 
    except KeyboardInterrupt:
        # GPIO.output(16,GPIO.LOW)
        GPIO.cleanup()