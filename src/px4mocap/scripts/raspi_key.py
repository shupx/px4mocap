#!/usr/bin/python3
# coding=utf-8

# Peixuan Shu
# 2023.5.29
# 参考 https://blog.csdn.net/u014663232/article/details/105613749

import RPi.GPIO as GPIO
import time


key_id = 21 # GPIO number

GPIO.setmode(GPIO.BCM)
# GPIO.setup(16,GPIO.OUT)
GPIO.setup(key_id,GPIO.IN,pull_up_down=GPIO.PUD_UP) # 上拉，默认为高电平，按下为低
# GPIO.setup(key,GPIO.IN)


keystate = True # 上拉，默认为高电平，按下为低
keydown_time = 0.0


def key_cb(chn):
    # 按下和回弹按键的回调函数callback
    global key_id, keystate, keydown_time
    time.sleep(0.05)
    keystate = GPIO.input(key_id) # 读按键状态
    print(keystate)
    if keystate == False: # 按下按键
        keydown_time = time.time()
    else: # 回弹按键
        timenow = time.time()
        if timenow - keydown_time > 0.7:
            keydown_time = timenow # 防止按键抖动连续产生两次long press
            print("long pressed")
        else:
            print("short pressed")
    # GPIO.output(16,state)
    time.sleep(0.2)    # 这个语句的作用是防止按键电平不稳定，造成多次触发
                       # 可以在add_event_detect()中添加bouncetime=200来代替


try:
    GPIO.add_event_detect(key_id,GPIO.BOTH,callback=key_cb)
    # GPIO.RISING    上升沿检测
    # GPIO.FALLING   下降沿检测
    # GPIO.BOTH      两者都可以，也就是说检测到边沿变化
    # bouncetime=50 等待时间ms（可选）
    # 如果在后面我们不需要事件响应了，可以用GPIO.remove_event_detect(引脚)在事件列表中删除
    while True:
        # print("waiting for pressing key {}".format(key_id))
        time.sleep(10)
except KeyboardInterrupt:
    # GPIO.output(16,GPIO.LOW)
    GPIO.cleanup()
