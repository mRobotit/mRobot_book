#!/usr/bin/env python
#-*-coding: utf-8 -*-
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float32
from time import sleep

GPIO.setmode(GPIO.BCM)            # 使用BCM编号方式
GPIO.setup(4,GPIO.OUT)            # 将GPIO4设置为输出模式
GPIO.setup(18,GPIO.OUT)            # 将GPIO18设置为输出模式
GPIO.setup(24,GPIO.OUT)            # 将GPIO24设置为输出模式

def callback(data):
    if data>10:
        GPIO.output(24,GPIO.HIGH)   # 将GPIO24设置为高电平，点亮LED
        GPIO.output(18,GPIO.HIGH)   # 将GPIO18设置为高电平，点亮LED
        GPIO.output(4,GPIO.HIGH)   # 将GPIO4设置为高电平，点亮LED
    elif data>5:
        GPIO.output(24,GPIO.LOW)  # 将GPIO24设置为低电平，熄灭LED
    elif data>2:
        GPIO.output(18,GPIO.LOW)  # 将GPIO18设置为低电平，熄灭LED
    else:
        GPIO.output(4,GPIO.LOW)  # 将GPIO4设置为低电平，熄灭LED
    
def led_light():
    rospy.init_node('led_light', anonymous=Ture)
    rospy.Subscriber('/robot/PowerValtage', Float32, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    led_light()
    GPIO.cleanup()                     # 清理释放GPIO资源，将GPIO复位
