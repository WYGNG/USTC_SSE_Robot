# -*- coding:UTF-8 -*-

import RPi.GPIO as GPIO
import time

#RGB三色灯引脚定义
LED_R = 22
LED_G = 27
LED_B = 24

#设置RGB三色灯为BCM编码方式
GPIO.setmode(GPIO.BCM)

#RGB三色灯设置为输出模式
GPIO.setup(LED_R, GPIO.OUT)
GPIO.setup(LED_G, GPIO.OUT)
GPIO.setup(LED_B, GPIO.OUT)

#循环显示7种不同的颜色
try:
    while True:
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.LOW)
        time.sleep(1)
        GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.HIGH)
        GPIO.output(LED_B, GPIO.LOW)
        time.sleep(1)
        GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.HIGH)
        GPIO.output(LED_B, GPIO.LOW)
        time.sleep(1)
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.HIGH)
        GPIO.output(LED_B, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.LOW)
        time.sleep(1)
except:
    print "except"
#使用try except语句，当CTRL+C结束进程时会触发异常后
#会执行gpio.cleanup()语句清除GPIO管脚的状态
GPIO.cleanup()
