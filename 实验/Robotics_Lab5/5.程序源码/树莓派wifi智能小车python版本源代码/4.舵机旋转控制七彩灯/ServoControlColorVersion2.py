#-*- coding:UTF-8 -*-
#本次舵机控制七彩灯采用的是自己定义的脉冲函数来
#产生pwm波形
import RPi.GPIO as GPIO
import time

#RGB三色灯引脚定义
LED_R = 22
LED_G = 27
LED_B = 24

#舵机引脚定义
ServoPin = 23

#设置GPIO口为BCM编码方式
GPIO.setmode(GPIO.BCM)

#忽略警告信息
GPIO.setwarnings(False)

#RGB三色灯初始化为输出模式
#舵机引脚设置为输出模式
def init():
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)

#定义一个脉冲函数，用来模拟方式产生pwm值
#时基脉冲为20ms，该脉冲高电平部分在0.5-
#2.5ms控制0-180度
def servo_pulse(myangle):
    pulsewidth = (myangle * 11) + 500
    GPIO.output(ServoPin, GPIO.HIGH)
    time.sleep(pulsewidth/1000000.0)
    GPIO.output(ServoPin, GPIO.LOW)
    time.sleep(20.0/1000-pulsewidth/1000000.0)

	
#根据转动的角度来点亮相应的颜色
def corlor_light(pos):
    if pos > 150:
        GPIO.output(LED_R, GPIO.HIGH)
	GPIO.output(LED_G, GPIO.LOW)
	GPIO.output(LED_B, GPIO.LOW)
    elif pos > 125:
	GPIO.output(LED_R, GPIO.LOW)
	GPIO.output(LED_G, GPIO.HIGH)
	GPIO.output(LED_B, GPIO.LOW)
    elif pos >100:
        GPIO.output(LED_R, GPIO.LOW)
	GPIO.output(LED_G, GPIO.LOW)
	GPIO.output(LED_B, GPIO.HIGH)
    elif pos > 75:
	GPIO.output(LED_R, GPIO.HIGH)
	GPIO.output(LED_G, GPIO.HIGH)
	GPIO.output(LED_B, GPIO.LOW)
    elif pos > 50:
	GPIO.output(LED_R, GPIO.LOW)
	GPIO.output(LED_G, GPIO.HIGH)
	GPIO.output(LED_B, GPIO.HIGH)
    elif pos > 25:
        GPIO.output(LED_R, GPIO.HIGH)
	GPIO.output(LED_G, GPIO.LOW)
	GPIO.output(LED_B, GPIO.HIGH)
    elif pos > 0:
        GPIO.output(LED_R, GPIO.HIGH)
	GPIO.output(LED_G, GPIO.HIGH)
	GPIO.output(LED_B, GPIO.HIGH)
    else :
        GPIO.output(LED_R, GPIO.LOW)
	GPIO.output(LED_G, GPIO.LOW)
	GPIO.output(LED_B, GPIO.LOW)
		
#舵机来回转动
def servo_control_color():
    for pos in range(181):
        servo_pulse(pos)
	corlor_light(pos)
	time.sleep(0.009) 
    for pos in reversed(range(181)):
        servo_pulse(pos)
	corlor_light(pos)
	time.sleep(0.009)

#延时2s		
time.sleep(2)

#try/except语句用来检测try语句块中的错误，
#从而让except语句捕获异常信息并处理。
try:
    init()
    while True:
 	servo_control_color()
		
except KeyboardInterrupt:
    pass
GPIO.cleanup()
