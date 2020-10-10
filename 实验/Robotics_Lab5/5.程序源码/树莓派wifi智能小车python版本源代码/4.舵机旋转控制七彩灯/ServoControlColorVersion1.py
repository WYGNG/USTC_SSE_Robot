#-*- coding:UTF-8 -*-
#本次舵机转动控制七彩灯控制舵机采用的是系统的pwm库
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
    global pwm_servo
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)
    #设置pwm引脚和频率为50hz
    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(0)

	
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
        pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)
	corlor_light(pos)
	time.sleep(0.009) 
    for pos in reversed(range(181)):
        pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)
	corlor_light(pos)
	time.sleep(0.009)

#延时2s		
time.sleep(2)

#try/except语句用来检测try语句块中的错误，
#从而让except语句捕获异常信息并处理。
try:
    init()
    #舵机初始化向前
    pwm_servo.ChangeDutyCycle(2.5 + 10 * 90/180)
    while True:
 	servo_control_color()
		
except KeyboardInterrupt:
    pass
pwm_servo.stop()
GPIO.cleanup()
