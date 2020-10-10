#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

#小车电机引脚定义
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

#小车按键定义
key = 8

#红外避障引脚定义
AvoidSensorLeft = 12
AvoidSensorRight = 17

#设置GPIO口为BCM编码方式
GPIO.setmode(GPIO.BCM)

#忽略警告信息
GPIO.setwarnings(False)

#电机引脚初始化为输出模式
#按键引脚初始化为输入模式
#红外避障引脚初始化为输入模式
def init():
    global pwm_ENA
    global pwm_ENB
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(key,GPIO.IN)
    GPIO.setup(AvoidSensorLeft,GPIO.IN)
    GPIO.setup(AvoidSensorRight,GPIO.IN)
    #设置pwm引脚和频率为2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
	
#小车前进	
def run():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(100)
    pwm_ENB.ChangeDutyCycle(100)

#小车后退
def back():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(80)
    pwm_ENB.ChangeDutyCycle(80)
	
#小车左转	
def left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(0)
    pwm_ENB.ChangeDutyCycle(80)

#小车右转
def right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(80)
    pwm_ENB.ChangeDutyCycle(0)
	
#小车原地左转
def spin_left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(80)
    pwm_ENB.ChangeDutyCycle(80)

#小车原地右转
def spin_right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(80)
    pwm_ENB.ChangeDutyCycle(80)

#小车停止	
def brake():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

#按键检测
def key_scan():
    while GPIO.input(key):
         pass
    while not GPIO.input(key):
	 time.sleep(0.01)
         if not GPIO.input(key):
             time.sleep(0.01)
	     while not GPIO.input(key):
	         pass

#延时2s	
time.sleep(2)

#try/except语句用来检测try语句块中的错误，
#从而让except语句捕获异常信息并处理。
try:
    init()
    key_scan()
    while True:
        #遇到障碍物,红外避障模块的指示灯亮,端口电平为LOW
        #未遇到障碍物,红外避障模块的指示灯灭,端口电平为HIGH
        LeftSensorValue  = GPIO.input(AvoidSensorLeft);
        RightSensorValue = GPIO.input(AvoidSensorRight);

        if LeftSensorValue == True and RightSensorValue == True :
            run()         #当两侧均未检测到障碍物时调用前进函数
        elif LeftSensorValue == True and RightSensorValue == False :
            spin_left()   #右边探测到有障碍物，有信号返回，原地向左转
	    time.sleep(0.002)
        elif RightSensorValue == True and LeftSensorValue == False:
            spin_right()  #左边探测到有障碍物，有信号返回，原地向右转
            time.sleep(0.002)			
        elif RightSensorValue == False and LeftSensorValue == False :
            spin_right()  #当两侧均检测到障碍物时调用固定方向的避障(原地右转)
	    time.sleep(0.002)
       
except KeyboardInterrupt:
    pass
pwm_ENA.stop()
pwm_ENB.stop()
GPIO.cleanup()

