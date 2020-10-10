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

#超声波引脚定义
EchoPin = 0
TrigPin = 1

#RGB三色灯引脚定义
LED_R = 22
LED_G = 27
LED_B = 24

#舵机引脚定义
ServoPin = 23

#红外避障引脚定义
AvoidSensorLeft = 12
AvoidSensorRight = 17

#设置GPIO口为BCM编码方式
GPIO.setmode(GPIO.BCM)

#忽略警告信息
GPIO.setwarnings(False)

#电机引脚初始化为输出模式
#按键引脚初始化为输入模式
#超声波,RGB三色灯,舵机引脚初始化
#红外避障引脚初始化
def init():
    global pwm_ENA
    global pwm_ENB
    global pwm_servo
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(key,GPIO.IN)
    GPIO.setup(EchoPin,GPIO.IN)
    GPIO.setup(TrigPin,GPIO.OUT)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)
    GPIO.setup(AvoidSensorLeft,GPIO.IN)
    GPIO.setup(AvoidSensorRight,GPIO.IN)
    #设置pwm引脚和频率为2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    #设置舵机的频率和起始占空比
    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(0)
	
#小车前进	
def run(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#小车后退
def back(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)
	
#小车左转	
def left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#小车右转
def right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)
	
#小车原地左转
def spin_left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#小车原地右转
def spin_right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

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
				
#超声波函数
def Distance_test():
    GPIO.output(TrigPin,GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TrigPin,GPIO.LOW)
    while not GPIO.input(EchoPin):
        pass
    t1 = time.time()
    while GPIO.input(EchoPin):
        pass
    t2 = time.time()
    print "distance is %d " % (((t2 - t1)* 340 / 2) * 100)
    time.sleep(0.01)
    return ((t2 - t1)* 340 / 2) * 100
	
#舵机旋转到指定角度
def servo_appointed_detection(pos):
    for i in range(18):
        pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)	
		
#舵机旋转超声波测距避障，led根据车的状态显示相应的颜色
def servo_color_carstate():
    #开红灯
    GPIO.output(LED_R, GPIO.HIGH)
    GPIO.output(LED_G, GPIO.LOW)
    GPIO.output(LED_B, GPIO.LOW)
    back(20, 20)
    time.sleep(0.08)
    brake()
	
    #舵机旋转到0度，即右侧，测距
    servo_appointed_detection(0)
    time.sleep(0.8)
    rightdistance = Distance_test()
  
    #舵机旋转到180度，即左侧，测距
    servo_appointed_detection(180)
    time.sleep(0.8)
    leftdistance = Distance_test()

    #舵机旋转到90度，即前方，测距
    servo_appointed_detection(90)
    time.sleep(0.8)
    frontdistance = Distance_test()
 
    if leftdistance < 30 and rightdistance < 30 and frontdistance < 30:
        #亮品红色，掉头
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
        spin_right(85, 85)
 	time.sleep(0.58)
    elif leftdistance >= rightdistance:
	#亮蓝色
	GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
	spin_left(85, 85)
	time.sleep(0.28)
    elif leftdistance <= rightdistance:
	#亮品红色，向右转
	GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
	spin_right(85, 85)
	time.sleep(0.28)
		
#延时2s	
time.sleep(2)

#try/except语句用来检测try语句块中的错误，
#从而让except语句捕获异常信息并处理。
try:
    init()
    key_scan()
    while True:
        distance = Distance_test()
	if distance > 50:
	    #遇到障碍物,红外避障模块的指示灯亮,端口电平为LOW
            #未遇到障碍物,红外避障模块的指示灯灭,端口电平为HIGH
            LeftSensorValue  = GPIO.input(AvoidSensorLeft)
            RightSensorValue = GPIO.input(AvoidSensorRight)

            if LeftSensorValue == True and RightSensorValue == True :
                run(100, 100)         #当两侧均未检测到障碍物时调用前进函数
            elif LeftSensorValue == True and RightSensorValue == False :
                spin_left(85, 85)     #右边探测到有障碍物，有信号返回，原地向左转
		time.sleep(0.002)
            elif RightSensorValue == True and LeftSensorValue == False:
                spin_right(85, 85)    #左边探测到有障碍物，有信号返回，原地向右转
                time.sleep(0.002)				
            elif RightSensorValue == False and LeftSensorValue == False :
                spin_right(85, 85)    #当两侧均检测到障碍物时调用固定方向的避障(原地右转)
		time.sleep(0.002)
		run(100, 100)
		GPIO.output(LED_R, GPIO.LOW)
	        GPIO.output(LED_G, GPIO.HIGH)
	        GPIO.output(LED_B, GPIO.LOW)
	elif 30 <= distance <= 50:
	    #遇到障碍物,红外避障模块的指示灯亮,端口电平为LOW
            #未遇到障碍物,红外避障模块的指示灯灭,端口电平为HIGH
            LeftSensorValue  = GPIO.input(AvoidSensorLeft)
            RightSensorValue = GPIO.input(AvoidSensorRight)

            if LeftSensorValue == True and RightSensorValue == True :
                run(100, 100)         #当两侧均未检测到障碍物时调用前进函数
            elif LeftSensorValue == True and RightSensorValue == False :
                spin_left(85, 85)     #右边探测到有障碍物，有信号返回，原地向左转
		time.sleep(0.002)
            elif RightSensorValue == True and LeftSensorValue == False:
                spin_right(85, 85)    #左边探测到有障碍物，有信号返回，原地向右转
                time.sleep(0.002)				
            elif RightSensorValue == False and LeftSensorValue == False :
                spin_right(85, 85)    #当两侧均检测到障碍物时调用固定方向的避障(原地右转)
		time.sleep(0.002)
		run(60, 60)
	elif distance < 30:
	    servo_color_carstate()
       
except KeyboardInterrupt:
    pass
pwm_ENA.stop()
pwm_ENB.stop()
GPIO.cleanup()
