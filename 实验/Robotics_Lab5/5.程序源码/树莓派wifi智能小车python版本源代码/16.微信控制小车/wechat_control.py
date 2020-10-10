#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
import string
import serial

#按键值定义
run_car  = '1'  #按键前
back_car = '2'  #按键后
left_car = '3'  #按键左
right_car = '4' #按键右
stop_car = '0'  #按键停

#状态值定义
enSTOP = 0
enRUN =1
enBACK = 2
enLEFT = 3
enRIGHT = 4
enTLEFT =5
enTRIGHT = 6


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

#蜂鸣器引脚定义
buzzer = 8

#灭火电机引脚设置
OutfirePin = 2

#循迹红外引脚定义
#TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
#      3                 5                  4                   18
TrackSensorLeftPin1  =  3   #定义左边第一个循迹红外传感器引脚为3口
TrackSensorLeftPin2  =  5   #定义左边第二个循迹红外传感器引脚为5口
TrackSensorRightPin1 =  4   #定义右边第一个循迹红外传感器引脚为4口
TrackSensorRightPin2 =  18  #定义右边第二个循迹红外传感器引脚为18口

#光敏电阻引脚定义
LdrSensorLeft = 7
LdrSensorRight = 6

#串口计时全局变量定义
global timecount 
global count 
global t1
global t2
red = 0
green = 0
blue = 0
NewLineReceived = 0
InputString = ''
InputStringcache = ''
g_CarState = 0 
CarSpeedControl = 50 
g_num = 0
g_packnum = 0 
ReturnTemp = ''
infrared_track_value = ''
infrared_avoid_value = ''
LDR_value = ''
StartBit = 0
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
    global pwm_rled
    global pwm_gled
    global pwm_bled
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(key,GPIO.IN)
    GPIO.setup(buzzer,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(OutfirePin,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(EchoPin,GPIO.IN)
    GPIO.setup(TrigPin,GPIO.OUT)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)
    GPIO.setup(AvoidSensorLeft,GPIO.IN)
    GPIO.setup(AvoidSensorRight,GPIO.IN)
    GPIO.setup(LdrSensorLeft,GPIO.IN)
    GPIO.setup(LdrSensorRight,GPIO.IN)
    GPIO.setup(TrackSensorLeftPin1,GPIO.IN)
    GPIO.setup(TrackSensorLeftPin2,GPIO.IN)
    GPIO.setup(TrackSensorRightPin1,GPIO.IN)
    GPIO.setup(TrackSensorRightPin2,GPIO.IN)
    #设置pwm引脚和频率为2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    #设置舵机的频率和起始占空比
    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(0)
    pwm_rled = GPIO.PWM(LED_R, 1000)
    pwm_gled = GPIO.PWM(LED_G, 1000)
    pwm_bled = GPIO.PWM(LED_B, 1000)
    pwm_rled.start(0)
    pwm_gled.start(0)
    pwm_bled.start(0)
	

	
#小车前进	
def run():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

#小车后退
def back():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
	
#小车左转	
def left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

#小车右转
def right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
	
#小车原地左转
def spin_left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

#小车原地右转
def spin_right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

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
    #print "distance is %d " % (((t2 - t1)* 340 / 2) * 100)
    time.sleep(0.01)
    return ((t2 - t1)* 340 / 2) * 100
	
#舵机旋转到指定角度
def servo_appointed_detection(pos):
    for i in range(18):
        pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)

#巡线测试
def tracking_test():
    global infrared_track_value
    #检测到黑线时循迹模块相应的指示灯亮，端口电平为LOW
    #未检测到黑线时循迹模块相应的指示灯灭，端口电平为HIGH
    TrackSensorLeftValue1  = GPIO.input(TrackSensorLeftPin1)
    TrackSensorLeftValue2  = GPIO.input(TrackSensorLeftPin2)
    TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
    TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)
    infrared_track_value_list = ['0','0','0','0']
    infrared_track_value_list[0] = str(1^ TrackSensorLeftValue1)
    infrared_track_value_list[1] =str(1^ TrackSensorLeftValue2)
    infrared_track_value_list[2] = str(1^ TrackSensorRightValue1)
    infrared_track_value_list[3] = str(1^ TrackSensorRightValue2)
    infrared_track_value = ''.join(infrared_track_value_list)
    

#避障红外引脚测试
def infrared_avoid_test():
    global infrared_avoid_value
    #遇到障碍物,红外避障模块的指示灯亮,端口电平为LOW
    #未遇到障碍物,红外避障模块的指示灯灭,端口电平为HIGH
    LeftSensorValue  = GPIO.input(AvoidSensorLeft)
    RightSensorValue = GPIO.input(AvoidSensorRight)
    infrared_avoid_value_list = ['0','0']
    infrared_avoid_value_list[0] = str(1 ^ LeftSensorValue)
    infrared_avoid_value_list[1] = str(1 ^ RightSensorValue)
    infrared_avoid_value = ''.join(infrared_avoid_value_list)
    	
#寻光引脚测试
def follow_light_test():
    global LDR_value
    #遇到光线,寻光模块的指示灯灭,端口电平为HIGH
    #未遇光线,寻光模块的指示灯亮,端口电平为LOW
    LdrSersorLeftValue  = GPIO.input(LdrSensorLeft)
    LdrSersorRightValue = GPIO.input(LdrSensorRight)  
    LDR_value_list = ['0','0']
    LDR_value_list[0] = str(LdrSersorLeftValue)
    LDR_value_list[1] = str(LdrSersorRightValue)	
    LDR_value = ''.join(LDR_value_list)
	
#小车鸣笛
def whistle():
    GPIO.output(buzzer, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(buzzer, GPIO.HIGH)
    time.sleep(0.001)	
	
#七彩灯亮指定颜色
def color_led_pwm(iRed,iGreen, iBlue):
    print iRed 
    print iGreen
    print iBlue
    v_red = (100*iRed)/255
    v_green = (100*iGreen)/255
    v_blue = (100*iBlue)/255
    print v_red
    print v_green
    print v_blue
    pwm_rled.ChangeDutyCycle(v_red)
    pwm_gled.ChangeDutyCycle(v_green)
    pwm_bled.ChangeDutyCycle(v_blue)
    time.sleep(0.02)
	
#串口数据解析并指定相应的动作
def serial_data_parse():
    global NewLineReceived
    global CarSpeedControl
    global g_CarState
    global red
    global green
    global blue
    global ReturnTemp
    ReturnTemp = ''	  		  
    #解析上位机发来的通用协议指令,并执行相应的动作
    #如:$1,0,0,0,0,0,0,0,0,0#    小车前进
    
    if InputString[3] == '1':
        g_CarState = enTLEFT
        print "g_CarState: %d" % g_CarState
    elif InputString[3] == '2':
        g_CarState = enTRIGHT
    else :
        g_CarState = enSTOP

    if InputString[5] == '1':
        whistle()
    if InputString[7] == '1':
        CarSpeedControl += 20
    if CarSpeedControl > 100:
        CarSpeedControl = 100
    if InputString[9] == '1':
        CarSpeedControl -= 20
    if CarSpeedControl < 20:
        CarSpeedControl = 20
    if InputString[11] == '1':
        servo_appointed_detection(180)
    if InputString[13] == '2':
        servo_appointed_detection(0)
			
    if InputString[17] == '1':
        color_led_pwm(255, 255, 255)
    if InputString[17] == '2':
        color_led_pwm(255, 0, 0)
    if InputString[17] == '3':
        color_led_pwm(0, 255, 0)
    if InputString[17] == '4':
        color_led_pwm(0, 0, 255)
    if InputString[17] == '5':
        color_led_pwm(0, 255, 255)
    if InputString[17] == '6':
        color_led_pwm(255, 0, 255)
    if InputString[17] == '7':
        color_led_pwm(255, 255, 0)
    if InputString[17] == '8':
        color_led_pwm(0,0,0)		
          
    if InputString[19] == '1':
        GPIO.output(OutfirePin,not GPIO.input(OutfirePin) )
        time.sleep(1)
  
    if InputString[21] == '1':
        servo_appointed_detection(90)
        print "carstate:%d" % g_CarState
    if g_CarState != enTLEFT and g_CarState != enTRIGHT:
           
        print run_car
        print InputString[1]
        if InputString[1] == run_car:
            g_CarState = enRUN
            print "run car"
        elif InputString[1] == back_car:
            g_CarState = enBACK	
        elif InputString[1] == left_car:
            g_CarState = enLEFT
        elif InputString[1] == right_car:
            g_CarState = enRIGHT
        elif InputString[1] == stop_car:
            g_CarState = enSTOP
        else:
            g_CarState = enSTOP
    #采集的传感器数据串口回发给上位机显示				
    distance = Distance_test()
    ReturnTemp += "$0,0,0,0,0,0,0,0,0,0,0,"
    ReturnTemp += str(int(distance))
    ReturnTemp += "cm,8.4v#"		
    NewLineReceived = 0
    ser.write(ReturnTemp)
    InputString.zfill(len(InputString))	
		  	
def serialEvent():
    global InputString
    global InputStringcache
    global StartBit
    global NewLineReceived
    InputString = ''
    while True:
        size = ser.inWaiting()
        if size == 0:
           
            break
        else:
            while size != 0:
                serialdatabit = ser.read(1)
                size -= 1
                if serialdatabit == '$':
                    StartBit = 1
                if StartBit == 1:
                    InputStringcache += serialdatabit
                if StartBit == 1 and serialdatabit == '#':
                    NewLineReceived = 1
                    InputString = InputStringcache
                    InputStringcache = ''
                    StartBit = 0
                    size = 0
                    print InputString	
          
try:
    ser = serial.Serial("/dev/ttyAMA0", 9600, timeout = 0.001)
    print "serial.isOpen() = ",ser.isOpen()
    ser.write("serial is on!")  
    init()
    while True:
        serialEvent()
     #   time.sleep(0.4)
	if NewLineReceived == 1:
            print "serialdata:%s" % InputString
	    serial_data_parse()
	    NewLineReceived = 0

        #print "nice to meet you"	
	if g_CarState == enSTOP:
	    brake()          
	elif g_CarState == enRUN:
	    run()
	elif g_CarState == enLEFT:
	    left()
	elif g_CarState == enRIGHT:
	    right()
	elif g_CarState == enBACK:
	    back()
	elif g_CarState == enTLEFT:
	    spin_left()
	elif g_CarState == enTRIGHT:
	    spin_right()
	else:
	    brake()
    
    
    	
except KeyboardInterrupt:
    pass
ser.close()
pwm_ENA.stop()
pwm_ENB.stop()
pwm_rled.stop()
pwm_gled.stop()
pwm_bled.stop()
pwm_servo.stop()
GPIO.cleanup()
	
	
	
	
	
	
