#-*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import socket
import time
import string
import threading

#按键值定义
run_car  = '1'  #按键前
back_car = '2'  #按键后
left_car = '3'  #按键左
right_car = '4' #按键右
stop_car = '0'  #按键停

#舵机按键值定义
front_left_servo = '1'  #前舵机向左
front_right_servo = '2' #前舵机向右
up_servo = '3'          #摄像头舵机向上
down_servo = '4'        #摄像头舵机向下
left_servo = '6'        #摄像头舵机向左
right_servo = '7'       #摄像头舵机向右
updowninit_servo = '5'  #摄像头舵机上下复位
stop_servo = '8'        #舵机停止

#小车状态值定义
enSTOP = 0
enRUN =1
enBACK = 2
enLEFT = 3
enRIGHT = 4
enTLEFT =5
enTRIGHT = 6

#小车舵机定义
enFRONTSERVOLEFT = 1
enFRONTSERVORIGHT = 2
enSERVOUP = 3
enSERVODOWN = 4
enSERVOUPDOWNINIT = 5
enSERVOLEFT = 6
enSERVORIGHT = 7
enSERVOSTOP = 8



#初始化上下左右角度为90度
ServoLeftRightPos = 90
ServoUpDownPos = 90
g_frontServoPos = 90
g_nowfrontPos = 0


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
FrontServoPin = 23
ServoUpDownPin = 9
ServoLeftRightPin = 11

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

#变量的定义
#七彩灯RGB三色变量定义
red = 0
green = 0
blue = 0
#TCP通信数据包标志位以及接受和发送数据变量
NewLineReceived = 0
InputString = ''
recvbuf = ''
ReturnTemp = ''
#小车和舵机状态变量
g_CarState = 0
g_ServoState = 0
#小车速度变量
CarSpeedControl = 80 
#寻迹，避障，寻光变量
infrared_track_value = ''
infrared_avoid_value = ''
LDR_value = ''
g_lednum = 0

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
    global pwm_FrontServo
    global pwm_UpDownServo
    global pwm_LeftRightServo
    global pwm_rled
    global pwm_gled
    global pwm_bled
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(buzzer,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(OutfirePin,GPIO.OUT)
    GPIO.setup(EchoPin,GPIO.IN)
    GPIO.setup(TrigPin,GPIO.OUT)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(FrontServoPin, GPIO.OUT)
    GPIO.setup(ServoUpDownPin, GPIO.OUT)
    GPIO.setup(ServoLeftRightPin, GPIO.OUT)
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
    pwm_FrontServo = GPIO.PWM(FrontServoPin, 50)
    pwm_UpDownServo = GPIO.PWM(ServoUpDownPin, 50)
    pwm_LeftRightServo = GPIO.PWM(ServoLeftRightPin, 50)
    pwm_FrontServo.start(0)
    pwm_UpDownServo.start(0)
    pwm_LeftRightServo.start(0)
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
				
#超声波测距函数
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
	
#前舵机旋转到指定角度
def frontservo_appointed_detection(pos): 
    for i in range(18):   
    	pwm_FrontServo.ChangeDutyCycle(2.5 + 10 * pos/180)
    	time.sleep(0.02)							#等待20ms周期结束
    	#pwm_FrontServo.ChangeDutyCycle(0)	#归零信号

#摄像头舵机左右旋转到指定角度
def leftrightservo_appointed_detection(pos): 
    for i in range(1):   
    	pwm_LeftRightServo.ChangeDutyCycle(2.5 + 10 * pos/180)
    	time.sleep(0.02)							#等待20ms周期结束
    	#pwm_LeftRightServo.ChangeDutyCycle(0)	#归零信号

#摄像头舵机上下旋转到指定角度
def updownservo_appointed_detection(pos):  
    for i in range(1):  
    	pwm_UpDownServo.ChangeDutyCycle(2.5 + 10 * pos/180)
    	time.sleep(0.02)							#等待20ms周期结束
    	#pwm_UpDownServo.ChangeDutyCycle(0)	#归零信号

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
    infrared_track_value_list[0] = str(1 ^TrackSensorLeftValue1)
    infrared_track_value_list[1] = str(1 ^TrackSensorLeftValue2)
    infrared_track_value_list[2] = str(1 ^TrackSensorRightValue1)
    infrared_track_value_list[3] = str(1 ^TrackSensorRightValue2)
    infrared_track_value = ''.join(infrared_track_value_list)
    

#避障红外引脚测试
def infrared_avoid_test():
    global infrared_avoid_value
    #遇到障碍物,红外避障模块的指示灯亮,端口电平为LOW
    #未遇到障碍物,红外避障模块的指示灯灭,端口电平为HIGH
    LeftSensorValue  = GPIO.input(AvoidSensorLeft)
    RightSensorValue = GPIO.input(AvoidSensorRight)
    infrared_avoid_value_list = ['0','0']
    infrared_avoid_value_list[0] = str(1 ^LeftSensorValue)
    infrared_avoid_value_list[1] = str(1 ^RightSensorValue)
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
    v_red = (100*iRed)/255
    v_green = (100*iGreen)/255
    v_blue = (100*iBlue)/255
    pwm_rled.ChangeDutyCycle(v_red)
    pwm_gled.ChangeDutyCycle(v_green)
    pwm_bled.ChangeDutyCycle(v_blue)
    time.sleep(0.02)

#摄像头舵机向上运动
def servo_up():
    global ServoUpDownPos
    pos = ServoUpDownPos
    updownservo_appointed_detection(pos)
    #time.sleep(0.05)
    pos +=0.7 
    ServoUpDownPos = pos
    if ServoUpDownPos >= 180:
        ServoUpDownPos = 180

#摄像头舵机向下运动		
def servo_down():
    global ServoUpDownPos
    pos = ServoUpDownPos
    updownservo_appointed_detection(pos)
    #time.sleep(0.05)
    pos -= 0.7
    ServoUpDownPos = pos
    if ServoUpDownPos <= 45:
        ServoUpDownPos = 45
    

#摄像头舵机向左运动
def servo_left():
    global ServoLeftRightPos
    pos = ServoLeftRightPos
    leftrightservo_appointed_detection(pos)
    #time.sleep(0.10)
    pos += 0.7
    ServoLeftRightPos = pos
    if ServoLeftRightPos >= 180:
        ServoLeftRightPos = 180

#摄像头舵机向右运动
def servo_right():
    global ServoLeftRightPos
    pos = ServoLeftRightPos
    leftrightservo_appointed_detection(pos)
    #time.sleep(0.10)
    pos -= 0.7 
    ServoLeftRightPos = pos
    if ServoLeftRightPos <= 0:
        ServoLeftRightPos =  0

#前舵机向左
def front_servo_left():
    frontservo_appointed_detection(180)

#前舵机向右
def front_servo_right():
    frontservo_appointed_detection(0)

#所有舵机归位
def servo_init():
    servoflag = 0
    servoinitpos = 90
    if servoflag != servoinitpos:        
        frontservo_appointed_detection(servoinitpos)
        updownservo_appointed_detection(servoinitpos)
        leftrightservo_appointed_detection(servoinitpos)
        time.sleep(0.5)
        pwm_FrontServo.ChangeDutyCycle(0)	#归零信号
        pwm_LeftRightServo.ChangeDutyCycle(0)	#归零信号
        pwm_UpDownServo.ChangeDutyCycle(0)	#归零信号

#摄像头舵机上下归位	
def servo_updown_init():
    updownservo_appointed_detection(90)
	
#舵机停止
def servo_stop():
    pwm_LeftRightServo.ChangeDutyCycle(0)	#归零信号
    pwm_UpDownServo.ChangeDutyCycle(0)	#归零信号 
    pwm_FrontServo.ChangeDutyCycle(0)	#归零信号
		
#tcp数据解析并指定相应的动作
def tcp_data_parse():
    global NewLineReceived
    global CarSpeedControl
    global g_CarState
    global g_ServoState
    global g_frontServoPos
    global red
    global green
    global blue
    global g_lednum
    #解析上位机发来的舵机云台的控制指令并执行舵机旋转
    #如:$4WD,PTZ180# 舵机转动到180度	
    if (InputString.find("$4WD,PTZ", 0, len(InputString)) != -1):
        i = InputString.find("PTZ",  0, len(InputString)) 
        ii = InputString.find("#",  0, len(InputString))
  	if ii > i:
            string = InputString[i+3:ii]
	    m_kp = int(string)
	    g_frontServoPos = 180 - m_kp;
	    NewLineReceived = 0
	    InputString.zfill(len(InputString))
		  
    #解析上位机发来的七彩探照灯指令并点亮相应的颜色
    #如:$4WD,CLR255,CLG0,CLB0# 七彩灯亮红色
    if (InputString.find("CLR", 0, len(InputString)) != -1):
        i = InputString.find("CLR", 0,  len(InputString)) 
        ii = InputString.find(",CLG",  0,  len(InputString))
	if ii > i:
           string = InputString[i+3:ii]
	   m_kp = int(string)
	   red = m_kp
        i = InputString.find("CLG",  0, len(InputString)) 
        ii = InputString.find(",CLB",  0, len(InputString))
	if ii > i:
           string = InputString[i+3:ii]
	   m_kp = int(string)
	   green = m_kp
        i = InputString.find("CLB",  0, len(InputString)) 
        ii = InputString.find("#",  0,  len(InputString))
	if ii > i:
            string = InputString[i+3:ii]
	    m_kp = int(string)
	    blue = m_kp
        color_led_pwm(red, green, blue)		  
        NewLineReceived = 0
        InputString.zfill(len(InputString))
		  
    #解析上位机发来的通用协议指令,并执行相应的动作
    #如:$1,0,0,0,0,0,0,0,0,0#    小车前进
    if (InputString.find("$4WD", 0, len(InputString)) == -1) and (InputString.find("#",  0, len(InputString)) != -1):
        if InputString[3] == '1':
            g_CarState = enTLEFT      #小车原地左旋
        elif InputString[3] == '2':
            g_CarState = enTRIGHT     #小车原地右旋
        else:
            g_CarState = enSTOP

        if InputString[5] == '1':    
            whistle()                 #小车鸣笛

        if InputString[7] == '1':
            CarSpeedControl += 20
            if CarSpeedControl > 100:
                CarSpeedControl = 100 #小车加速
        if InputString[7] == '2':
            CarSpeedControl -= 20
            if CarSpeedControl < 20:  #小车减速
                CarSpeedControl = 20
        #小车点亮指定颜色
        if InputString[13] == '1':
            g_lednum=g_lednum+1
            if g_lednum == 1:
                color_led_pwm(255, 255, 255)
            elif g_lednum == 2:
                color_led_pwm(255, 0, 0)
            elif g_lednum == 3:
                color_led_pwm(0, 255, 0)
            elif g_lednum == 4:
                color_led_pwm(0, 0, 255)
            elif g_lednum == 5:
                color_led_pwm(255, 255, 0)
            elif g_lednum == 6:
                color_led_pwm(0, 255, 255)
            elif g_lednum == 7:
                color_led_pwm(255, 0, 255)
            else : 
                color_led_pwm(0, 0 ,0)
                g_lednum = 0

        if InputString[13] == '2':
            color_led_pwm(255, 0, 0)
        if InputString[13] == '3':
            color_led_pwm(0, 255, 0)
        if InputString[13] == '4':
            color_led_pwm(0, 0, 255)	
        #灭火
        if InputString[15] == '1':
            GPIO.output(OutfirePin,not GPIO.input(OutfirePin) )
            time.sleep(1)
        #前舵机归位
        if InputString[17] == '1':
            g_frontServoPos = 90

        #小车状态数据解析
        if g_CarState != enTLEFT and g_CarState != enTRIGHT:
            if InputString[1] == run_car:
                g_CarState = enRUN
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
        #舵机状态数据解析	
        			
	if InputString[9] == front_left_servo:
	    g_frontServoPos = 180
	elif InputString[9] == front_right_servo:
	    g_frontServoPos = 0
        elif InputString[9] == up_servo:
	    g_ServoState = enSERVOUP
	elif InputString[9] == down_servo:
	    g_ServoState = enSERVODOWN
        elif InputString[9] == left_servo:
	    g_ServoState = enSERVOLEFT
        elif InputString[9] == right_servo:
	    g_ServoState = enSERVORIGHT
        elif InputString[9] == updowninit_servo:
	    g_ServoState = enSERVOUPDOWNINIT
	elif InputString[9] == stop_servo:
	    g_ServoState = enSERVOSTOP
	else:
	    g_ServoState = enSERVOSTOP
            
        NewLineReceived = 0
        InputString.zfill(len(InputString))

	#根据解析的数据让小车做出相应的运动	
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

#对Tcp传过来的数据封包
def Data_Pack():
    global InputString
    global NewLineReceived
    if recvbuf[0] == '$' and recvbuf.find("#",  0, len(recvbuf)) != -1:
        InputString = recvbuf
        NewLineReceived = 1
        print "InputString: %s" % InputString
		
#采集的传感器数据串口回发给上位机显示
def tcp_data_postback():
    #小车超声波传感器采集的信息发给上位机显示
    #打包格式如:
    #    超声波 电压  灰度  巡线  红外避障 寻光
    #$4WD,CSB120,PV8.3,GS214,LF1011,HW11,GM11#
    global ReturnTemp
    ReturnTemp = ''
    distance = Distance_test()
    ReturnTemp += "$4WD,CSB"
    ReturnTemp += str(int(distance))
    ReturnTemp += ",PV8.4"
    ReturnTemp += ",GS0"
    ReturnTemp += ",LF"
    tracking_test()
    ReturnTemp += infrared_track_value
    ReturnTemp += ",HW"
    infrared_avoid_test()
    ReturnTemp += infrared_avoid_value
    ReturnTemp += ",GM"
    follow_light_test()
    ReturnTemp += LDR_value
    ReturnTemp += "#"
    print "ReturnTemp: %s" % ReturnTemp
    return ReturnTemp
#定义舵机控制线程
def ServorThread():
    #舵机状态判断并执行相应的函数
    global g_frontServoPos
    global g_nowfrontPos
    
    if g_ServoState == enSERVOUP:
        servo_up()
    elif g_ServoState == enSERVODOWN:
	servo_down()
    elif g_ServoState == enSERVOLEFT:
	servo_left()
    elif g_ServoState == enSERVORIGHT:
	servo_right()
    elif g_ServoState == enSERVOUPDOWNINIT:
	servo_updown_init()
    elif g_ServoState == enSERVOSTOP:
    	servo_stop()
    
    if g_nowfrontPos != g_frontServoPos:
    	frontservo_appointed_detection(g_frontServoPos)
    	g_nowfrontPos = g_frontServoPos
    	pwm_FrontServo.ChangeDutyCycle(0)	#归零信号
            
    

		  
try:
    init()
    servo_init()
    global g_ServoState
    global timecount
    global connectflag 
    connectflag = 0
    timecount = 1000
    count = 50
    #通过socket创建监听套接字并设置为非阻塞模式
    tcpservicesock= socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    tcpservicesock.setblocking(0)
    #填充绑定服务器的ip地址和端口号
    #注意：这里一定要根据自己的树莓派的ip地址来填
    tcpservicesock.bind(('192.168.50.1', 8888))
    #监听客户端的连接
    tcpservicesock.listen(5)
    print "waiting for connection...."
    #创建监听列表
    clientAddrList = []
    thread1 = threading.Thread(target = ServorThread)
    thread1.start()
    thread1.join()

    while True:
        try:
            #准备接受客户端的连接并返回连接套接字
            print "Start accept!"
            tcpclientsock,addr = tcpservicesock.accept()
            if  tcpclientsock:
                connectflag = 1
        except:
            pass 
        else:
            print "new user :%s " % str(addr)
            #设置连接套接字为非阻塞的模式并将连接的套接字放入到监听列表中
            tcpclientsock.setblocking(0)
            clientAddrList.append((tcpclientsock,addr))
        for tcpclientsock,addr in clientAddrList:
            try:
                global recvbuf
                global sendbuf
                recvbuf = ''
                #TCP接受数据
                print "Start recv!"
                recvbuf = tcpclientsock.recv(128)
                print "Start recv over!"
            except:
                pass
            else:
                if len(recvbuf) > 0:
                     #数据打包
                     Data_Pack()
                     if NewLineReceived == 1:
                         #调用数据解析函数
                         tcp_data_parse()
                else:
                    tcpclientsock.close()
                    clientAddrList.remove((tcpclientsock,addr))
        #延时并调用传感器采集的数据回发客户端
        timecount -= 1
        if timecount == 0:
           count -= 1
           timecount = 1000
           if count == 0:
               sendbuf = ''
               sendbuf = tcp_data_postback()
               if not sendbuf:
                   break
               if  connectflag:
                   #采集数据给客户端
                   tcpclientsock.send(sendbuf)
               timecount = 1000
               count = 50
        ServorThread()
except KeyboardInterrupt:
    pass
tcpclientsock.close()
tcpservicesock.close()
pwm_ENA.stop()
pwm_ENB.stop()
pwm_rled.stop()
pwm_gled.stop()
pwm_bled.stop()
pwm_FrontServo.stop()
pwm_LeftRightServo.stop()
pwm_UpDownServo.stop()
GPIO.cleanup()	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	




