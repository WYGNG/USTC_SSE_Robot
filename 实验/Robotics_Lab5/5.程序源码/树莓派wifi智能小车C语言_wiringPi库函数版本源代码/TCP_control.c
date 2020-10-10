/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         tcp_control.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        tcp控制智能小车实验
* @details
* @par History  见如下说明
*
*/
#include <wiringPi.h>
#include <softPwm.h>
#include <sys/time.h>
#include <stdio.h>
#include <string.h>
#include <wiringSerial.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>	       
#include <stdlib.h>
#include <pthread.h>

#define N 1024

#define run_car     '1'//按键前
#define back_car    '2'//按键后
#define left_car    '3'//按键左
#define right_car   '4'//按键右
#define stop_car    '0'//按键停

#define front_left_servo  '1'
#define front_right_servo '2'
#define up_servo    '3'
#define down_servo  '4'
#define left_servo  '6'
#define right_servo  '7'
#define updowninit_servo '5'
#define stop_servo  '8'

/*小车运行状态枚举*/
enum {
  enSTOP = 0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enTLEFT,
  enTRIGHT
} enCarState;

/*小车舵机状态枚举*/
enum {
  enFRONTSERVOLEFT = 1,
  enFRONTSERVORIGHT,
  enSERVOUP,
  enSERVODOWN,
  enSERVOUPDOWNINIT,
  enSERVOLEFT,
  enSERVORIGHT,
  enSERVOSTOP,
  enSERVOFRONTINIT
} enServoState;

//定义引脚
int Left_motor_go = 28;       //左电机前进AIN1连接Raspberry的wiringPi编码28口
int Left_motor_back = 29;     //左电机后退AIN2连接Raspberry的wiringPi编码29口

int Right_motor_go = 24;      //右电机前进BIN1连接Raspberry的wiringPi编码24口
int Right_motor_back = 25;    //右电机后退BIN2连接Raspberry的wiringPi编码25口

int Left_motor_pwm = 27;      //左电机控速PWMA连接Raspberry的wiringPi编码27口
int Right_motor_pwm = 23;     //右电机控速PWMB连接Raspberry的wiringPi编码23口

/*循迹红外传感器引脚及变量设置*/
//TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
//      9                  21                  7                   1
const int TrackSensorLeftPin1  =  9;    //定义左边第一个循迹红外传感器引脚为wiringPi编码9口
const int TrackSensorLeftPin2  =  21;  //定义左边第二个循迹红外传感器引脚为wiringPi编码21口
const int TrackSensorRightPin1 =  7;   //定义右边第一个循迹红外传感器引脚为wiringPi编码7口
const int TrackSensorRightPin2 =  1;   //定义右边第二个循迹红外传感器引脚为wiringPi编码1口

//定义各个循迹红外引脚采集的数据的变量
int TrackSensorLeftValue1;
int TrackSensorLeftValue2;
int TrackSensorRightValue1;
int TrackSensorRightValue2;
char infrared_track_value[5] = {0};

/*避障红外传感器引脚及变量设置*/
const int AvoidSensorLeft =  26; //定义左边避障的红外传感器引脚为wiringPi编码26口
const int AvoidSensorRight = 0;  //定义右边避障的红外传感器引脚为wiringPi编码0口

int LeftSensorValue ;            //定义变量来保存红外传感器采集的数据大小
int RightSensorValue ;
char infrared_avoid_value[3] = {0};

/*定义光敏电阻引脚及变量设置*/
const int LdrSensorLeft =  11;   //定义左边光敏电阻引脚为wiringPi编码11口
const int LdrSensorRight = 22;   //定义右边光敏电阻引脚为wiringPi编码22口

int LdrSersorLeftValue ;         //定义变量来保存光敏电阻采集的数据大小
int LdrSersorRightValue ;
char LDR_value[3] = {0};

/*蜂鸣器引脚设置*/
int buzzer = 10;                //设置控制蜂鸣器引脚为wiringPi编码10口

/*小车初始速度控制*/
int CarSpeedControl = 150;

/*设置舵机驱动引脚*/
int FrontServoPin = 4;
int ServoUpDownPin = 13;
int ServoLeftRightPin = 14;

/*摄像头舵机上下和左右两个自由度的变量*/
int ServoUpDownPos = 90;
int ServoLeftRightPos = 90;
/*前舵机左右摇动变量*/
int FrontServoLeftRightPos = 90;

/*舵机控制标志位*/
int ServoFlags;

/*超声波引脚及变量设置*/
int EchoPin = 30;         //定义回声脚为连接Raspberry的wiringPi编码30口
int TrigPin = 31;         //定义触发脚为连接Raspberry的wiringPi编码31口

/*RGBLED引脚设置*/
int LED_R = 3;           //LED_R接在Raspberry上的wiringPi编码3口
int LED_G = 2;           //LED_G接在Raspberry上的wiringPi编码2口
int LED_B = 5;           //LED_B接在Raspberry上的wiringPi编码5口
int g_lednum = 0;        //led颜色切换变量

/*灭火电机引脚设置*/
int OutfirePin = 8;      //设置灭火电机引脚为wiringPi编码8口

char recvbuf[N] = {0};        //用来储存接收到的内容
char InputString[N] = {0};    //用来储存接收到的有效数据包
int NewLineReceived = 0;      //前一次数据结束标志
int g_CarState = enSTOP;      //1前2后3左4右0停止
int g_ServoState = enSERVOSTOP;
char ReturnTemp[N] = {0};     //存储返回值
unsigned char g_frontservopos = 90;

/**
* Function       servo_pulse
* @author        Danny
* @date          2017.08.16
* @brief         定义一个脉冲函数，用来模拟方式产生PWM值
*                时基脉冲为20ms,该脉冲高电平部分在0.5-2.5ms
*                控制0-180度
* @param[in1]    ServPin:舵机控制引脚
* @param[in2]    myangle:舵机转动指定的角度
* @param[out]    void
* @retval        void
* @par History   无
*/
void servo_pulse(int ServoPin, int myangle)
{
  int PulseWidth;                    //定义脉宽变量
  PulseWidth = (myangle * 11) + 500; //将角度转化为500-2480 的脉宽值
  digitalWrite(ServoPin, HIGH);      //将舵机接口电平置高
  delayMicroseconds(PulseWidth);     //延时脉宽值的微秒数
  digitalWrite(ServoPin, LOW);       //将舵机接口电平置低
  delay(20 - PulseWidth / 1000);     //延时周期内剩余时间
  return;
}

/**
* Function       servo_appointed_detection
* @author        Danny
* @date          2017.08.16
* @brief         舵机旋转到指定角度
* @param[in]     pos：指定的角度
* @param[out]    void
* @retval        void
* @par History   无
*/
void servo_appointed_detection(int pos)
{
  int i = 0;
  for (i = 0; i <= 20; i++)    //产生PWM个数，等效延时以保证能转到响应角度
  {
    servo_pulse(FrontServoPin, pos); //模拟产生PWM
  } 
}

/**
* Function       Distance
* @author        Danny
* @date          2017.08.16
* @brief         超声波测一次前方的距离
* @param[in]     void
* @param[out]    void
* @retval        float:distance返回距离值
* @par History   无
*/
float Distance()
{
	float distance;
	struct timeval tv1;
	struct timeval tv2;
	struct timeval tv3;
	struct timeval tv4;
	long start, stop;
	
	digitalWrite(TrigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(TrigPin, HIGH);      //向Trig脚输入至少10US的高电平
	delayMicroseconds(10);
	digitalWrite(TrigPin, LOW);
    
	//防止程序未检测到电平变化，陷入死循环，加入一个超时重测机制
    gettimeofday(&tv3, NULL);        //超时重测机制开始计时
	start = tv3.tv_sec * 1000000 + tv3.tv_usec;
	while(!digitalRead(EchoPin) == 1)
	{
		gettimeofday(&tv4, NULL);    //超时重测机制结束计时
		stop = tv4.tv_sec * 1000000 + tv4.tv_usec;
		
		if ((stop - start) > 30000)  //最大测5米时的时间值：10/340=0.03s
		{
			return -1;               //超时返回-1
		}
	} 
	
	//防止程序未检测到电平变化，陷入死循环，加入一个超时重测机制
	gettimeofday(&tv1, NULL);      //当echo脚电平变高时开始计时
    start = tv1.tv_sec*1000000+tv1.tv_usec;
	while(!digitalRead(EchoPin) == 0)
	{
		gettimeofday(&tv3,NULL);   //超时重测机制开始计时
		stop = tv3.tv_sec*1000000+tv3.tv_usec;
		if ((stop - start) > 30000)
		{
			return -1;
		}
	}                              //超时重测机制结束计时
	gettimeofday(&tv2, NULL);      //当echo脚电平变低时结束计时

	start = tv1.tv_sec * 1000000 + tv1.tv_usec;
	stop = tv2.tv_sec * 1000000 + tv2.tv_usec;

	distance = (float)(stop - start)/1000000 * 34000 / 2;
	printf("distance: %f\n", distance);
	return distance;
}

/**
* Function       track_test
* @author        Danny
* @date          2017.08.16
* @brief         循迹模式引脚测试
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void track_test()
{
  //检测到黑线时循迹模块相应的指示灯亮，端口电平为LOW
  //未检测到黑线时循迹模块相应的指示灯灭，端口电平为HIGH
  TrackSensorLeftValue1 = digitalRead(TrackSensorLeftPin1);
  TrackSensorLeftValue2 = digitalRead(TrackSensorLeftPin2);
  TrackSensorRightValue1 = digitalRead(TrackSensorRightPin1);
  TrackSensorRightValue2 = digitalRead(TrackSensorRightPin2);

  infrared_track_value[0] =((TrackSensorLeftValue1 == LOW)? '1' : '0');
  infrared_track_value[1] =((TrackSensorLeftValue2 == LOW)? '1' : '0');
  infrared_track_value[2] =((TrackSensorRightValue1 == LOW)?'1': '0');
  infrared_track_value[3] =((TrackSensorRightValue2 == LOW)? '1': '0');
  return;
}

/**
* Function       infrared_avoid_test
* @author        Danny
* @date          2017.08.16
* @brief         避障红外引脚测试
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void infrared_avoid_test()
{ 
  //遇到障碍物,红外避障模块的指示灯亮,端口电平为LOW
  //未遇到障碍物,红外避障模块的指示灯灭,端口电平为HIGH
  LeftSensorValue  = digitalRead(AvoidSensorLeft);
  RightSensorValue = digitalRead(AvoidSensorRight);
  infrared_avoid_value[0]=((LeftSensorValue == LOW) ?'1' : '0');
  infrared_avoid_value[1]=((RightSensorValue == LOW) ? '1' : '0');
  return;
}

/**
* Function       follow_light_test
* @author        Danny
* @date          2017.08.16
* @brief         寻光引脚测试
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void follow_light_test()
{
  //遇到光线,寻光模块的指示灯灭,端口电平为HIGH
  //未遇光线,寻光模块的指示灯亮,端口电平为LOW
  LdrSersorRightValue = digitalRead(LdrSensorRight);
  LdrSersorLeftValue  = digitalRead(LdrSensorLeft);

  LDR_value[0]=((LdrSersorLeftValue == LOW) ? '0' :  '1');
  LDR_value[1]=((LdrSersorRightValue == LOW)? '0' :  '1');
  return;
}


/**
* Function       run
* @author        Danny
* @date          2017.08.16
* @brief         小车前进
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void run()
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);   //左电机前进使能
  digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       brake
*  @author       Danny
* @date          2017.08.16
* @brief         小车刹车
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void brake()
{
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Left_motor_back, LOW);
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, LOW);
}

/**
* Function       left
* @author        Danny
* @date          2017.08.16
* @brief         小车左转(左轮不动,右轮前进)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void left()
{
  //左电机停止
  digitalWrite(Left_motor_go, LOW);     //左电机前进禁止
  digitalWrite(Left_motor_back, LOW);   //左电机后退禁止
  softPwmWrite(Left_motor_pwm, 0);

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       spin_left
* @author        Danny
* @date          2017.08.16
* @brief         小车原地左转(左轮后退，右轮前进)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void spin_left()
{
  //左电机后退
  digitalWrite(Left_motor_go, LOW);     //左电机前进禁止
  digitalWrite(Left_motor_back, HIGH);  //左电机后退使能
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       right
* @author        Danny
* @date          2017.08.16
* @brief         小车右转(左轮前进,右轮不动)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void right()
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);    //左电机前进使能
  digitalWrite(Left_motor_back, LOW);   //左电机后退禁止
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  //右电机停止
  digitalWrite(Right_motor_go, LOW);    //右电机前进禁止
  digitalWrite(Right_motor_back, LOW);  //右电机后退禁止
  softPwmWrite(Right_motor_pwm, 0);
}

/**
* Function       spin_right
* @author        Danny
* @date          2017.08.16
* @brief         小车原地右转(右轮后退，左轮前进)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void spin_right()
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);    //左电机前进使能
  digitalWrite(Left_motor_back, LOW);   //左电机后退禁止
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  //右电机后退
  digitalWrite(Right_motor_go, LOW);    //右电机前进禁止
  digitalWrite(Right_motor_back, HIGH); //右电机后退使能
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       back
* @author        Danny
* @date          2017.08.16
* @brief         小车后退
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void back()
{
  //左电机后退
  digitalWrite(Left_motor_go, LOW);     //左电机前进禁止
  digitalWrite(Left_motor_back, HIGH);  //左电机后退使能
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  //右电机后退
  digitalWrite(Right_motor_go, LOW);    //右电机前进禁止
  digitalWrite(Right_motor_back, HIGH); //右电机后退使能
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       whistle
* @author        Danny
* @date          2017.08.16
* @brief         小车鸣笛
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void whistle()
{
  digitalWrite(buzzer, LOW);   //发声音
  delay(100);                  //延时100ms
  digitalWrite(buzzer, HIGH);  //不发声音
  delay(1);                    //延时1ms

  digitalWrite(buzzer, LOW);   //发声音
  delay(200);                  //延时200ms
  digitalWrite(buzzer, HIGH);  //不发声音
  delay(2);                    //延时2ms
  return;
}

/**
* Function       servo_up
* @author        Danny
* @date          2017.08.16
* @brief         摄像头舵机上旋
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void servo_up()
{
    int pos, i;
    pos = ServoUpDownPos;
	servo_pulse(ServoUpDownPin, pos); //模拟产生PWM
	pos += 2;
	ServoUpDownPos = pos;
	if (ServoUpDownPos >= 180)
	{
		ServoUpDownPos = 180;
	}
}

/**
* Function       servo_down
* @author        Danny
* @date          2017.08.16
* @brief         摄像头舵机下旋
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void servo_down()
{
    int pos, i;
    pos = ServoUpDownPos;
   	servo_pulse(ServoUpDownPin, pos); //模拟产生PWM
	pos -= 2;
	ServoUpDownPos = pos;
	if (ServoUpDownPos <= 45)
	{
		ServoUpDownPos = 45;
	}
}

/**
* Function       servo_left
* @author        Danny
* @date          2017.08.16
* @brief         摄像头舵机左旋
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void servo_left()
{
    int pos, i;
    pos = ServoLeftRightPos;
   	servo_pulse(ServoLeftRightPin, pos); //模拟产生PWM
	pos += 2;
	ServoLeftRightPos = pos;
	if (ServoLeftRightPos >= 180)
	{
		ServoLeftRightPos = 180;
	}
}

/**
* Function       servo_right
* @author        Danny
* @date          2017.08.16
* @brief         摄像头舵机右旋
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void servo_right()
{  
    int pos, i;
    pos = ServoLeftRightPos;
   	servo_pulse(ServoLeftRightPin, pos); //模拟产生PWM
	pos -= 2;
	ServoLeftRightPos = pos;
	if (ServoLeftRightPos <= 0)
	{
		ServoLeftRightPos = 0;
	}
}

/**
* Function       front_servo_left
* @author        Danny
* @date          2017.08.16
* @brief         前舵机左旋
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void front_servo_left()
{
    servo_appointed_detection(180);
}

/**
* Function       front_servo_right
* @author        Danny
* @date          2017.08.16
* @brief         前舵机右旋
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void front_servo_right()
{ 
      servo_appointed_detection(0);
}
/**
* Function       servo_init
* @author        Danny
* @date          2017.08.16
* @brief         舵机位置初始化
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void servo_init()
{
	int i = 0;
	for (i = 0; i < 10; i++)
	{
	servo_pulse(ServoLeftRightPin, 90); 
	}
	for (i = 0; i < 10; i++)
	{
	servo_pulse(ServoUpDownPin, 90);
	}
	for (i = 0; i < 10; i++)
	{
	servo_pulse(FrontServoPin, 90);
	}
}

/**
* Function       servo_updown_init
* @author        Danny
* @date          2017.08.16
* @brief         摄像头舵机上下复位
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void servo_updown_init()
{
	int i = 0;
	for (i = 0; i < 10; i++)
	{
		servo_pulse(ServoUpDownPin, 90);
	}
}

/**
* Function       servo_front_init
* @author        Danny
* @date          2017.08.16
* @brief         前舵机复位
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void servo_front_init()
{
	servo_pulse(FrontServoPin, 90);
}

/**
* Function       servo_stop
* @author        Danny
* @date          2017.08.16
* @brief         舵机停止
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void servo_stop()
{
}

/**
* Function       color_led_pwm
* @author        Danny
* @date          2017.08.16
* @brief         七彩灯亮指定的颜色
* @param[in1]    v_iRed:指定的颜色值（0-255）
* @param[in2]    v_iGreen:指定的颜色值（0-255）
* @param[in3]    v_iBlue:指定的颜色值（0-255）
* @param[out]    void
* @retval        void
* @par History   无
*/
void color_led_pwm(int v_iRed, int v_iGreen, int v_iBlue)
{
  softPwmWrite(LED_R, v_iRed);
  softPwmWrite(LED_G, v_iGreen);
  softPwmWrite(LED_B, v_iBlue);
  return;
}

/**
* Function       StringFind
* @author        Danny
* @date          2017.08.16    
* @brief         字符串查找
* @param[in]     pSrc:源字符串; pDst:查找的字符串; v_iStartPos:源字符串起始位置
* @param[out]    void
* @retval        void
* @par History   无
*/
int StringFind(const char *pSrc, const char *pDst, int v_iStartPos)  
{  
    int i, j;  
    for (i = v_iStartPos; pSrc[i]!='\0'; i++)  
    {  
        if(pSrc[i]!=pDst[0])  
            continue;         
        j = 0;  
        while(pDst[j] !='\0' && pSrc[i+j]!='\0')  
        {  
            j++;  
            if(pDst[j]!=pSrc[i+j])  
            break;  
        }  
        if(pDst[j]=='\0')  
            return i;  
    }  
    return -1;  
} 

/**
* Function       tcp_data_parse
* @author        Danny
* @date          2017.08.16
* @brief         tcp数据解析并指定相应的动作
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void tcp_data_parse()
{
  //解析客户端发来的舵机云台的控制指令并执行舵机旋转
  //如:$4WD,PTZ180# 舵机转动到180度
  	if (StringFind((const char *)InputString, (const char *)"PTZ", 0) > 0)
	{
		int m_kp, i, ii;
        //寻找以PTZ开头,#结束中间的字符
		i = StringFind((const char *)InputString, (const char *)"PTZ", 0); 
		ii = StringFind((const char *)InputString, (const char *)"#", i);
		if (ii > i)
		{
			char m_skp[5] = {0};
			memcpy(m_skp, InputString + i + 3, ii - i -3);
			
			m_kp = atoi(m_skp);        //将找到的字符串变成整型

			g_frontservopos = 180 - m_kp;
			NewLineReceived = 0;  
			memset(InputString, 0x00, sizeof(InputString));
			return;
		}
  	}

  //解析客户端发来的七彩探照灯指令并点亮相应的颜色
  //如:$4WD,CLR255,CLG0,CLB0# 七彩灯亮红色
  	 if (StringFind((const char *)InputString, (const char *)"CLR", 0) > 0)
 	{
		int m_kp, i, ii, red, green, blue;
		char m_skp[5] = {0};
		i = StringFind((const char *)InputString, (const char *)"CLR", 0);
		ii = StringFind((const char *)InputString, (const char *)",", i);
		if (ii > i)
		{			
			memcpy(m_skp, InputString + i + 3, ii - i -3);
			m_kp = atoi(m_skp);
			red =   m_kp;
		}
		i = StringFind((const char *)InputString, (const char *)"CLG", 0);
		ii = StringFind((const char *)InputString, (const char *)",", i);
		if (ii > i)
		{
			memcpy(m_skp, InputString + i + 3, ii - i -3);
			m_kp = atoi(m_skp);
			green =   m_kp;
		}
		i = StringFind((const char *)InputString, (const char *)"CLB", 0);
		ii = StringFind((const char *)InputString, (const char *)"#", i);
		if (ii > i)
		{
			memcpy(m_skp, InputString + i + 3, ii - i -3);
			m_kp = atoi(m_skp);
			blue =  m_kp;
			color_led_pwm(red, green, blue);//点亮相应颜色的灯
			NewLineReceived = 0;  
			memset(InputString, 0x00, sizeof(InputString));
			return;
		}
	}

  //解析客户端发来的通用协议指令,并执行相应的动作
  //如:$1,0,0,0,0,0,0,0,0,0#    小车前进
  if (StringFind((const char *)InputString, (const char *)"4WD", 0) == -1 &&
		                    StringFind((const char *)InputString,(const char *)"#",0) > 0)
  {
    //小车原地左旋右旋判断
    if (InputString[3] == '1')      //小车原地左旋
    {
      g_CarState = enTLEFT;
    }
    else if (InputString[3] == '2') //小车原地右旋
    {
      g_CarState = enTRIGHT;
    }
    else
    {
      g_CarState = enSTOP;
    }

    //小车鸣笛判断
    if (InputString[5] == '1')     //鸣笛
    {
      whistle();
    }

    //小车加减速判断
    if (InputString[7] == '1')     //加速，每次加50
    {
      CarSpeedControl += 50;
      if (CarSpeedControl > 255)
      {
        CarSpeedControl = 255;
      }
    }
    if (InputString[7] == '2')    //减速，每次减50
    {
      CarSpeedControl -= 50;
      if (CarSpeedControl < 50)
      {
        CarSpeedControl = 50;
      }
    }
	
    //点灯判断
    if (InputString[13] == '1')//七彩灯亮白色
   
    {
		g_lednum++;
        if(g_lednum == 1){
		color_led_pwm(255, 255, 255);
		}
		else if(g_lednum == 2)
		{
			color_led_pwm(255,0,0);
		}
		else if(g_lednum == 3)
        {
			color_led_pwm(0,255,0);
		}
		else if(g_lednum == 4)
		{
			color_led_pwm(0,0,255);
		}
		else if(g_lednum == 5)
		{
			color_led_pwm(255,255,0);
		}
		else if(g_lednum == 6)
		{
			color_led_pwm(0,255,255);
		}
		else if(g_lednum == 7)
		{
			color_led_pwm(255,0,255);
		}
		else{
		   color_led_pwm(0,0,0);
			   g_lednum=0;
		}
    }
    if (InputString[13] == '2')//七彩灯亮红色
    {
      color_led_pwm(255, 0, 0);
    }
    if (InputString[13] == '3')//七彩灯亮绿灯
    {
      color_led_pwm(0, 255, 0);
    }
    if (InputString[13] == '4') //七彩灯亮蓝灯
    {
      color_led_pwm(0, 0, 255);
    }

    //灭火判断
    if (InputString[15] == '1')  //灭火
    {
      digitalWrite(OutfirePin, !digitalRead(OutfirePin));
    }

    //前舵机左右摇复位
    if (InputString[17] == '1') //舵机旋转到90度
    {
		g_frontservopos = 90; 
	}

    //小车的前进,后退,左转,右转,停止动作
    if (g_CarState != enTLEFT && g_CarState != enTRIGHT)
    {
      switch (InputString[1])
      {
        case run_car:   g_CarState = enRUN;  break;
        case back_car:  g_CarState = enBACK;  break;
        case left_car:  g_CarState = enLEFT;  break;
        case right_car: g_CarState = enRIGHT;  break;
        case stop_car:  g_CarState = enSTOP;  break;
        default: g_CarState = enSTOP; break;
      }
    }

	//前后舵机前后左右转动
	switch(InputString[9])
	{
		case front_left_servo: g_frontservopos = 180 ;break;
		case front_right_servo: g_frontservopos = 0;break;
	    case up_servo:  g_ServoState = enSERVOUP; break;
		case down_servo: g_ServoState = enSERVODOWN;break;
		case left_servo: g_ServoState = enSERVOLEFT;break;
		case right_servo: g_ServoState = enSERVORIGHT;break;
		case updowninit_servo: g_ServoState = enSERVOUPDOWNINIT;break;
		case stop_servo: g_ServoState = enSERVOSTOP;break;
	}

    memset(InputString, 0x00, sizeof(InputString));       //清空串口数据
    NewLineReceived = 0;

    //根据小车状态做相应的动作
    switch (g_CarState)
    {
      case enSTOP: brake(); break;
      case enRUN: run(); break;
      case enLEFT: left(); break;
      case enRIGHT: right(); break;
      case enBACK: back(); break;
      case enTLEFT: spin_left(); break;
      case enTRIGHT: spin_right(); break;
      default: brake(); break;
    }
  }
}

/**
* Function       itoa
* @author        Danny
* @date          2017.08.16
* @brief         将整型数转换为字符
* @param[in]     void
* @retval        void
* @par History   无
*/
void itoa(int i, char *string)
{
	sprintf(string,"%d", i);
	return;
}

/**
* Function       tcp_data_postback
* @author        Danny
* @date          2017.08.16
* @brief         将采集的传感器数据通过tcp传输给上位机显示
* @param[in]     void
* @retval        void
* @par History   无
*/
char *tcp_data_postback()
{
  //小车超声波传感器采集的信息发给上位机显示
  //打包格式如:
  //    超声波 电压  灰度  巡线  红外避障 寻光
  //$4WD,CSB120,PV8.3,GS214,LF1011,HW11,GM11#
  char *p= ReturnTemp;
  char str[25];
  float distance;
  memset(ReturnTemp,0,sizeof(ReturnTemp));
  //超声波
  distance = Distance();
  if((int)distance == -1)
  {
	  distance = 0;
  }
  strcat(p, "$4WD,CSB");
  itoa((int)distance, str);
  strcat(p, str);
  //电压
  strcat(p, ",PV8.4");
  //灰度
  strcat(p, ",GS0");
  //巡线
  strcat(p, ",LF");
  track_test();
  strcat(p,infrared_track_value);
  //红外避障
  strcat(p, ",HW");
  infrared_avoid_test();
  strcat(p,infrared_avoid_value);
  //寻光
  strcat(p, ",GM");
  follow_light_test();
  strcat(p, LDR_value);
  strcat(p, "#");
  //printf("ReturnTemp_first:%s\n",p);
  return ReturnTemp;
}


/**
* Function       Data_Pack
* @author        Danny
* @date          2017.08.16
* @brief         tcp数据封包
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void Data_Pack()
{ 
   if(recvbuf[0] == '$' && StringFind((const char *)recvbuf, (const char *)"#", 0) > 0)
   {
	    strcpy(InputString,recvbuf); 
		//printf("InputString:%s\n",InputString);
		NewLineReceived = 1;
   }
   else
   {
		NewLineReceived = 0;
   }
   return;
}

/**
* Function       do_client_recv
* @author        Danny
* @date          2017.08.16
* @brief         线程处理客户端发过来的数据
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void *do_client_recv(void *arg)
{
	int n = 0;
	int sockfd = *(int *)arg;
	//下面pthread_create传递过来的参数
    //即连接套接字的地址，sockfd就是连接套接字了
	while(1)
	{
		memset(recvbuf,0,sizeof(recvbuf));
		n = recv(sockfd,recvbuf,sizeof(recvbuf),0);	
		if(n < 0)
		{
			perror("Fail to recv!\n");	
		    break;
			//	exit(EXIT_FAILURE);
		}else if(n == 0){
			printf("clinet_recv is not connect\n");	
		    break;
			//	exit(EXIT_FAILURE);
		}		
		printf("Recv %d bytes : %s\n",n,recvbuf);
		//数据打包
		Data_Pack();
		if (NewLineReceived == 1)
		{
			tcp_data_parse();
		}
	}
	close(sockfd);
	free(arg);//释放堆区空间
	pthread_exit(NULL);
}

void *servo_control()
{
	int i_ServoState = 0;
	int i_frontservopos = 0;
    while(1)
	{
		//根据舵机的状态做相应的动作
	   switch (g_ServoState)
	   {
	    case enSERVOUP: 			
			servo_up();
			break;
        case enSERVODOWN: 			
			servo_down();
			break;
	    case enSERVOLEFT:			
			servo_left();
			break;
	    case enSERVORIGHT: 			
			servo_right();
	   		break;
	    case enSERVOUPDOWNINIT:
			servo_updown_init();
			break;
	    case enSERVOSTOP: 			
			servo_stop();
			break;
	   
	   }
		if (g_frontservopos != i_frontservopos)
		{
			servo_appointed_detection(g_frontservopos);
			i_frontservopos = g_frontservopos;
		}
      sleep(1);
	}
    
	 pthread_exit(NULL);
}

/**
* Function       do_client_postback
* @author        Danny
* @date          2017.08.16
* @brief         线程处理主控板采集的数据回发给客户端
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void *do_client_postback(void *arg)
{
	int n = 0;
	int sockfd = *(int *)arg;
	char str[1024] ={0};
	char *pstr = str;
	while(1)
	{   
        
        memset(str,0,sizeof(str));
		strcpy(str,tcp_data_postback());
		puts(str);
	
		n = send(sockfd,str,strlen(str),0);	
		if(n < 0)
		{
			perror("Fail to send!\n");	
	        break;
			//	exit(EXIT_FAILURE);
		}else if(n == 0){
			printf("clinet_postback is not connect\n");	
		    break;
			//	exit(EXIT_FAILURE);
		}
        printf("send %d bytes : %s\n",n,str);	
        sleep(4000);		
	}
//	close(sockfd);
//	free(arg);//释放堆区空间
	pthread_exit(NULL);
}
/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         对tcp发送过来的数据解析，并执行相应的指令
* @param[in1]    服务器编译生成的可执行程序
* @param[in2]    ip
* @param[in3]    port
* @retval        void
* @par History   无
*/
int main(int argc, const char *argv[])
{
   char buf[1024] = {0};
   //定义监听套接字
   int listen_fd = 0;
   //定义连接套接字
   int connect_fd = 0;
   //定义结构体存储服务器地址信息
   struct sockaddr_in my_addr;
   //定义结构体存储客户机地址信息
   struct sockaddr_in client_addr;
   //服务器地址空间大小
   int len = sizeof(my_addr);
   int n = 0 ;
   int *pconnect_fd = NULL;
   //定义线程id，分别用于发和收
   pthread_t tid1;
   pthread_t tid2;
   pthread_t tid3;
   int ret = 0;
   
   //wiringPi初始化
   wiringPiSetup();
  
  //初始化电机驱动IO为输出方式
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT);
  pinMode(Right_motor_go, OUTPUT);
  pinMode(Right_motor_back, OUTPUT);
  
  //创建两个软件控制的PWM脚
  softPwmCreate(Left_motor_pwm,0,255); 
  softPwmCreate(Right_motor_pwm,0,255);

  //定义左右传感器为输入接口
  pinMode(AvoidSensorLeft, INPUT);
  pinMode(AvoidSensorRight, INPUT);
  
  //定义寻迹红外传感器为输入模式
  pinMode(TrackSensorLeftPin1, INPUT);
  pinMode(TrackSensorLeftPin2, INPUT);
  pinMode(TrackSensorRightPin1, INPUT);
  pinMode(TrackSensorRightPin2, INPUT);
  
  //定义光敏电阻引脚为输入模式
  pinMode(LdrSensorLeft, INPUT);
  pinMode(LdrSensorRight, INPUT);
  
  //初始化蜂鸣器IO为输出方式
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, HIGH);
  

  //初始化超声波引脚模式
  pinMode(EchoPin, INPUT);   //定义超声波输入脚
  pinMode(TrigPin, OUTPUT);  //定义超声波输出脚

  //定义灭火IO口为输出模式并初始化
  pinMode(OutfirePin, OUTPUT);
  digitalWrite(OutfirePin,HIGH);
  //初始化RGB三色LED的IO口为输出方式，并初始化
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  softPwmCreate(LED_R,0,255); 
  softPwmCreate(LED_G,0,255); 
  softPwmCreate(LED_B,0,255); 

  //初始化舵机引脚为输出模式
  pinMode(FrontServoPin, OUTPUT);
  pinMode(ServoUpDownPin, OUTPUT);
  pinMode(ServoLeftRightPin, OUTPUT);
  
  //舵机位置初始化
  servo_init();
  
  //输入./可执行文件  
	if(argc < 1)
	{
		fprintf(stderr,"Usage : %s ip port!\n",argv[0]);	
		exit(EXIT_FAILURE);
	}

	//1.通过socket创建监听套接字
	listen_fd = socket(AF_INET,SOCK_STREAM,0);
	if(listen_fd < 0)
	{
		perror("Fail to socket");	
		exit(EXIT_FAILURE);
	}

	//2.填充服务器的ip地址和端口
	//注：此处填充的是我们的树莓派的ip地址和我们所指定的port
	//    ip地址以自己本机为准！
	memset(&my_addr,0,sizeof(my_addr));	
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(atoi("8888"));
	my_addr.sin_addr.s_addr = inet_addr("192.168.50.1");

	//3.把ip和port进行绑定
	if(bind(listen_fd,(struct sockaddr *)&my_addr,len) < 0)
	{
		perror("Fail to bind");	
		exit(EXIT_FAILURE);
	}

	//4.监听客户端的连接
	listen(listen_fd,5);
	printf("Listen....\n");
	
	while(1)
	{
		//由于每次有用户连接的时候，系统都会调用accept来返回连接套接字。
		//若是同时两个2个用户请求连接，可能存在，后一个线程把前一个线程
		//的数据给覆盖点，因此在堆区申请空间，每一个连接套接字拥有不同的
		//地址空间.
		pconnect_fd = (int *)malloc(sizeof(int));

		//5.准备接收客户端的连接请求
		*pconnect_fd = accept(listen_fd,(struct sockaddr *)&client_addr,&len);		
		if(*pconnect_fd < 0)
		{
			perror("Fail to accept");	
			exit(EXIT_FAILURE);
		}

		printf("=============================================\n");
		printf("connect_fd : %d\n",*pconnect_fd);
		printf("client IP : %s\n",inet_ntoa(client_addr.sin_addr));
		printf("client port : %d\n", ntohs(client_addr.sin_port));

		ret = pthread_create(&tid1,NULL,do_client_recv,(void *)pconnect_fd);
		
		//pconnect_fd是给线程函数do_client_recv传递的参数
		if(ret != 0)
		{
			fprintf(stderr,"Fail to pthread_create : %s\n",strerror(errno));	
			exit(EXIT_FAILURE);
		}
				
		ret = pthread_create(&tid3,NULL,servo_control,NULL);
		
		//pconnect_fd是给线程函数servo_control传递的参数
		if(ret != 0)
		{
			fprintf(stderr,"Fail to pthread_create : %s\n",strerror(errno));	
			exit(EXIT_FAILURE);
		}
		ret = pthread_create(&tid2,NULL,do_client_postback,(void *)pconnect_fd);
		//pconnect_fd是给线程函数do_client_postback传递的参数
		if(ret != 0)
		{
			fprintf(stderr,"Fail to pthread_create : %s\n",strerror(errno));	
			exit(EXIT_FAILURE);
		}
		//分离式线程：线程结束的时候由系统自动来释放资源
		pthread_detach(tid1);
		pthread_detach(tid3);
		pthread_detach(tid2);
	}
	close(listen_fd);
    exit(EXIT_SUCCESS);
}




