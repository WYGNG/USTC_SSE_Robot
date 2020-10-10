/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         master_system_control.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        上位机控制智能小车实验
* @details
* @par History  见如下说明
*
*/
#include <wiringPi.h>
#include <softPwm.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wiringSerial.h>
#include <errno.h>

#define run_car     '1'//按键前
#define back_car    '2'//按键后
#define left_car    '3'//按键左
#define right_car   '4'//按键右
#define stop_car    '0'//按键停

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

//定义引脚
int Left_motor_go = 28;       //左电机前进AIN2连接Raspberry的wiringPi编码28口
int Left_motor_back = 29;     //左电机后退AIN1连接Raspberry的wiringPi编码29口

int Right_motor_go = 24;      //右电机前进BIN2连接Raspberry的wiringPi编码24口
int Right_motor_back = 25;    //右电机后退BIN1连接Raspberry的wiringPi编码25口

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
int ServoPin = 4;

/*超声波引脚及变量设置*/
int EchoPin = 30;         //定义回声脚为连接Raspberry的wiringPi编码30口
int TrigPin = 31;         //定义触发脚为连接Raspberry的wiringPi编码31口

/*RGBLED引脚设置*/
int LED_R = 3;           //LED_R接在Raspberry上的wiringPi编码3口
int LED_G = 2;           //LED_G接在Raspberry上的wiringPi编码2口
int LED_B = 5;           //LED_B接在Raspberry上的wiringPi编码5口

/*灭火电机引脚设置*/
int OutfirePin = 8;      //设置灭火电机引脚为wiringPi编码8口

/*串口设备打开的文件描述符*/
int fd;

/*串口长度变量*/
int g_num=0;
int g_packnum=0;

/*计时变量*/
int time = 20000;
int count = 100;

/*串口数据设置*/
char InputString[512] = {0};  //用来储存接收到的内容
int NewLineReceived = 0;      //前一次数据结束标志
int StartBit  = 0;            //协议开始标志
int g_CarState = enSTOP;      //1前2后3左4右0停止
char ReturnTemp[512] = {0};   //存储返回值

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
 
  //由于上位机上的逻辑是高电平亮，所以这里我们需要更换逻辑
  infrared_track_value[0] = ((TrackSensorLeftValue1 == LOW)? '1' : '0');
  infrared_track_value[1] = ((TrackSensorLeftValue2 == LOW)? '1' : '0');
  infrared_track_value[2] = ((TrackSensorRightValue1 == LOW)?'1': '0');
  infrared_track_value[3] = ((TrackSensorRightValue2 == LOW)? '1': '0');
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
  
  //由于上位机上的逻辑是高电平亮，所以这里我们需要更换逻辑
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
*  @author        Danny
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
  for (i = 0; i <= 15; i++)    //产生PWM个数，等效延时以保证能转到响应角度
  {
    servo_pulse(ServoPin, pos); //模拟产生PWM
  }
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
* Function       serial_data_parse
* @author        Danny
* @date          2017.08.16
* @brief         串口数据解析并指定相应的动作
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void serial_data_parse()
{
  //解析上位机发来的舵机云台的控制指令并执行舵机旋转
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

			servo_appointed_detection(180 - m_kp);//转动到指定角度m_kp
			NewLineReceived = 0;  
			memset(InputString, 0x00, sizeof(InputString));
			return;
		}
  	}

  //解析上位机发来的七彩探照灯指令并点亮相应的颜色
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

  //解析上位机发来的通用协议指令,并执行相应的动作
  //如:$1,0,0,0,0,0,0,0,0,0#    小车前进
  if (StringFind((const char *)InputString, (const char *)"4WD", 0) == -1 &&
		                    StringFind((const char *)InputString,(const char *)"#",0) > 0)
  {
    //puts(InputString);
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
	
    //舵机左旋右旋判断
    if (InputString[9] == '1') //舵机旋转到180度
    {
      servo_appointed_detection(180);
    }
    if (InputString[9] == '2') //舵机旋转到0度
    {
      servo_appointed_detection(0);
    }

    //点灯判断
    if (InputString[13] == '1')//七彩灯亮白色
    {
      color_led_pwm(255, 255, 255);
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

    //舵机归为判断
    if (InputString[17] == '1') //舵机旋转到90度
    {
      servo_appointed_detection(90);
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
* Function       serial_data_postback
* @author        Danny
* @date          2017.08.16
* @brief         将采集的传感器数据通过串口传输给上位机显示
* @param[in]     void
* @retval        void
* @par History   无
*/
void serial_data_postback(int fd)
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
  //将ReturnTemp所指的内容写入到串口设备中
  printf("ReturnTemp:%s\n",p);
  serialPrintf(fd, p);
  return;
}

/**
* Function       serialEvent
* @author        Danny
* @date          2017.08.16
* @brief         串口解包
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void serialEvent()
{ 
  int UartReturnCount;
  char uartvalue = 0;
  while(1)
  {
    UartReturnCount = serialDataAvail(fd);
  if( UartReturnCount == 0)
   {
	  break;
   }
  else if (UartReturnCount > 0 )
   {
     while(UartReturnCount--)
	 {
	   uartvalue = (char)serialGetchar(fd);
	   if(uartvalue == '$')
	   {
		 StartBit = 1;
		 g_num = 0;
	   }
	  if(StartBit == 1)
	   {
		 InputString[g_num]= uartvalue;
	   }
      if(StartBit ==1&& uartvalue == '#')
	   {
		 NewLineReceived = 1;
		 StartBit = 0;
		 g_packnum = g_num;
	   //printf("inputstring:%s\n", InputString);
	   }
	   g_num++;
	
	   if(g_num >= 80)
	   {
		 g_num=0;
		 StartBit=0;
		 NewLineReceived=0;
	   }
     }
   }
  }
}

/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         对串口发送过来的数据解析，并执行相应的指令
* @param[in]     void
* @retval        void
* @par History   无
*/
int main()
{
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

  //初始化RGB三色LED的IO口为输出方式，并初始化
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  softPwmCreate(LED_R,0,255); 
  softPwmCreate(LED_G,0,255); 
  softPwmCreate(LED_B,0,255); 

  //初始化舵机引脚为输出模式
  pinMode(ServoPin, OUTPUT);
  
  //打开串口设备，如若失败则会打印错误信息
  if ((fd = serialOpen("/dev/ttyAMA0", 9600)) < 0)
  {
    fprintf(stderr, "Uable to open serial device: %s\n", strerror(errno));
	return -1;
  }
  while(1)
  {
   //调用串口解包函数
   serialEvent();
   if (NewLineReceived)
   {	
    printf("serialdata:%s\n",InputString);
    serial_data_parse();  //调用串口解析函数
	NewLineReceived = 0;
   }
   //让串口平均每秒发送采集的数据给上位机   
   time--;
   if(time ==0)
   {
	 count--;
	 time = 20000;
	 if(count == 0)
	 {
     	 serial_data_postback(fd);
		 time = 20000;
		 count = 100;
	 }
   }
  }
 serialClose(fd);  //关闭串口
 return 0;
}




