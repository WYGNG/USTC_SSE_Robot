/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         PS2_control.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        智能小车PS2控制实验
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
#include <errno.h>
#include <wiringPiSPI.h>//添加SPI库

/*定义手柄按键*/
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

/*PS2引脚定义*/
#define PS2_DAT_PIN   12//MOS
#define PS2_CMD_PIN   13//MIS
#define PS2_SEL_PIN   6 //CS
#define PS2_CLK_PIN   14//SCK

/*回发过来的后4个数据是摇杆的数据*/
#define PSS_RX 5        //右摇杆X轴数据
#define PSS_RY 6        //右摇杆Y轴数据
#define PSS_LX 7        //左摇杆X轴数据
#define PSS_LY 8        //右摇杆Y轴数据

/*小车运行状态枚举*/
enum {
  enSTOP = 1,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enTLEFT,
  enTRIGHT,
  enUPLEFT,
  enUPRIGHT,
  enDOWNLEFT,
  enDOWNRIGHT
}enCarState;

//定义引脚
int Left_motor_go = 28;       //左电机前进AIN1连接Raspberry的wiringPi编码28口
int Left_motor_back = 29;     //左电机后退AIN2连接Raspberry的wiringPi编码29口

int Right_motor_go = 24;      //右电机前进BIN1连接Raspberry的wiringPi编码24口
int Right_motor_back = 25;    //右电机后退BIN2连接Raspberry的wiringPi编码25口

int Left_motor_pwm = 27;      //左电机控速PWMA连接Raspberry的wiringPi编码27口
int Right_motor_pwm = 23;     //右电机控速PWMB连接Raspberry的wiringPi编码23口

/*蜂鸣器引脚设置*/
int buzzer = 10;              //设置控制蜂鸣器引脚为wiringPi编码10口

/*小车初始速度控制*/
int CarSpeedControl = 150;

/*设置舵机驱动引脚*/
int ServoPin = 4;

/*小车运动状态和舵机运动状态标志*/
int g_CarState = enSTOP;  //1前2后3左4右0停止
int g_ServoState = 0;     //1左摇 2 右摇

/*RGBLED引脚设置*/
int LED_R = 3;           //LED_R接在Raspberry上的wiringPi编码3口
int LED_G = 2;           //LED_G接在Raspberry上的wiringPi编码2口
int LED_B = 5;           //LED_B接在Raspberry上的wiringPi编码5口

unsigned char PS2_KEY;
unsigned char X1,Y1,X2,Y2; 

//按键值  
unsigned short MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_TRIANGLE,
    PSB_CIRCLE,
    PSB_CROSS,
    PSB_SQUARE
	};	
	
unsigned short  Handkey;
unsigned char scan[9]={0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 

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
* @author        Danny
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
* Function       upleft
* @author        Danny
* @date          2017.08.16
* @brief         小车沿左前轮前进(左轮前进,右轮前进，两者有差速)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void upleft()
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);    //左电机前进使能
  digitalWrite(Left_motor_back, LOW);   //左电机后退禁止
  softPwmWrite(Left_motor_pwm, CarSpeedControl - 30);     //左边电机速度设为120(0-255)

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, CarSpeedControl + 30);   //右边电机速度设180(0-255)
}

/**
* Function       downleft
* @author        Danny
* @date          2017.08.16
* @brief         小车沿左后方后退(左轮后退,右轮后退，两者有差速)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void downleft()
{
  //左电机后退
  digitalWrite(Left_motor_go, LOW);         //左电机前进禁止
  digitalWrite(Left_motor_back, HIGH);      //左电机后退使能
  softPwmWrite(Left_motor_pwm, CarSpeedControl - 30);//左边电机速度设为control-15(0-255)

  //右电机后退
  digitalWrite(Right_motor_go, LOW);        //右电机前进禁止
  digitalWrite(Right_motor_back, HIGH);     //右电机后退使能
  softPwmWrite(Right_motor_pwm, CarSpeedControl + 30);//右边电机速度设control+50(0-255)
}

/**
* Function       upright
* @author        Danny
* @date          2017.08.16
* @brief         小车沿右上方前进 (左轮前进,右轮前进，两者有差速)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void upright()
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);    //左电机前进使能
  digitalWrite(Left_motor_back, LOW);   //左电机后退禁止
  softPwmWrite(Left_motor_pwm, CarSpeedControl + 30);     //左边电机速度设180(0-255)

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);   //右电机前进使能
  digitalWrite(Right_motor_back, LOW);  //右电机后退禁止
  softPwmWrite(Right_motor_pwm, CarSpeedControl - 30);    //右边电机速度设120(0-255)
}

/**
* Function       downright
* @author        Danny
* @date          2017.08.16
* @brief         小车沿右下方后退(左轮后退,右轮后退，两者有差速)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void downright()
{
  //左电机后退
  digitalWrite(Left_motor_go, LOW);     //左电机前进禁止
  digitalWrite(Left_motor_back, HIGH);  //左电机后退使能
  softPwmWrite(Left_motor_pwm, CarSpeedControl + 30);     //左边电机速度设180(0-255)

  //右电机后退
  digitalWrite(Right_motor_go, LOW);   //右电机前进禁止
  digitalWrite(Right_motor_back, HIGH);//右电机后退使能
  softPwmWrite(Right_motor_pwm, CarSpeedControl - 30);   //右边电机速度设120(0-255)
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
* Function       PS2_Init
* @author        Danny
* @date          2017.08.16
* @brief         PS2初始化
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void PS2_Init()
{
	//初始化一个SPI通道，0代表SPI通道0,500000代表的是SPI时钟速度
	wiringPiSPISetup(0,500000);
	pinMode(PS2_CMD_PIN, OUTPUT);
	pinMode(PS2_CLK_PIN, OUTPUT);
	pinMode(PS2_DAT_PIN, INPUT);
	pinMode(PS2_SEL_PIN, OUTPUT);
	
	digitalWrite(PS2_CLK_PIN,HIGH);
	digitalWrite(PS2_SEL_PIN,HIGH);
	digitalWrite(PS2_CMD_PIN,HIGH);
}

/**
* Function       PS2_AnologData
* @author        Danny
* @date          2017.08.16
* @brief         读取PS2摇杆的模拟值
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
//得到一个摇杆的模拟量范围0~255
unsigned char PS2_AnologData(unsigned char button)
{
	return Data[button];
}

/**
* Function       PS2_ClearData
* @author        Danny
* @date          2017.08.16
* @brief         清空接受PS2的数据
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void PS2_ClearData()
{
	memset(Data, 0 ,sizeof(Data)/sizeof(Data[0]));
	return;
}

/**
* Function       PS2_ReadData
* @author        Danny
* @date          2017.08.16
* @brief         读取PS2的数据
* @param[in]     command:
* @param[out]    void
* @retval        void
* @par History   无
*/
unsigned char PS2_ReadData(unsigned char command)
{
	unsigned char i,j = 1;
	unsigned char res = 0; 
    for(i=0; i<=7; i++)          
    {
		if(command&0x01)
		digitalWrite(PS2_CMD_PIN,HIGH);
		else
		digitalWrite(PS2_CMD_PIN,LOW);
		command = command >> 1;
		delayMicroseconds(10);
		digitalWrite(PS2_CLK_PIN,LOW);
		delayMicroseconds(10);
		if(digitalRead(PS2_DAT_PIN) == HIGH) 
			res = res + j;
		j = j << 1; 
		digitalWrite(PS2_CLK_PIN,HIGH);
		delayMicroseconds(10);	 
    }
    digitalWrite(PS2_CMD_PIN,HIGH);
	delayMicroseconds(50);
 //   printf("res:%d\n",res);	
    return res;	
}

/**
* Function       PS2_DataKey
* @author        Danny
* @date          2017.08.16
* @brief         PS2获取按键类型
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
//对读出来的 PS2 的数据进行处理
//按下为 0， 未按下为 1
unsigned char PS2_DataKey()
{
	unsigned char index = 0, i = 0;
   	PS2_ClearData();
	digitalWrite(PS2_SEL_PIN,LOW);
	for(i=0;i<9;i++)	//更新扫描按键
	{
		Data[i] = PS2_ReadData(scan[i]);
//		printf("data[%d]:%d\n",i,Data[i]);	
	} 
	digitalWrite(PS2_SEL_PIN,HIGH);
	
	//这是16个按键，按下为0，未按下为1
	Handkey=(Data[4]<<8)|Data[3];     
	for(index=0;index<16;index++)
	{	 
		if((Handkey&(1<<(MASK[index]-1)))==0)
		{
			return index+1;
		}
//		printf("index:%d\n",index+1);
	}
	return 0;      //没有任何按键按下
}

/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         对手柄发过来的指令解析，并执行相应的指令
* @param[in]     void
* @retval        void
* @par History   无
*/
void main()
{
	//wiringPi初始化
	wiringPiSetup();
	//初始化PS2
    PS2_Init();
	
    //初始化电机驱动IO为输出方式
    pinMode(Left_motor_go, OUTPUT);
    pinMode(Left_motor_back, OUTPUT);
    pinMode(Right_motor_go, OUTPUT);
    pinMode(Right_motor_back, OUTPUT);
  
    //创建两个软件控制的PWM脚用于电机速度控制
    softPwmCreate(Left_motor_pwm,0,255); 
    softPwmCreate(Right_motor_pwm,0,255);
  
    //初始化蜂鸣器IO为输出方式
    pinMode(buzzer, OUTPUT);
    digitalWrite(buzzer, HIGH);
 
    //初始化RGB三色LED的IO口为输出方式，并初始化
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);

	//创建三个软件控制的PWM脚用于LED控制
    softPwmCreate(LED_R,0,255); 
    softPwmCreate(LED_G,0,255); 
    softPwmCreate(LED_B,0,255);
  
    //初始化舵机引脚为输出模式
    pinMode(ServoPin, OUTPUT);
	
    while(1)
    {	   
      //状态标识
      int flag = 0;
      PS2_KEY = PS2_DataKey();	 //手柄按键捕获处理
	  switch(PS2_KEY)
	  {
		//select键按下
	    case PSB_SELECT: 	puts("PSB_SELECT");  break;
		//L3键按下，停车
	    case PSB_L3:     	g_CarState = enSTOP;  puts("PSB_L3");  break; 
        //R3键按下，舵机归位		
	    case PSB_R3:     	servo_appointed_detection(90);	puts("PSB_R3");  break; 
        //start键按下		
	    case PSB_START:  	puts("PSB_START");  break;  
		//UP键按下，小车前进
	    case PSB_PAD_UP: 	g_CarState = enRUN;   puts("PSB_PAD_UP");  break; 
        //RIGHT键按下，小车右转		
	    case PSB_PAD_RIGHT:	g_CarState = enRIGHT; puts("PSB_PAD_RIGHT");  break;
		//DOWN键按下，小车后退
	    case PSB_PAD_DOWN:	g_CarState = enBACK;  puts("PSB_PAD_DOWN");  break;
        //LEFT键按下，小车左转		
	    case PSB_PAD_LEFT:	g_CarState = enLEFT;  puts("PSB_PAD_LEFT");  break; 
		//L2按键按下，小车每次加速
	    case PSB_L2:      	CarSpeedControl += 50;
                            if (CarSpeedControl > 255)
                            {
                              CarSpeedControl = 255;
                            }
						    puts("PSB_L2");  break; 
		//R2按键按下，小车每次减速
	    case PSB_R2:      	CarSpeedControl -= 50;
                            if (CarSpeedControl < 50)
                            {
                              CarSpeedControl = 100;
                            }
						    puts("PSB_R2");  break; 
		//L1按键按下
	    case PSB_L1:      	puts("PSB_L1");  break; 
		//R1按键按下
	    case PSB_R1:      	puts("PSB_R1");  break; 
        //三角形按下，亮绿灯		
	    case PSB_TRIANGLE:	color_led_pwm(0, 255, 0);
                            delay(100);
						    puts("PSB_TRIANGLE");  break; 
		//圆形键按下时，亮蓝灯
	    case PSB_CIRCLE:  	color_led_pwm(0, 0, 255);
                            delay(100);
					        puts("PSB_CIRCLE");  break; 
	    
	    //方形键按下，亮红灯
	    case PSB_SQUARE:  	color_led_pwm(255, 0, 0);
                            delay(100);
						    puts("PSB_SQUARE");  break;

		//当×型按键按下时，蜂鸣器响
		case PSB_CROSS:		whistle();
							color_led_pwm(0, 0, 0);break;
							
	    default: g_CarState = enSTOP; break; 
	   }
      
	  //当L1或者R1按下时，读取摇杆数据的模拟值
	  if(PS2_KEY == PSB_L1 || PS2_KEY == PSB_R1)
	  {
		X1 = PS2_AnologData(PSS_LX);
		Y1 = PS2_AnologData(PSS_LY);
		X2 = PS2_AnologData(PSS_RX);
		Y2 = PS2_AnologData(PSS_RY);
		
        /*左侧摇杆控制小车的运动状态*/		
	    if (Y1 < 5 && X1 > 80 && X1 < 180) 
		{
		    g_CarState = enRUN;		//前进 
		}
		else if (Y1 > 230 && X1 > 80 && X1 < 180) 
	    {
			g_CarState = enBACK;	//后退			  
		}
		else if (X1 < 5 && Y1 > 80 && Y1 < 180) 
		{
			g_CarState = enLEFT;    //左转
		}
	    else if (Y1 > 80 && Y1 < 180 && X1 > 230)
		{
			g_CarState = enRIGHT;   //右转
	    }
	    else if (Y1 <= 80 && X1 <= 80) 
		{
		    g_CarState = enUPLEFT;  //左上
		}
		else if (Y1 <= 80 && X1 >= 180) 
		{
			g_CarState = enUPRIGHT;	//右上
		}
	    else if (X1 <= 80 && Y1 >= 180) 
		{
			g_CarState = enDOWNLEFT;//左下		
		}
	    else if (Y1 >= 180 && X1 >= 180) 
		{
			g_CarState = enDOWNRIGHT;//右下			  
		}
	    else
		{
			g_CarState = enSTOP;     //停车
	    }

		/*右摇杆控制舵机运动状态*/
	    if (X2 < 5 && Y2 > 110 && Y2 < 150) 
	    {
		    g_ServoState = 1;        //摇杆左侧，舵机向左转
		}
		else if (Y2 > 110 && Y2 < 150 && X2 > 230)
		{
		    g_ServoState = 2;        //摇杆右侧，舵机向右转
		}
		else
		{
		    g_ServoState = 0;        //舵机归位
		}
	  }
	
     //flag是用作为状态机	
     switch (g_ServoState)
     {
       case 0: if (flag != 0) {
               flag = 0;
               servo_appointed_detection(90);
               } break;
       case 1: if (flag != 1) {
               flag = 1;
               servo_appointed_detection(180);
               } break;
       case 2: if (flag != 2) {
               flag = 2;
               servo_appointed_detection(0);
               } break;
       default: break;
     }
	 
	 //小车运动状态判断
     switch (g_CarState)
     {
      case enSTOP: brake(); break;
      case enRUN: run(); break;
      case enLEFT: left(); break;
      case enRIGHT: right(); break;
      case enBACK: back(); break;
      case enTLEFT: spin_left(); break;
      case enTRIGHT: spin_right(); break;
	  case enUPLEFT: upleft();break;
	  case enUPRIGHT: upright();break;
	  case enDOWNLEFT:downleft();break;
	  case enDOWNRIGHT:downright();break;
      default: brake(); break;
     }
	 
     //下面的延时是必须要的,主要是为了避免过于频繁的发送手柄指令造成的不断重启
     delay(50);	
   }
} 
