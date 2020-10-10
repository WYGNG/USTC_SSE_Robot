/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         servo_ultrasonic_avoid.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        带舵机云台超声波避障
* @details
* @par History  见如下说明
*
*/
#include <wiringPi.h>
#include <softPwm.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>

#define ON  1       //使能LED
#define OFF 0       //禁止LED

//定义引脚
int Left_motor_go = 28;   //左电机前进AIN2连接Raspberry的wiringPi编码28口
int Left_motor_back = 29; //左电机后退AIN1连接Raspberry的wiringPi编码29口

int Right_motor_go = 24;  //右电机前进BIN2连接Raspberry的wiringPi编码24口
int Right_motor_back = 25;//右电机后退BIN1连接Raspberry的wiringPi编码25口

int Left_motor_pwm = 27;  //左电机控速PWMA连接Raspberry的wiringPi编码27口
int Right_motor_pwm = 23; //右电机控速PWMB连接Raspberry的wiringPi编码23口

//按键
int key = 10;             //定义按键为Raspberry的wiringPi编码10口

//超声波
int EchoPin = 30;         //定义回声脚为连接Raspberry的wiringPi编码30口
int TrigPin = 31;         //定义触发脚为连接Raspberry的wiringPi编码31口

//定义引脚
int LED_R = 3;           //LED_R接在Raspberry上的wiringPi编码3口
int LED_G = 2;           //LED_G接在Raspberry上的wiringPi编码2口
int LED_B = 5;           //LED_B接在Raspberry上的wiringPi编码5口

//定义舵机引脚
int ServoPin = 4;

//初始化舵机位置向前
int ServoPos = 90;

const int AvoidSensorLeft =  26; //定义左边避障的红外传感器引脚为wiringPi编码26口
const int AvoidSensorRight = 0;  //定义右边避障的红外传感器引脚为wiringPi编码0口

int LeftSensorValue ;            //定义变量来保存红外传感器采集的数据大小
int RightSensorValue ;

//函数声明
void brake();
void spin_left(int);
void spin_right(int);
void back(int);
float Distance_test();
void bubble(unsigned long *, int);
void corlor_led(int, int, int);

/**
* Function       servo_pulse
* @author        Danny
* @date          2017.08.16
* @brief         定义一个脉冲函数，用来模拟方式产生PWM值
*                时基脉冲为20ms,该脉冲高电平部分在0.5-2.5ms
*                控制0-180度
* @param[in1]    ServoPin:舵机引脚
* @param[in2]    myangle:舵机转动指定的角度
* @param[out]    void
* @retval        void
* @par History   无
*/
void servo_pulse(int ServoPin, int myangle)
{
  int PulseWidth;                    //定义脉宽变量
  PulseWidth = (myangle * 11) + 0; //将角度转化为500-2480 的脉宽值
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
  for (i = 0; i <= 15; i++)    //产生PWM个数，等效延时以保证能转到响应角度
  {
    servo_pulse(ServoPin, pos);//模拟产生PWM
  }
}

/**
* Function       servo_color_carstate
* @author        Danny
* @date          2017.08.16
* @brief         舵机转向超声波测距避障行驶,led根据车的状态显示相应的颜色
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void servo_color_carstate()
{
  float distance;
  //定义舵机位置变量和小车前方,左侧,右侧距离
  int iServoPos = 0;
  int LeftDistance = 0;    //左方距离值变量LeftDistance
  int RightDistance = 0;   //右方距离值变量RightDistance
  int FrontDistance = 0;   //前方距离值变量FrontDistance
  corlor_led(ON, OFF, OFF);//开红灯
  back(80);                //避免突然停止,刹不住车
  brake();

  //舵机旋转到0度,即右侧,测距
  servo_appointed_detection(0);
  delay(500);
  distance = Distance_test();  //测距
  RightDistance = distance;    //所测的右侧距离赋给变量RightDistance
  //printf("rightdistance :%d\n",RightDistance);
 
  //舵机旋转到180度,即左侧,测距
  servo_appointed_detection(180);
  delay(500);
  distance = Distance_test();  //测距
  LeftDistance = distance;//所测的左侧距离赋给变量LeftDistance
  // printf("leftdistance :%d\n",LeftDistance);

  //舵机旋转到90度,即左侧,测距
  servo_appointed_detection(90);
  delay(500);
  distance = Distance_test();
  FrontDistance = distance;//所测前侧距离付给变量FrontDistance
  //printf("FrontDistance:%d\n",FrontDistance);
 
  if (LeftDistance < 30 && RightDistance < 30 && FrontDistance < 30  )
  {
    //亮品红色,掉头
    corlor_led(ON, OFF, ON);
    spin_left(700);
  }
  else if ( LeftDistance >= RightDistance) //当发现左侧距离大于右侧，原地左转
  {
    //亮蓝色
    corlor_led(OFF, OFF, ON);
    spin_left(350);
  }
  else if (LeftDistance < RightDistance ) //当发现右侧距离大于左侧，原地右转
  {
    //亮品红色,向右转
    corlor_led(ON, OFF, ON);
    spin_right(350);
  }
}

/**
* Function       run
* @author        Danny
* @date          2017.08.16
* @brief         小车前进
* @param[in1]    LeftSpeed:左侧速度
* @param[in2]    RightSpeed:右侧速度
* @param[out]    void
* @retval        void
* @par History   无
*/
void run(int LeftSpeed, int RightSpeed)
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);   //左电机前进使能
  digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
  softPwmWrite(Left_motor_pwm, LeftSpeed);

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, RightSpeed);
}

/**
* Function       brake
* @author        Danny
* @date          2017.08.16
* @brief         小车刹车
* @param[in]
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
* @brief         小车左转(左轮不动，右轮前进)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void left()
{
  //左电机停止
  digitalWrite(Left_motor_go, LOW);    //左电机前进禁止
  digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
  softPwmWrite(Left_motor_pwm, 0);

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, 80);
}

/**
* Function       right
* @author        Danny
* @date          2017.08.16
* @brief         小车右转(右轮不动，左轮前进)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void right()
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);   //左电机前进使能
  digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
  softPwmWrite(Left_motor_pwm, 80);

  //右电机停止
  digitalWrite(Right_motor_go, LOW);   //右电机前进禁止
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, 0);
}

/**
* Function       spin_left
* @author        Danny
* @date          2017.08.16
* @brief         小车原地左转(左轮后退，右轮前进)
* @param[in]     time:延时时间
* @param[out]    void
* @retval        void
* @par History   无
*/
void spin_left(int time)
{
  //左电机后退
  digitalWrite(Left_motor_go, LOW);     //左电机前进禁止
  digitalWrite(Left_motor_back, HIGH);  //左电机后退使能
  softPwmWrite(Left_motor_pwm, 200);

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, 200);

  delay(time);
}

/**
* Function       spin_right
* @author        Danny
* @date          2017.08.16
* @brief         小车原地右转(右轮后退，左轮前进)
* @param[in]     time:延时时间
* @param[out]    void
* @retval        void
* @par History   无
*/
void spin_right(int time)
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);   //左电机前进使能
  digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
  softPwmWrite(Left_motor_pwm, 200);

  //右电机后退
  digitalWrite(Right_motor_go, LOW);    //右电机前进禁止
  digitalWrite(Right_motor_back, HIGH); //右电机后退使能
  softPwmWrite(Right_motor_pwm, 200);

  delay(time);
}

/**
* Function       back
* @author        Danny
* @date          2017.08.16
* @brief         小车后退
* @param[in]     time:延时时间
* @param[out]    void
* @retval        void
* @par History   无
*/
void back(int time)
{
  //左电机后退
  digitalWrite(Left_motor_go, LOW);     //左电机前进禁止
  digitalWrite(Left_motor_back, HIGH);  //左电机后退使能
  softPwmWrite(Left_motor_pwm, 80);

  //右电机后退
  digitalWrite(Right_motor_go, LOW);    //右电机前进禁止
  digitalWrite(Right_motor_back, HIGH); //右电机后退使能
  softPwmWrite(Right_motor_pwm, 80);

  delay(time);
}

/**
* Function       corlor_led
* @author        Danny
* @date          2017.08.16
* @brief         由R,G,B三色的不同组合形成7种不同的色彩
* @param[in1]    v_iRed:Red开关
* @param[in2]    v_iGreen:Green开关
* @param[in3]    v_iBlue:Blue开关
* @retval        void
* @par History   无
*/
void corlor_led(int v_iRed, int v_iGreen, int v_iBlue)
{
  //红色LED
  v_iRed == ON ? digitalWrite(LED_R, HIGH): digitalWrite(LED_R, LOW);
 
  //绿色LED
  v_iGreen == ON ? digitalWrite(LED_G, HIGH) : digitalWrite(LED_G, LOW);
  
  //蓝色LED
  v_iBlue == ON ? digitalWrite(LED_B, HIGH) : digitalWrite(LED_B, LOW);
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
	return distance;
}

/**
* Function       Distane_test
* @author        Danny
* @date          2017.08.16
* @brief         超声波测五次，去掉最大值,最小值,
*                取平均值,提高测试准确性
* @param[in]     void
* @param[out]    void
* @retval        float:distance返回距离值
* @par History   无
*/
float Distance_test()
{
  float distance;
  unsigned long ultrasonic[5] = {0};
  int num = 0;
  while (num < 5)
  {
     distance = Distance();
	 //超时返回-1，重新测试
	 while((int)distance == -1)
	 {
		 distance = Distance();
	 }
    //过滤掉测试距离中出现的错误数据大于500
    while ( distance >= 500 || (int)distance == 0)
    {
         distance = Distance();
    }
    ultrasonic[num] = distance;
    num++;
	delay(10);
  }
  num = 0;
  bubble(ultrasonic, 5);
  distance = (ultrasonic[1] + ultrasonic[2] + ultrasonic[3]) / 3;
  
  printf("distance:%f\n",distance);      //打印测试的距离
  return distance;
}

/**
* Function       bubble
* @author        Danny
* @date          2017.08.16
* @brief         超声波测五次的数据进行冒泡排序
* @param[in1]    a:超声波数组首地址
* @param[in2]    n:超声波数组大小
* @param[out]    void
* @retval        void
* @par History   无
*/
void bubble(unsigned long *a, int n)

{
  int i, j, temp;
  for (i = 0; i < n - 1; i++)
  {
    for (j = i + 1; j < n; j++)
    {
      if (a[i] > a[j])
      {
        temp = a[i];
        a[i] = a[j];
        a[j] = temp;
      }
    }
  }
}

/**
* Function       key_scan
* @author        Danny
* @date          2017.08.16
* @brief         按键检测(包含软件按键去抖)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void key_scan()
{
  while (digitalRead(key));       //当按键没有被按下一直循环
  while (!digitalRead(key))       //当按键被按下时
  {
    delay(10);	                  //延时10ms
    if (digitalRead(key)  ==  LOW)//第二次判断按键是否被按下
    {
      delay(100);
      while (!digitalRead(key));  //判断按键是否被松开
    }
  }
  return;
}

/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         先调用按键扫描函数，
*                带舵机云台的超声波避障模式开启,
*                红外避障模块辅助避障(超声波避障有盲区)
* @param[in]     void
* @retval        void
* @par History   无
*/
void main()
{
  float distance;
  
  //wiringPi初始化
  wiringPiSetup();	

  //初始化电机驱动IO口为输出方式
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT);
  pinMode(Right_motor_go, OUTPUT);
  pinMode(Right_motor_back, OUTPUT);
  
  //创建两个软件控制的PWM脚
  softPwmCreate(Left_motor_pwm,0,255); 
  softPwmCreate(Right_motor_pwm,0,255);

  //初始化RGB三色LED的IO口为输出方式
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  //定义按键接口为输出接口
  pinMode(key, INPUT);

  //初始化超声波引脚
  pinMode(EchoPin, INPUT);    //定义超声波输入脚
  pinMode(TrigPin, OUTPUT);   //定义超声波输出脚

  //舵机初始化为向前
  servo_appointed_detection(ServoPos);
  //舵机初始化为输出模式
  pinMode(ServoPin, OUTPUT);

  //定义左右传感器为输入接口
  pinMode(AvoidSensorLeft, INPUT);
  pinMode(AvoidSensorRight, INPUT);

  //调用按键扫描函数
  key_scan();
   
  while(1)
  {
    //遇到障碍物,红外避障模块的指示灯亮,端口电平为LOW
    //未遇到障碍物,红外避障模块的指示灯灭,端口电平为HIGH
    LeftSensorValue  = digitalRead(AvoidSensorLeft);
    RightSensorValue = digitalRead(AvoidSensorRight);

    if (LeftSensorValue == HIGH && RightSensorValue == HIGH)
    {
      run(80,80);        //当两侧均未检测到障碍物时调用前进函数
    }
    else if (LeftSensorValue == HIGH && RightSensorValue == LOW)
    {
      spin_left(40); //右边探测到有障碍物，有信号返回，原地向左转
    }
    else if (RightSensorValue == HIGH && LeftSensorValue == LOW)
    {
      spin_right(40);//左边探测到有障碍物，有信号返回，原地向右转
    }
    else if(RightSensorValue == LOW && LeftSensorValue == LOW)
    {
      servo_color_carstate();//调用超声波避障函数
    }
  }
	return;
}



