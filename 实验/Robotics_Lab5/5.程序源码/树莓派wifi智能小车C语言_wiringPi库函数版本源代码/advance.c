/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         advance.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        小车前进实验
* @details
* @par History  见如下说明
*
*/
#include <wiringPi.h>
#include <softPwm.h>

//定义引脚
int Left_motor_go = 28;       //左电机前进AIN2连接Raspberry的wiringPi编码28口
int Left_motor_back = 29;     //左电机后退AIN1连接Raspberry的wiringPi编码29口

int Right_motor_go = 24;      //右电机前进BIN2连接Raspberry的wiringPi编码24口
int Right_motor_back = 25;    //右电机后退BIN1连接Raspberry的wiringPi编码25口

int Left_motor_pwm = 27;      //左电机控速PWMA连接Raspberry的wiringPi编码27口
int Right_motor_pwm = 23;     //右电机控速PWMB连接Raspberry的wiringPi编码23口

/**
* Function       run
* @author        Danny
* @date          2017.08.16
* @brief         小车前进
* @param[in]     time延时时间
* @param[out]    void
* @retval        void
* @par History   无
*/
void run(int time)
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);    //左电机前进使能
  digitalWrite(Left_motor_back, LOW);   //左电机后退禁止
  //更新指定管脚的PWM值
  softPwmWrite(Left_motor_pwm, 150);    //pwm控速0-255之，左右轮略有差异

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);   //右电机前进使能
  digitalWrite(Right_motor_back, LOW);  //右电机后退禁止
  //更新指定管脚的PWM值
  softPwmWrite(Right_motor_pwm, 150);   //pwm控速0-255之，左右轮略有差异

  //延时
  delay(time * 100); //delay函数默认是以ms作为单位的
  return;
}

/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         先延时2s，接着前进
* @param[in]     void
* @retval        void
* @par History   无
*/
void main()
{
  //wiringPi初始化
  wiringPiSetup();
  
  //初始化电机驱动IO口为输出方式
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT);
  pinMode(Right_motor_go, OUTPUT);
  pinMode(Right_motor_back, OUTPUT);
  
  //创建两个软件控制的PWM脚
  //int softPwmCreate(int pin,int initialValue,int pwmRange);
  softPwmCreate(Left_motor_pwm,0,255); 
  softPwmCreate(Right_motor_pwm,0,255);   
  
  delay(2000);     //延时2s
  
  while(1)
  {
   run(10);        //前进
  }
  return;
}
