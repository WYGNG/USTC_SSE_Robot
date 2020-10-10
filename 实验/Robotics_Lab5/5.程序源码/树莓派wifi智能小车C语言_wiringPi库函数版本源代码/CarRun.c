/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         CarRun.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief       小车前进后退左右综合实验
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
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   无
*/
void run(int time)
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);   //左电机前进使能
  digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
  softPwmWrite(Left_motor_pwm, 150);   //PWM比例0-255调速，左右轮差异略增减

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, 150);  //PWM比例0-255调速，左右轮差异略增减

  //延时
  delay(time * 100);
}

/**
* Function       brake
* @author        Danny
* @date          2017.08.16
* @brief         小车刹车
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   无
*/
void brake(int time)
{
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Left_motor_back, LOW);
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, LOW);

  //延时
  delay(time * 100);
}

/**
* Function       left
* @author        Danny
* @date          2017.08.16
* @brief         小车左转 左转(左轮不动,右轮前进)
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   无
*/
void left(int time)
{
  //左电机停止
  digitalWrite(Left_motor_go, LOW);     //左电机前进禁止
  digitalWrite(Left_motor_back, LOW);   //左电机后退禁止
  softPwmWrite(Left_motor_pwm, 0);      //左边电机速度设为0(0-255)

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, 150);  //右边电机速度设200(0-255)

  //延时
  delay(time * 100);
}

/**
* Function       right
* @author        Danny
* @date          2017.08.16
* @brief         小车右转 右转(左轮前进,右轮不动)
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   无
*/
void right(int time)
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);    //左电机前进使能
  digitalWrite(Left_motor_back, LOW);   //左电机后退禁止
  softPwmWrite(Left_motor_pwm, 150);    //左边电机速度设200(0-255)

  //右电机停止
  digitalWrite(Right_motor_go, LOW);    //右电机前进禁止
  digitalWrite(Right_motor_back, LOW);  //右电机后退禁止
  softPwmWrite(Right_motor_pwm, 0);     //右边电机速度设0(0-255)

  //延时
  delay(time * 100);
}

/**
* Function       spin_left
* @author        Danny
* @date          2017.08.16
* @brief         小车原地左转 原地左转(左轮后退，右轮前进)
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   无
*/
void spin_left(int time)
{
  //左电机后退
  digitalWrite(Left_motor_go, LOW);    //左电机前进禁止
  digitalWrite(Left_motor_back, HIGH); //左电机后退使能
  softPwmWrite(Left_motor_pwm, 150);

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, 150);

  //延时
  delay(time * 100);
}

/**
* Function       spin_right
* @author        Danny
* @date          2017.08.16
* @brief         小车原地右转 原地右转(右轮后退，左轮前进)
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   无
*/
void spin_right(int time)
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);    //左电机前进使能
  digitalWrite(Left_motor_back, LOW);   //左电机后退禁止
  softPwmWrite(Left_motor_pwm, 150);

  //右电机后退
  digitalWrite(Right_motor_go, LOW);    //右电机前进禁止
  digitalWrite(Right_motor_back, HIGH); //右电机后退使能
  softPwmWrite(Right_motor_pwm, 150);

  //延时
  delay(time * 100);
}

/**
* Function       back
* @author        Danny
* @date          2017.08.16
* @brief         小车后退 
* @param[in]     time
* @param[out]    void
* @retval        void
* @par History   无
*/
void back(int time)
{
  //左电机后退
  digitalWrite(Left_motor_go, LOW);     //左电机前进禁止
  digitalWrite(Left_motor_back, HIGH);  //左电机后退使能
  softPwmWrite(Left_motor_pwm, 150);

  //右电机后退
  digitalWrite(Right_motor_go, LOW);    //右电机前进禁止
  digitalWrite(Right_motor_back, HIGH); //右电机后退使能
  softPwmWrite(Right_motor_pwm, 150);

  //延时
  delay(time * 100);
}

/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         先延时2，再前进1，后退1s,左转2s,右转2s,
*                原地左转3s,原地右转3s,停止0.5s
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
  
  while(1)
  {
   delay(2000);      //延时2s  
   run(10);          //前进1s(10 * 100ms)
   back(10);         //后退1s
   left(20);         //左转2s
   right(20);        //右转2s
   spin_left(30);    //原地左转3s
   spin_right(30);   //原地右转3s
   brake(5);         //停止0.5s
  }
  return;
}
