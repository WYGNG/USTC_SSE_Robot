/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         keysacnStart.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        按键控制小车的启动
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

int key = 10;                 //定义按键为Raspberry的wiringPi编码10口

/**
* Function       run
* @author        Danny
* @date          2017.08.16
* @brief         小车前进
* @param[in]     time：延时时间
* @param[out]    void
* @retval        void
* @par History   无
*/
void run(int time)
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);   //左电机前进使能
  digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
  softPwmWrite(Left_motor_pwm, 150);

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, 150);

  //延时
  delay(time * 100);
}

/**
* Function       brake
* @author        Danny
* @date          2017.08.16
* @brief         小车刹车
* @param[in]     time：延时时间
* @param[out]    void
* @retval        void
* @par History   无
*/
//刹车
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
* @brief         小车左转(左轮不动，右轮前进)
* @param[in]     time：延时时间
* @param[out]    void
* @retval        void
* @par History   无
*/
void left(int time)
{
  //左电机停止
  digitalWrite(Left_motor_go, LOW);    //左电机前进禁止
  digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
  softPwmWrite(Left_motor_pwm, 0);

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, 150);

  //延时
  delay(time * 100);
}

/**
* Function       right
* @author        Danny
* @date          2017.08.16
* @brief         小车右转(右轮不动，左轮前进)
* @param[in]     time：延时时间
* @param[out]    void
* @retval        void
* @par History   无
*/
void right(int time)
{
  //左电机前进
  digitalWrite(Left_motor_go, HIGH);   //左电机前进使能
  digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
  softPwmWrite(Left_motor_pwm, 150);

  //右电机停止
  digitalWrite(Right_motor_go, LOW);   //右电机前进禁止
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, 0);

  delay(time * 100);
}

/**
* Function       spin_left
* @author        Danny
* @date          2017.08.16
* @brief         小车原地左转(左轮后退，右轮前进)
* @param[in]     time：延时时间
* @param[out]    void
* @retval        void
* @par History   无
*/
void spin_left(int time)
{
  //左电机后退
  digitalWrite(Left_motor_go, LOW);     //左电机前进禁止
  digitalWrite(Left_motor_back, HIGH);  //左电机后退使能
  softPwmWrite(Left_motor_pwm, 150);

  //右电机前进
  digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
  digitalWrite(Right_motor_back, LOW); //右电机后退禁止
  softPwmWrite(Right_motor_pwm, 150);

  delay(time * 100);
}

/**
* Function       spin_right
* @author        Danny
* @date          2017.08.16
* @brief         小车原地右转(右轮后退，左轮前进)
* @param[in]     time：延时时间
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

  delay(time * 100);
}

/**
* Function       back
* @author        Danny
* @date          2017.08.16
* @brief         小车后退
* @param[in]     time：延时时间
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
}

/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         先调用按键扫描函数，再前进1，后退1s,左转2s,右转2s,
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
  softPwmCreate(Left_motor_pwm,0,255); 
  softPwmCreate(Right_motor_pwm,0,255);

  //定义按键接口为输入接口
  pinMode(key, INPUT);
 
  key_scan();       //调用按键扫描函数
  
  while(1)
  {
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
