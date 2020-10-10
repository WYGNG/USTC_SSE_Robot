/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         ServoControlColor.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        舵机旋转控制七彩探照灯
* @details
* @par History  见如下说明
*
*/
#include <wiringPi.h>
#include <softPwm.h>

#define ON  1   //使能LED
#define OFF 0   //禁止LED

//定义引脚
int LED_R = 3;  //LED_R接在Raspberry上的wiringPi编码3口
int LED_G = 2;  //LED_G接在Raspberry上的wiringPi编码2口
int LED_B = 5;  //LED_B接在Raspberry上的wiringPi编码5口

//定义舵机引脚
int ServoPin = 4;

//函数声明
void corlor_light(int);
void corlor_led(int, int, int);

/**
* Function       servo_pulse
* @author        Danny
* @date          2017.08.16
* @brief         定义一个脉冲函数，用来模拟方式产生PWM值
*                时基脉冲为20ms,该脉冲高电平部分在0.5-2.5ms
*                控制0-180度
* @param[in1]    myangle:舵机转动指定的角度
* @param[out]    void
* @retval        void
* @par History   无
*/
void servo_pulse(int myangle)
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
* Function       servo_control_color
* @author        Danny
* @date          2017.08.16
* @brief         舵机从0-180的转动,然后从180到0转，
*                同时180度角分为7个区间来表示7种不同的颜色
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void servo_control_color()
{
  //定义舵机转动位置
  int pos = 0;
  for (pos = 0; pos < 180; pos++)
  {
    //舵机0旋转到180,每次延时20ms
    servo_pulse(pos);
    //旋转到相应的角度,判断调用相关函数
    corlor_light(pos);
    delay(20);
  }

  for (pos = 180; pos > 0; pos--)
  {
    //舵机180旋转到0,每次延时20ms
    servo_pulse(pos);
    //旋转到相应的角度,判断调用相关函数
    corlor_light(pos);
    delay(20);
  }
  return;
}

/**
* Function       corlor_light
* @author        Danny
* @date          2017.08.16
* @brief         根据转动的角度来点亮相应的颜色
* @param[in]     pos 舵机的转动位置
* @param[out]    void
* @retval        void
* @par History   无
*/
void corlor_light(int pos)
{
  //当转动在150-180度时,点亮一种颜色
  if (pos > 150)
  {
    corlor_led(ON, OFF, OFF);
  }
  //当转动在125-150度时,点亮一种颜色
  else if (pos > 125)
  {
    corlor_led(OFF, ON, OFF);
  }
  //当转动在100-125度时,点亮一种颜色
  else if (pos > 100)
  {
    corlor_led(OFF, OFF, ON);
  }
  //当转动在75-100度时,点亮一种颜色
  else if (pos > 75)
  {
    corlor_led(OFF, ON, ON);
  }
  //当转动在50-75度时,点亮一种颜色
  else if (pos > 50)
  {
    corlor_led(ON, ON, OFF);
  }
  //当转动在25-50度时,点亮一种颜色
  else if (pos > 25)
  {
    corlor_led(ON, OFF, ON);
  }
  //当转动在0-25度时,点亮一种颜色
  else if (pos > 0)
  {
    corlor_led(ON, ON, ON);
  }
  else
  {
    corlor_led(OFF, OFF, OFF);
  }
}

/**
* Function        corlor_led
* @author         Danny
* @date           2017.08.16
* @brief          由R,G,B三色的不同组合形成7种不同的色彩
* @param[in1]     Red开关
* @param[in2]     Green开关
* @param[in3]     Blue开关
* @retval         void
* @par History    无
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
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         先延时0.5，接着调用舵机控制七彩灯程序
* @param[in]     void
* @retval        void
* @par History   无
*/
void main()
{
  //wiringPi初始化
  wiringPiSetup();
  
  //舵机设置为输出模式
  pinMode(ServoPin, OUTPUT);
  
  //初始化RGB三色LED的IO口为输出方式
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  
  //初始化舵机位置向前
  int ServoPos = 90;
  servo_pulse(ServoPos);
  
  while(1)
  {
   //延时0.5s
   delay(500);
   //调用舵机控制七彩灯程序
   servo_control_color();
  }
  return;
}


