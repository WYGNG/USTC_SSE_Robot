/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         ColorLED.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        七彩探照灯实验
* @details
* @par History  见如下说明
*/
#include <wiringPi.h>

#define ON 1     //使能LED
#define OFF 0    //禁止LED

//定义引脚
int LED_R = 3;  //LED_R接在Raspberry上的wiringPi编码3口
int LED_G = 2;  //LED_G接在Raspberry上的wiringPi编码2口
int LED_B = 5;  //LED_B接在Raspberry上的wiringPi编码5口

/**
* Function       color_led
* @author        Danny
* @date          2017.08.16
* @brief         由R,G,B三色的不同组合形成7种不同的色彩
* @param[in1]    Red开关
* @param[in2]    Green开关
* @param[in3]    Blue开关
* @retval        void
* @par History   无
*/
void color_led(int v_iRed, int v_iGreen, int v_iBlue)
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
* @brief         循环显7色LED
* @param[in]     void
* @retval        void
* @par History   无
*/
int main()
{
	wiringPiSetup();
	
	//RGB引脚模式设置为输出模式
	pinMode(LED_R, OUTPUT);
	pinMode(LED_G, OUTPUT);
	pinMode(LED_B, OUTPUT);
	
	while (1)
	{                        //   LED_R   LED_G    LED_B
    color_led(ON, OFF, OFF); //   1        0        0
    delay(1000);
    color_led(OFF, ON, OFF); //   0        1        0
    delay(1000);
    color_led(OFF, OFF, ON); //   0        0        1
    delay(1000);
    color_led(ON, ON, OFF);  //   1        1        0
    delay(1000);
    color_led(ON, OFF, ON);  //   1        0        1
    delay(1000);
    color_led(OFF, ON, ON);  //   0        1        1
    delay(1000);
    color_led(ON, ON, ON);   //   1        1        1	
	}
   return 0;	
}