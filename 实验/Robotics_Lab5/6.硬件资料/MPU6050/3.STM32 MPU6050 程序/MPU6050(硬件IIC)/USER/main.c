/******************** **************************
 * 文件名  ：main.c
 * 描述    ：I2C 测试，测试信息通过USART1打印在电脑的终端。
 *          

**********************************************************************************/	
#include "stm32f10x.h"
#include "I2C_MPU6050.h"
#include "usart1.h"
#include "delay.h"



/*
 * 函数名：main
 * 描述  ：主函数
 * 输入  ：无
 * 输出  ：无
 * 返回  ：无
 */
int main(void)
{	


	/* 串口1初始化 */
	USART1_Config();	 
	/*IIC接口初始化*/
	I2C_MPU6050_Init(); 	 
	/*陀螺仪传感器初始化*/
  InitMPU6050();
	
	/***********************************************************************/
	
	while(1)
	{
		static int temp;
	temp=  GetData(GYRO_ZOUT_H);
	    if(temp>32768)
		{
		  temp-=65536;
		}

		
		printf("\r\n---------加速度X轴原始数据---------%d \r\n",GetData(ACCEL_XOUT_H));
		printf("\r\n---------加速度Y轴原始数据---------%d \r\n",GetData(ACCEL_YOUT_H));	
		printf("\r\n---------加速度Z轴原始数据---------%d \r\n",GetData(ACCEL_ZOUT_H));	
		printf("\r\n---------陀螺仪X轴原始数据---------%d \r\n",GetData(GYRO_XOUT_H));	
		printf("\r\n---------陀螺仪Y轴原始数据---------%d \r\n",GetData(GYRO_YOUT_H));	
		printf("\r\n---------陀螺仪Z轴原始数据---------%d \r\n",temp);
		delay_ms(100);
	}		

 
}

/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
