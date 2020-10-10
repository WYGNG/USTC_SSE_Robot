/******************** **************************
 * �ļ���  ��main.c
 * ����    ��I2C ���ԣ�������Ϣͨ��USART1��ӡ�ڵ��Ե��նˡ�
 *          

**********************************************************************************/	
#include "stm32f10x.h"
#include "I2C_MPU6050.h"
#include "usart1.h"
#include "delay.h"



/*
 * ��������main
 * ����  ��������
 * ����  ����
 * ���  ����
 * ����  ����
 */
int main(void)
{	


	/* ����1��ʼ�� */
	USART1_Config();	 
	/*IIC�ӿڳ�ʼ��*/
	I2C_MPU6050_Init(); 	 
	/*�����Ǵ�������ʼ��*/
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

		
		printf("\r\n---------���ٶ�X��ԭʼ����---------%d \r\n",GetData(ACCEL_XOUT_H));
		printf("\r\n---------���ٶ�Y��ԭʼ����---------%d \r\n",GetData(ACCEL_YOUT_H));	
		printf("\r\n---------���ٶ�Z��ԭʼ����---------%d \r\n",GetData(ACCEL_ZOUT_H));	
		printf("\r\n---------������X��ԭʼ����---------%d \r\n",GetData(GYRO_XOUT_H));	
		printf("\r\n---------������Y��ԭʼ����---------%d \r\n",GetData(GYRO_YOUT_H));	
		printf("\r\n---------������Z��ԭʼ����---------%d \r\n",temp);
		delay_ms(100);
	}		

 
}

/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
