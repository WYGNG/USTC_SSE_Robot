#include "stm32f10x.h"
#include "Calculate.h"
#include "I2C_MPU6050.h"
#include "usart1.h"
#include "math.h"


void loop()
{

	ax=GetData(ACCEL_XOUT_H);
	ay=GetData(ACCEL_YOUT_H);
	ax=GetData(ACCEL_ZOUT_H);
	gx=GetData(GYRO_XOUT_H);
	gy=GetData(GYRO_YOUT_H);
	gz=GetData(GYRO_ZOUT_H);
	
//======һ�������ǶԼ��ٶȽ����������ó���λΪg�ļ��ٶ�ֵ
   Ax=ax/16384.00;
   Ay=ay/16384.00;
   Az=az/16384.00;
   //==========�����������ü��ٶȼ����������ˮƽ������ϵ֮��ļн�
//��ο���[url]http://www.geek-workshop.com/forum.php?mod=viewthread&tid=2328&page=1#pid27876[/url]
//���˾���ԭ����case0��Ĳ��ԣ�Ӧ����z/sqrt��x*x+y*y���������ǽ���¥��д����
   Angel_accX=atan(Ax/sqrt(Az*Az+Ay*Ay))*180/3.14;
   Angel_accY=atan(Ay/sqrt(Ax*Ax+Az*Az))*180/3.14;
   Angel_accZ=atan(Az/sqrt(Ax*Ax+Ay*Ay))*180/3.14;
   //==========���������ǶԽ��ٶ�������==========
   ggx=gx/131.00;
   ggy=gy/131.00;
   ggz=gz/131.00;
 
  //===============�����ǶԽǶȽ��л��ִ���================
  NowTime=RunningTime;//��ȡ��ǰ�������еĺ�����
  TimeSpan=NowTime-LastTime;//����ʱ�������㲻�Ǻ��Ͻ�
//�������о���ͨ���Խ��ٶȻ���ʵ�ָ�����ĽǶȲ�������Ȼ����������ʼ�Ƕȶ���0
  Gx=Gx+(ggx-Gx_offset)*TimeSpan/1000;
  Gy=Gy+(ggy-Gy_offset)*TimeSpan/1000;
  Gz=Gz+(ggz-Gz_offset)*TimeSpan/1000;
 
  LastTime=NowTime;
 
  //==============================
	printf("\r\n*************%f \r\n",Angel_accX );
	printf("\r\n*************%f \r\n",Angel_accY );
	printf("\r\n*************%f \r\n",Angel_accZ );
	printf("\r\n*************%f \r\n",Gx );
	printf("\r\n*************%f \r\n",Gy );
	printf("\r\n*************%f \r\n",Gz );
 
}
