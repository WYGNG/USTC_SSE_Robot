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
	
//======一下三行是对加速度进行量化，得出单位为g的加速度值
   Ax=ax/16384.00;
   Ay=ay/16384.00;
   Az=az/16384.00;
   //==========以下三行是用加速度计算三个轴和水平面坐标系之间的夹角
//请参考：[url]http://www.geek-workshop.com/forum.php?mod=viewthread&tid=2328&page=1#pid27876[/url]
//个人觉得原帖中case0算的不对，应该是z/sqrt（x*x+y*y），估计是江南楼主写错了
   Angel_accX=atan(Ax/sqrt(Az*Az+Ay*Ay))*180/3.14;
   Angel_accY=atan(Ay/sqrt(Ax*Ax+Az*Az))*180/3.14;
   Angel_accZ=atan(Az/sqrt(Ax*Ax+Ay*Ay))*180/3.14;
   //==========以下三行是对角速度做量化==========
   ggx=gx/131.00;
   ggy=gy/131.00;
   ggz=gz/131.00;
 
  //===============以下是对角度进行积分处理================
  NowTime=RunningTime;//获取当前程序运行的毫秒数
  TimeSpan=NowTime-LastTime;//积分时间这样算不是很严谨
//下面三行就是通过对角速度积分实现各个轴的角度测量，当然假设各轴的起始角度都是0
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
