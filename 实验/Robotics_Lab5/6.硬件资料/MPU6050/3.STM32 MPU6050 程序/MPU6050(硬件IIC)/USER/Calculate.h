#ifndef __CALCULATE_H
#define __CALCULATE_H

//====一下三个定义了陀螺仪的偏差===========
#define Gx_offset -3.06
#define Gy_offset 1.01
#define Gz_offset -0.88
//====================
//MPU6050 accelgyro;
extern  uint32_t RunningTime=0;
int16_t ax,ay,az;
int16_t gx,gy,gz;//存储原始数据
float aax,aay,aaz,ggx,ggy,ggz;//存储量化后的数据
float Ax,Ay,Az;//单位 g(9.8m/s^2)
float Gx,Gy,Gz;//单位 °/s
 
float Angel_accX,Angel_accY,Angel_accZ;//存储加速度计算出的角度
 
long LastTime=0,NowTime=0,TimeSpan=0;//用来对角速度积分的
void loop(void);


#endif
