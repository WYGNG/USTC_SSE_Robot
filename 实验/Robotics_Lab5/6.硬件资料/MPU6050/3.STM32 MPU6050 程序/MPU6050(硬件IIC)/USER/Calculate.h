#ifndef __CALCULATE_H
#define __CALCULATE_H

//====һ�����������������ǵ�ƫ��===========
#define Gx_offset -3.06
#define Gy_offset 1.01
#define Gz_offset -0.88
//====================
//MPU6050 accelgyro;
extern  uint32_t RunningTime=0;
int16_t ax,ay,az;
int16_t gx,gy,gz;//�洢ԭʼ����
float aax,aay,aaz,ggx,ggy,ggz;//�洢�����������
float Ax,Ay,Az;//��λ g(9.8m/s^2)
float Gx,Gy,Gz;//��λ ��/s
 
float Angel_accX,Angel_accY,Angel_accZ;//�洢���ٶȼ�����ĽǶ�
 
long LastTime=0,NowTime=0,TimeSpan=0;//�����Խ��ٶȻ��ֵ�
void loop(void);


#endif
