#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#define PI 3.14159265

#define DIFFERENCE 100
extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
void TIM3_IRQHandler(void);
int balance(float angle,float gyro);
int velocity(int encoder_left,int encoder_right);
int turn(u8 CCD,float gyro);
void Set_Pwm(int moto1,int moto2);
void Key(void);
void Xianfu_Pwm(void);
u8 Turn_Off(float angle, int voltage);
void Get_Angle(u8 way);
int myabs(int a);
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
int Put_Down(float Angle,int encoder_left,int encoder_right);
void  Get_MC6(void);
void  Find_CCD_Zhongzhi(void);
#endif
