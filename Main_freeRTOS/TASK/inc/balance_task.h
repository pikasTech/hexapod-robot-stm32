#ifndef __MPU6050_TASK_H
#define __MPU6050_TASK_H
#include "sys.h"

extern double theatx,theaty;//可控旋转角
//机器人足端向量
extern double T06_RF[3];
extern double T06_RM[3];
extern double T06_RB[3];
extern double T06_LF[3];
extern double T06_LM[3];
extern double T06_LB[3];

extern int pid_flag;

extern float Pitch;
extern float Roll;
extern float Pitch_use;
extern float Roll_use;

//机器人足端向量初始值（静立）
extern double T06_init_RF[3];
extern double T06_init_RM[3];
extern double T06_init_RB[3];
extern double T06_init_LF[3];
extern double T06_init_LM[3];
extern double T06_init_LB[3];

//机器人足端向量初始值（静立）(理想值）
extern double T06_init_RF_ideal[3];
extern double T06_init_RM_ideal[3];
extern double T06_init_RB_ideal[3];
extern double T06_init_LF_ideal[3];
extern double T06_init_LM_ideal[3];
extern double T06_init_LB_ideal[3];

//机器人机械参数
extern double d_RF[4];
extern double d_RM[4];
extern double d_RB[4];
extern double d_LF[4];
extern double d_LM[4];
extern double d_LB[4];

struct INIT_BuChang
{
    float rf;
    float rm;
    float rb;
    float lf;
    float lm;
    float lb;
};

extern struct _lobot_servo_ servos_array[18];
extern u8 flag_action_run_single,flag_action_run_continiue;

void T06_init(void);
void T06_init_ideal(void);
#endif
