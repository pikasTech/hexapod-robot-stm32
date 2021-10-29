#ifndef __ACTION_TASK_H
#define __ACTION_TASK_H
#include "sys.h"

#define NORMAL 0
#define TEST 1
#define USE_T06_ideal 0
extern int action_temp[6][3];
extern int action_n;
extern int action_t;
extern float ChaBu_hNow;
extern float LRL_hNow;
extern float RLR_hNow;
//前申比例
extern float Rf;
extern u8 STA_STAND;
//前转比例
extern float Rz;

//移动命令向量 [Vx,Vy,w]
extern float Order[3];
extern float LRL_phase;
extern float RLR_phase;
extern int STEP_number;
extern float Action_T;
extern float Action_T_per_V;
extern float speed;
extern u8 STA_STOP;
extern u8 STA_TIME_STOP;     //单步停止位
extern u8 STA_TEST_TaiJiao;  //抬脚测试标志位
extern u8 STA_ACTION;        //运行模式
//伸展系数
extern double L_init, L_init_best;

//机身高度
extern double H_init, H_init_best;

void LH_to_xyz_init(double T03[3][1],
                    double d[4],
                    double L,
                    double H,
                    double xyz_init[3]);

#endif
