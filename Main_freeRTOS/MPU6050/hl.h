#ifndef __HL_H
#define __HL_H
#include "sys.h"

#define K_angle 11.6666

#define LB 0
#define LM 1
#define LF 2
#define RF 3
#define RM 4
#define RB 5

#define LB1_init 1500
#define LB2_init 1500
#define LB3_init 1000
#define LM1_init 1500
#define LM2_init 1500
#define LM3_init 1000
#define LF1_init 1500
#define LF2_init 1500
#define LF3_init 1000
#define RB1_init 1500
#define RB2_init 1500
#define RB3_init 2000
#define RM1_init 1500
#define RM2_init 1500
#define RM3_init 2000
#define RF1_init 1500
#define RF2_init 1500
#define RF3_init 2000

#define LB1 16
#define LB2 17
#define LB3 18
#define LM1 13
#define LM2 14
#define LM3 15
#define LF1 10
#define LF2 11
#define LF3 12
#define RF1 1
#define RF2 2
#define RF3 3
#define RM1 4
#define RM2 5
#define RM3 6
#define RB1 7
#define RB2 8
#define RB3 9

#define L60_3 4.7458*0.00001
#define L60_2 -0.022967
#define L60_1 4.43149
#define L60_0 -319.33

#define L70_3 4.7411*0.00001
#define L70_2 -0.0222
#define L70_1 4.20473
#define L70_0 -301.76

#define L80_3 4.79330*0.00001
#define L80_2 -0.021729
#define L80_1 3.9944
#define L80_0 -284.0922

#define L90_3 4.89353467143924*0.00001
#define L90_2 -0.0213216793600082
#define L90_1 3.79134958393646
#define L90_0 -266.014303050716

#define L100_3 5.02878739380669*0.00001
#define L100_2 -0.0209507686443580
#define L100_1 3.58286361128485
#define L100_0 -247.102914018675

#define L110_3 5.17802432288606*0.00001
#define L110_2 -0.0204981725081162
#define L110_1 3.35268161968685
#define L110_0 -226.836024091699

#define L120_3 5.30645620254704*0.00001
#define L120_2 -0.0197953496623206
#define L120_1 3.08041842877280
#define L120_0 -204.666200651396

#define L130_3 5.35636817364911*0.00001
#define L130_2 -0.0186054274802930
#define L130_1 2.74226578743364
#define L130_0 -180.162547564973

#define FOOT_L_A 75.00//大腿长度
#define FOOT_L_B 165.00//小腿长度
#define FOOT_L_C 60.00 //腿根长度

void action_angle(u8 number,double angle,int time);//单关节控制(角度制)
void action_foot(u8 foot,double angle1,double angle2,double angle3,int time);//单腿全变量控制

#endif
