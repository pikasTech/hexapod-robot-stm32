#ifndef _PID_H
#define _PID_H
#include "sys.h"
typedef struct {
    float Setvalue;     //用户设定值
    float Presentvale;  //当前获得值
    float Kp;
    float T;  //采样周期
    // float Ti;   //积分常数
    float Ki;
    // float Td;   //微分常数
    float Kd;
    float Ek;     //本次偏差
    float Ek_1;   //上次偏差
    float SumEk;  //历史偏差之和
    float OUT_0;  //计算值为0时，维持当前输出
    float OUT;    //本次计算输出的控制量
} PID;
void PID_Calculate(PID* pid);  // PID 计算，当计算周期到达时计算 并返回计算值。
//初始化设定值设置当前值 设置Kp Ki Kd 设置采样周期（用不到），设置OUT_0
//需要更新值 Pre
#endif
