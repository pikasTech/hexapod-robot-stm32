#include "s32high_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"
/*该任务为六足距离地面距离判断任务，暂未添加至start_task*/
/*该任务包括PID计算*/
/*
RF RM RB :dis1 dis2 dis3
LF LM LB: disa diab disc
dis 格式 u16
*/
#define MaxDis    5          //定义允许偏差最大值
#define BacisDis  1000       //即六足着地时 正常距地距离
void InitPID(void);         //上电初始化设定值

void s32high_task(void *pvParameters)
{
    InitPID();
  while(1)
    {
      DisUpdate();      //更新设定值
        PID_Calculate(&RF_DIS);
        PID_Calculate(&RM_DIS);
        PID_Calculate(&RB_DIS);
        PID_Calculate(&LF_DIS);
        PID_Calculate(&LM_DIS);
        PID_Calculate(&LB_DIS);
        vTaskDelay(200); //计算周期 待改进
    }
}
void DisUpdate(void)   //设定值更新
{
  RF_DIS.Presentvale=RF_s32.dist/10;        //转mm为cm制度
    RM_DIS.Presentvale=RM_s32.dist/10;
    RB_DIS.Presentvale=RB_s32.dist/10;
  LF_DIS.Presentvale=LF_s32.dist/10;
    LM_DIS.Presentvale=LM_s32.dist/10;
    LB_DIS.Presentvale=LB_s32.dist/10;
    //pid的输出值为下次调整高度实际为毫米制 /10后得到cm
}
void InitPID(void)
{
    RF_DIS.Setvalue=7.5;   //设定值恒为静止时对地的距离；
    RF_DIS.Kp=1;
    RF_DIS.Kd=0;
    RF_DIS.OUT_0=0;

    RM_DIS.Setvalue=7.5;   //设定值恒为静止时对地的距离；
    RM_DIS.Kp=1;
    RM_DIS.Kd=0;
    RM_DIS.OUT_0=0;

    RB_DIS.Setvalue=7.5;   //设定值恒为静止时对地的距离；
    RB_DIS.Kp=1;
    RB_DIS.Kd=0;
    RB_DIS.OUT_0=0;

    LF_DIS.Setvalue=7.5;   //设定值恒为静止时对地的距离；
    LF_DIS.Kp=1;
    LF_DIS.Kd=0;
    LF_DIS.OUT_0=0;

    LM_DIS.Setvalue=7.5;   //设定值恒为静止时对地的距离；
    LM_DIS.Kp=1;
    LM_DIS.Kd=0;
    LM_DIS.OUT_0=0;

    LB_DIS.Setvalue=7.5;   //设定值恒为静止时对地的距离；
    LB_DIS.Kp=1;
    LB_DIS.Kd=0;
    LB_DIS.OUT_0=0;

}
