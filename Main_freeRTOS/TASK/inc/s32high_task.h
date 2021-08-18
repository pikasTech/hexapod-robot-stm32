#ifndef __s32high_TASK_H
#define __s32high_TASK_H
#include "sys.h"
#include "action_task_723.h"
#include "pid.h"
PID  RF_DIS;
PID  RM_DIS;
PID  RB_DIS;
PID  LF_DIS;
PID  LM_DIS;
PID  LB_DIS;
void DisUpdate(void);   //∏¸–¬ ‰»Î
void InitPID(void);
#endif
