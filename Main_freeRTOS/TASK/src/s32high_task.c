#include "s32high_task.h"
#include "FreeRTOS.h"
#include "pid.h"
#include "task.h"
/*������Ϊ��������������ж�������δ������start_task*/
/*���������PID����*/
/*
RF RM RB :dis1 dis2 dis3
LF LM LB: disa diab disc
dis ��ʽ u16
*/
#define MaxDis 5       //��������ƫ�����ֵ
#define BacisDis 1000  //�������ŵ�ʱ ������ؾ���
void InitPID(void);  //�ϵ��ʼ���趨ֵ

void s32high_task(void* pvParameters) {
    InitPID();
    while (1) {
        DisUpdate();  //�����趨ֵ
        PID_Calculate(&RF_DIS);
        PID_Calculate(&RM_DIS);
        PID_Calculate(&RB_DIS);
        PID_Calculate(&LF_DIS);
        PID_Calculate(&LM_DIS);
        PID_Calculate(&LB_DIS);
        vTaskDelay(200);  //�������� ���Ľ�
    }
}
void DisUpdate(void)  //�趨ֵ����
{
    RF_DIS.Presentvale = RF_s32.dist / 10;  //תmmΪcm�ƶ�
    RM_DIS.Presentvale = RM_s32.dist / 10;
    RB_DIS.Presentvale = RB_s32.dist / 10;
    LF_DIS.Presentvale = LF_s32.dist / 10;
    LM_DIS.Presentvale = LM_s32.dist / 10;
    LB_DIS.Presentvale = LB_s32.dist / 10;
    // pid�����ֵΪ�´ε����߶�ʵ��Ϊ������ /10��õ�cm
}
void InitPID(void) {
    RF_DIS.Setvalue = 7.5;  //�趨ֵ��Ϊ��ֹʱ�Եصľ��룻
    RF_DIS.Kp = 1;
    RF_DIS.Kd = 0;
    RF_DIS.OUT_0 = 0;

    RM_DIS.Setvalue = 7.5;  //�趨ֵ��Ϊ��ֹʱ�Եصľ��룻
    RM_DIS.Kp = 1;
    RM_DIS.Kd = 0;
    RM_DIS.OUT_0 = 0;

    RB_DIS.Setvalue = 7.5;  //�趨ֵ��Ϊ��ֹʱ�Եصľ��룻
    RB_DIS.Kp = 1;
    RB_DIS.Kd = 0;
    RB_DIS.OUT_0 = 0;

    LF_DIS.Setvalue = 7.5;  //�趨ֵ��Ϊ��ֹʱ�Եصľ��룻
    LF_DIS.Kp = 1;
    LF_DIS.Kd = 0;
    LF_DIS.OUT_0 = 0;

    LM_DIS.Setvalue = 7.5;  //�趨ֵ��Ϊ��ֹʱ�Եصľ��룻
    LM_DIS.Kp = 1;
    LM_DIS.Kd = 0;
    LM_DIS.OUT_0 = 0;

    LB_DIS.Setvalue = 7.5;  //�趨ֵ��Ϊ��ֹʱ�Եصľ��룻
    LB_DIS.Kp = 1;
    LB_DIS.Kd = 0;
    LB_DIS.OUT_0 = 0;
}
