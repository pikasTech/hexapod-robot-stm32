#include "blobs_task.h"
#include "FreeRTOS.h"
#include "action_task_723.h"
#include "task.h"
#include "usart.h"
// blobs������
struct BLOB_USE blobs_use;
#define error 0
#define right 1

float blobs_goal = 0;
float blobs_Integral;
float Kp_blobs = -0.02f;
float Kd_blobs;
float Ki_blobs;

float pid_blobs(float Angle) {
    static float Bias_last;
    float Bias;
    float balance;
    float blobs_D;
    Bias = Angle - (blobs_goal);  //===���ƽ��ĽǶ���ֵ �ͻ�е���
    blobs_Integral += Bias;
    blobs_D = Bias - Bias_last;
    balance = Kp_blobs * Bias + blobs_D * Kd_blobs +
              blobs_Integral *
                  Ki_blobs;  //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ��
    Bias_last = Bias;
    return balance;
}

float PidOutPut_blobs;
void blobs_task(void* pvParameters) {
    while (1) {
        u8 STA_blobs;

        if (blobs.x == 255 && blobs.y == 255)
            STA_blobs = error;
        else
            STA_blobs = right;

        if (STA_blobs == right) {
            blobs_use.x = blobs.x - 80;
            blobs_use.y = -(blobs.y - 60);
        }

        PidOutPut_blobs = pid_blobs(blobs_use.x);
        if (PidOutPut_blobs <= 0.15f)
            ;
        else
            PidOutPut_blobs = 0.15f;
        if (PidOutPut_blobs <= -0.15f)
            PidOutPut_blobs = -0.15f;

        if (STA_blobs == right) {
            Order[2] = PidOutPut_blobs;
            Order[2] = PidOutPut_blobs;
        }
        if (STA_blobs == error) {
            Order[2] = 0;
            Order[2] = 0;
        }

        vTaskDelay(20);
    }
}
