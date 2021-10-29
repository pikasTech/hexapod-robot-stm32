#include "ps2_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "balance_task.h"
#include "action_task_723.h"
#include "genetic_task.h"
//ps2任务函数

void ps2_task(void *pvParameters)
{

    while(1)
    {
            int LX,LY,RX,RY;
            u8 key;
            float direction[2];//平移方向
            key=PS2_DataKey();
//          if(Handkey!=0)
//          {
                if(key==PSB_SELECT)NVIC_SystemReset();

                if(key==PSB_L1)H_init-=0.4f;
                if(key==PSB_L2)H_init+=0.4f;

                if(key==PSB_R1)L_init-=0.4f;
                if(key==PSB_R2)L_init+=0.4f;

                if(key==PSB_L3)
                {
                    genetic_mode=0;
                    genetic_start=1;
                }

                if(key==PSB_R3)
                {
                    genetic_mode=1;
                    genetic_start=1;
                }

                if(key==PSB_PAD_UP)theatx+=0.8;
                if(key==PSB_PAD_DOWN)theatx-=0.8;
                if(key==PSB_PAD_LEFT)theaty+=0.8;
                if(key==PSB_PAD_RIGHT)theaty-=0.8;


                if(STA_ACTION == NORMAL)
                {
                    if(key==PSB_PINK)       Action_T_per_V-=50;
                    if(key==PSB_RED)        Action_T_per_V+=50;
                    if(key==PSB_GREEN)  pid_flag=1;
                    if(key==PSB_BLUE)       pid_flag=0;
                    if(key==PSB_PAD_UP)theatx+=0.8;
                    if(key==PSB_PAD_DOWN)theatx-=0.8;
                    if(key==PSB_PAD_LEFT)theaty+=0.8;
                    if(key==PSB_PAD_RIGHT)theaty-=0.8;
                }
                else if(STA_ACTION == TEST)
                {
                    if(key==PSB_PINK)       STA_TIME_STOP=0;//停时控制
                    if(key==PSB_RED)        STA_TIME_STOP=1;
                    if(key==PSB_PAD_UP)STA_TEST_TaiJiao = 1;//抬脚
                    if(key==PSB_PAD_DOWN)STA_TEST_TaiJiao = 0;//落脚
                    if(key==PSB_PAD_LEFT)theaty+=0.8;
                    if(key==PSB_PAD_RIGHT)theaty-=0.8;
                }

                LX=PS2_AnologData(PSS_LX);
                LY=PS2_AnologData(PSS_LY);
                RX=PS2_AnologData(PSS_RX);
                RY=PS2_AnologData(PSS_RY);


                direction[0]=(LX-128)/128.0f;
                direction[1]=(LY-128)/128.0f;

                if(RY<6)            STA_STOP=0,flag_action_run_continiue=1;
//              if(RY>250)      STA_STOP=1,flag_action_run_continiue=0;


                if(speed<=0)speed=0;
                if(speed>=20)speed=20;

                if(genetic_start==0)
                {
                    Order[0]=direction[0]*speed;
                    Order[1]=-direction[1]*speed;
                    Order[2]=-speed*0.03f*(RX-128)/128.0f;
                }
//          }
        vTaskDelay(50);

    }

}
