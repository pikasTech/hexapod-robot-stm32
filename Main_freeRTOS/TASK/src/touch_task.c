#include "touch_task.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "key.h"
#include "action_task_723.h"
#include "balance_task.h"

struct TOUCH touch_use;
u8 STA_ChuDi_rm = 0;
u8 STA_ChuDi_lf = 0;
u8 STA_ChuDi_lb = 0;
u8 STA_ChuDi_lm = 0;
u8 STA_ChuDi_rf = 0;
u8 STA_ChuDi_rb = 0;
void get_touch(struct TOUCH *touch_fun)
{
		(*touch_fun).rf = TOUCH_RF;
		(*touch_fun).rm = TOUCH_RM;
		(*touch_fun).rb = TOUCH_RB;
		(*touch_fun).lf = TOUCH_LF;
		(*touch_fun).lm = TOUCH_LM;
		(*touch_fun).lb = TOUCH_LB;
}

float cheat_STA_ChuDi_BuChang = 0;
void cheak_STA_ChuDi(void)
{


if (STA_ACTION == TEST)
			{
				if (STA_ChuDi_rm == 0 && touch_use.rm == 0)
				{
					STA_ChuDi_rm = 1;//已触地
					T06_RM[2] = T06_init_RM[2]+ChaBu_hNow + 1.5;
				}
		//							T06_init_RM[2] = T06_RM[2];
				if (STA_ChuDi_lf == 0 && touch_use.lf == 0)
				{	
					STA_ChuDi_lf = 1;
					T06_LF[2] = T06_init_LF[2]+ChaBu_hNow + 1.5;
				}
		//							T06_init_LF[2] = T06_LF[2];
				if (STA_ChuDi_lb == 0 && touch_use.lb == 0)
				{
					STA_ChuDi_lb = 1;
					T06_LB[2] = T06_init_LB[2]+ChaBu_hNow + 1.5;
				}
			}
			else if(STA_ACTION ==NORMAL)
			{
					//							T06_init_RM[2] = T06_RM[2];
				if (STA_ChuDi_lf == 0 && touch_use.lf == 0)
				{	
					STA_ChuDi_lf = 1;
					T06_init_LF[2] = T06_LF[2]-cheat_STA_ChuDi_BuChang;//记录当前站立实际高度
				}
				
				if (STA_ChuDi_rm == 0 && touch_use.rm == 0)
				{
					STA_ChuDi_rm = 1;//已触地
					T06_init_RM[2] = T06_RM[2]-cheat_STA_ChuDi_BuChang;//记录当前站立实际高度
				}

		//							T06_init_RM[2] = T06_RM[2];
				if (STA_ChuDi_lb == 0 && touch_use.lb == 0)
				{
					STA_ChuDi_lb = 1;
					T06_init_LB[2] = T06_LB[2]-cheat_STA_ChuDi_BuChang;//记录当前站立实际高度
				}
				
				if (STA_ChuDi_lm == 0 && touch_use.lm == 0)
				{
					STA_ChuDi_lm = 1;//已触地
					T06_init_LM[2] = T06_LM[2]-cheat_STA_ChuDi_BuChang;
				}
		//							T06_init_RM[2] = T06_RM[2];
				if (STA_ChuDi_rf == 0 && touch_use.rf == 0)
				{	
					STA_ChuDi_rf = 1;
					T06_init_RF[2] = T06_RF[2]-cheat_STA_ChuDi_BuChang;
				}
		//							T06_init_LF[2] = T06_LF[2];
				if (STA_ChuDi_rb == 0 && touch_use.rb == 0)
				{
					STA_ChuDi_rb = 1;
					T06_init_RB[2] = T06_RB[2]-cheat_STA_ChuDi_BuChang;
				}
				
			}


}
float step_init_BuChang = 0.002;
void touch_task(void *pvParameters)
{

	while(1)
	{
		get_touch(&touch_use);//获取当前的触地情况
		#if USE_STA_touch_ChuDi
		if(STA_STAND==1)
		{

				if(touch_use.lf == 1)
				{
					T06_init_LF[2] -=step_init_BuChang;					
				}
				if(touch_use.lm == 1)
				{
					T06_init_LM[2] -=step_init_BuChang;						
				}
				if(touch_use.lb == 1)
				{
					T06_init_LB[2] -=step_init_BuChang;						
				}
				if(touch_use.rf == 1)
				{
					T06_init_RF[2] -=step_init_BuChang;						
				}
				if(touch_use.rm == 1)
				{
					T06_init_RM[2] -=step_init_BuChang;						
				}
				if(touch_use.rb == 1)
				{
					T06_init_RB[2] -=step_init_BuChang;						
				}				
		}
		#endif
		#if USE_STA_ChuDi
			cheak_STA_ChuDi();
		//							T06_init_LB[2] = T06_LB[2];
		#endif
//		printf("[%d,%f]\r\n",touch_use.rf,T06_RF[2]+10.0f);
		
		vTaskDelay(1);
	}

}

