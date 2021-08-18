#include "sanjiaobo_task.h" 
#include "FreeRTOS.h"
#include "task.h"
extern int test[];
extern int flag_hanshu1;
extern int flag_hanshu2;
//sanjiaoboÈÎÎñº¯Êı
void sanjiaobo_task(void *pvParameters)
{
	
	while(1)
	{
		if(test[0]<60&&flag_hanshu1==0)test[0]+=3;
		if(test[0]==60)flag_hanshu1=1;
		if(test[0]>0&&flag_hanshu1==1)test[0]--;
		if(test[0]==0)flag_hanshu1=0;
		vTaskDelay(3);
		if(test[1]<30&&flag_hanshu2==0)test[1]+=3;
		if(test[1]==30)flag_hanshu2=1;
		if(test[1]>0&&flag_hanshu2==1)test[1]--;
		if(test[1]==0)flag_hanshu2=0;

	}
}

