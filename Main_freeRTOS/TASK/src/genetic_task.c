#include "genetic_task.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "action_task_723.h"
//genetic任务函数

int genetic_mode;
int genetic_start=0;
void genetic_task(void *pvParameters)
{

	while(1)
	{
	
			int i;
			while(STA_STOP==1)vTaskDelay(30);		
			while(genetic_start==0)vTaskDelay(30);	
			initiate();				//产生初始化种群 
			evaluation( 0 ,genetic_mode);		//对初始化种群进行评估、排序 

			for( i = 0 ; i < MAXloop ; i++ )
		{
			while(STA_STOP==1)vTaskDelay(30);
			cross();			//进行交叉操作 
			evaluation( 1 ,genetic_mode);	//对子种群进行评估、排序 
			mutation(genetic_mode);			//变异操作 				
			selection();		//对父子种群中选择最优的NUM个作为新的父种群 
			if( record() == 1 )	//满足终止规则1，则flag=1并停止循环 
			{
				break;
			}
		}
		
		while(1)  		vTaskDelay(30);
		
	}

}
