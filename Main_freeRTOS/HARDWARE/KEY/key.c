#include "key.h"
#include "delay.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//按键输入驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

//外部中断0服务程序
void EXTI3_IRQHandler(void)
{
	delay_xms(10);	//消抖
	 
	 EXTI_ClearITPendingBit(EXTI_Line3); //清除LINE0上的中断标志位 
}	

//外部中断9_5服务程序
void EXTI9_5_IRQHandler(void)
{
	delay_xms(10);	//消抖
 
 if(EXTI_GetITStatus(EXTI_Line5)!=RESET)     
		{
				 //中断处理，自行添加
				 EXTI_ClearITPendingBit(EXTI_Line5);
		}
 if(EXTI_GetITStatus(EXTI_Line6)!=RESET)     
		{
				 //中断处理，自行添加
				 EXTI_ClearITPendingBit(EXTI_Line6);
		}
 if(EXTI_GetITStatus(EXTI_Line7)!=RESET)     
		{
				 //中断处理，自行添加
				 EXTI_ClearITPendingBit(EXTI_Line7);
		}
 if(EXTI_GetITStatus(EXTI_Line9)!=RESET)     
		{
				 //中断处理，自行添加
				 EXTI_ClearITPendingBit(EXTI_Line9);
		}		
}
//外部中断15_10服务程序
void EXTI15_10_IRQHandler(void)
{
	delay_xms(10);	//消抖
	 if(EXTI_GetITStatus(EXTI_Line11)!=RESET)     
		{
				 //中断处理，自行添加
				 EXTI_ClearITPendingBit(EXTI_Line11);
		}	

}

#define USE_EXTI 0
//按键初始化函数
void KEY_EXTI_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;
#if USE_EXTI	
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
#endif
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOG, ENABLE);//使能GPIOA,GPIOE时钟
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_7; //KEY0 KEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOE2,3,4
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //KEY0 KEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIOE2,3,4

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11; //KEY0 KEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化GPIOE2,3,4
		 		 
#if USE_EXTI
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
	SYSCFG_EXTILineConfig(RCC_AHB1Periph_GPIOB, EXTI_PinSource3);
	SYSCFG_EXTILineConfig(RCC_AHB1Periph_GPIOB, EXTI_PinSource5);
	SYSCFG_EXTILineConfig(RCC_AHB1Periph_GPIOB, EXTI_PinSource7);	
	
	SYSCFG_EXTILineConfig(RCC_AHB1Periph_GPIOD, EXTI_PinSource6);

	SYSCFG_EXTILineConfig(RCC_AHB1Periph_GPIOG, EXTI_PinSource9);
	SYSCFG_EXTILineConfig(RCC_AHB1Periph_GPIOG, EXTI_PinSource11);
	
	/* 配置EXTI_Line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line3|EXTI_Line5|EXTI_Line6|EXTI_Line7|EXTI_Line9|EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //上升沿触发 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
  EXTI_Init(&EXTI_InitStructure);//配置

 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//外部中断0
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x06;//抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断2
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x06;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//外部中断3
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x06;//抢占优先级2
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
#endif
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//WK_UP对应引脚PA0
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;//下拉
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA0

} 
//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下
//1，KEY0按下
//2，KEY1按下
//3，KEY2按下 
//4，WKUP按下 WK_UP
//注意此函数有响应优先级,KEY0>KEY1>KEY2>WK_UP!!
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&(KEY0==0||KEY1==0||KEY2==0))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY0==0)return 1;
		else if(KEY1==0)return 2;
		else if(KEY2==0)return 3;
//		else if(WK_UP==1)return 4;
	}else if(KEY0==1&&KEY1==1&&KEY2==1)key_up=1; 	    
 	return 0;// 无按键按下
}




















