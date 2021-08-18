#include "adc.h"
#include "delay.h"		 
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//ADC 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

__IO uint16_t ADC_ConvertedValue[RHEOSTAT_NOFCHANEL]={0};		
//初始化ADC															   
void  Adc_Init(void)
{
//初始化ADC1	
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

  //先初始化ADC1通道5 IO口
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5;//PA5 通道5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化  
 
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 
 
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//初始化
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;//扫描模式开启
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//连续转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;//程序来自网络
  ADC_InitStructure.ADC_NbrOfConversion = RHEOSTAT_NOFCHANEL;//转换6个通道 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	
	ADC_RegularChannelConfig(ADC1, 0, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
//  ADC_RegularChannelConfig(ADC1, 1, 2, ADC_SampleTime_480Cycles );
////	ADC_RegularChannelConfig(ADC1, 2, 3, ADC_SampleTime_480Cycles );
////	ADC_RegularChannelConfig(ADC1, 3, 4, ADC_SampleTime_480Cycles );
//	ADC_RegularChannelConfig(ADC1, 4, 3, ADC_SampleTime_480Cycles );
//	ADC_RegularChannelConfig(ADC1, 5, 4, ADC_SampleTime_480Cycles );
	
   // 使能DMA请求 after last transfer (Single-ADC mode)
  ADC_DMARequestAfterLastTransferCmd(RHEOSTAT_ADC, ENABLE);
  // 使能ADC DMA
  ADC_DMACmd(RHEOSTAT_ADC, ENABLE);
	
	ADC_Cmd(ADC1, ENABLE);//开启AD转换器	
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
//初始化DMA

		
		 //使能DMA2对应的时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); 
		
		DMA_DeInit(DMA2_Stream0);
		
		// 选择 DMA 通道，通道存在于流中
		//配置为DMA2的通道0
		DMA_InitStructure.DMA_Channel = RHEOSTAT_ADC_DMA_CHANNEL; 
		// 外设基址为：ADC 数据寄存器地址
		//外设基地址为ADC3的基质+0x4c
		DMA_InitStructure.DMA_PeripheralBaseAddr = RHEOSTAT_ADC_DR_ADDR;	
		
		//注意这里加不加&
		// 存储器地址，实际上就是一个内部SRAM的变量
		DMA_InitStructure.DMA_Memory0BaseAddr = (u32)ADC_ConvertedValue;  
		
		
		// 数据传输方向为外设到存储器	
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	
		// 缓冲区大小为，指一次传输的数据量
		DMA_InitStructure.DMA_BufferSize = RHEOSTAT_NOFCHANEL;	
		
		// 外设寄存器只有一个，地址不用递增
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		// 存储器地址自增
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
		// // 外设数据大小为半字，即两个字节 
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 
		//	存储器数据大小也为半字，跟外设数据大小相同
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	
		// 循环传输模式
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		// DMA 传输通道优先级为高，当使用一个DMA通道时，优先级设置不影响
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		// 禁止DMA FIFO	，使用直连模式
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  
		// FIFO 大小，FIFO模式禁止时，这个不用配置	
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  
		
		//初始化DMA流，流相当于一个大的管道，管道里面有很多通道
		DMA_Init(RHEOSTAT_ADC_DMA_STREAM, &DMA_InitStructure);
		// 使能DMA流
		DMA_Cmd(RHEOSTAT_ADC_DMA_STREAM, ENABLE);
}				  
//获得ADC值
//ch: @ref ADC_channels 
//通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//返回值:转换结果
u16 Get_Adc(u8 ch)   
{
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	 
//	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}
//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
	}
	return temp_val/times;
} 
	 









