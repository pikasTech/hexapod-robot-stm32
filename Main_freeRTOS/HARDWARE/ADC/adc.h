#ifndef __ADC_H
#define __ADC_H
#include "sys.h"
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

void Adc_Init(void);                //ADC通道初始化
u16  Get_Adc(u8 ch);                //获得某个通道值
u16 Get_Adc_Average(u8 ch,u8 times);//得到某个通道给定次数采样的平均值
#define RHEOSTAT_NOFCHANEL      1

extern __IO uint16_t ADC_ConvertedValue[RHEOSTAT_NOFCHANEL];

// ADC 序号宏定义
#define RHEOSTAT_ADC              ADC1
#define RHEOSTAT_ADC_CLK          RCC_APB2Periph_ADC1
// ADC DR寄存器宏定义，ADC转换后的数字值则存放在这里

//ADC1
#define RHEOSTAT_ADC_DR_ADDR    ((u32)ADC1+0x4c)

// ADC DMA 通道宏定义，这里我们使用DMA传输
#define RHEOSTAT_ADC_DMA_CLK      RCC_AHB1Periph_DMA2
#define RHEOSTAT_ADC_DMA_CHANNEL  DMA_Channel_0
#define RHEOSTAT_ADC_DMA_STREAM   DMA2_Stream0

u16 Get_Adc3(u8 ch);                //获得某个通道值
//u16 Get_Adc31(u8 nan);
//u16 Get_Adc32(u8 xi);
//u16 Get_Adc33(u8 dong);
//u16 Get_Adc_Average(u8 ch,u8 times);//得到某个通道给定次数采样的平均值



#endif















