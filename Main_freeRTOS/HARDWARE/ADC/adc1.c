#i nclude "stm32f10x.h" //???????STM32F10x?????????????????
#i nclude "eval.h" //???(????????LED?????)
#i nclude "SysTickDelay.h"
#i nclude "UART_INTERFACE.h"
#i nclude <stdio.h>

#define N 50 //????50?
#define M 12 //?12???

vu16 AD_Value[N][M]; //????ADC????,??DMA?????
vu16 After_filter[M]; //?????????????
int i;



void GPIO_Configuration(void)
{
GPIO_InitTypeDef GPIO_InitStructure;

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //??USART1???????????GPIO???,???????????
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA, &GPIO_InitStructure);


GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
GPIO_Init(GPIOA, &GPIO_InitStructure);



//PA0/1/2 ??????????
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //??????
GPIO_Init(GPIOA, &GPIO_InitStructure);

//PB0/1 ??????????
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //??????
GPIO_Init(GPIOB, &GPIO_InitStructure);

//PC0/1/2/3/4/5 ??????????
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //??????
GPIO_Init(GPIOC, &GPIO_InitStructure);
}

}


void RCC_Configuration(void)
{
ErrorStatus HSEStartUpStatus;

RCC_DeInit(); //RCC ????
RCC_HSEConfig(RCC_HSE_ON); //??HSE
HSEStartUpStatus = RCC_WaitForHSEStartUp(); //??HSE???
if(HSEStartUpStatus == SUCCESS)
{
FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); //Enable Prefetch Buffer
FLASH_SetLatency(FLASH_Latency_2); //Set 2 Latency cycles
RCC_HCLKConfig(RCC_SYSCLK_Div1); //AHB clock = SYSCLK
RCC_PCLK2Config(RCC_HCLK_Div1); //APB2 clock = HCLK
RCC_PCLK1Config(RCC_HCLK_Div2); //APB1 clock = HCLK/2
RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6); //PLLCLK = 12MHz * 6 = 72 MHz
RCC_PLLCmd(ENABLE); //Enable PLL
while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); //Wait till PLL is ready
RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //Select PLL as system clock source
while(RCC_GetSYSCLKSource() != 0x08); //Wait till PLL is used as system clock source

RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB
| RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO |RCC_APB2Periph_USART1, ENABLE ); //??ADC1????,??????

RCC_ADCCLKConfig(RCC_PCLK2_Div6); //72M/6=12,ADC????????14M
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //??DMA??

}
}


void ADC1_Configuration(void)
{
ADC_InitTypeDef ADC_InitStructure;

ADC_DeInit(ADC1); //??? ADC1 ????????????


ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //ADC????:ADC1?ADC2???????
ADC_InitStructure.ADC_ScanConvMode =ENABLE; //???????????
ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //?????????????
ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //????????
ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //ADC?????
ADC_InitStructure.ADC_NbrOfChannel = M; //?????????ADC?????
ADC_Init(ADC1, &ADC_InitStructure); //??ADC_InitStruct???????????ADCx????


//????ADC??????,??????????????
//ADC1,ADC??x,????????y,?????239.5??
ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 );
ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5 );
ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5 );
ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5 );
ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 5, ADC_SampleTime_239Cycles5 );
ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 6, ADC_SampleTime_239Cycles5 );
ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 7, ADC_SampleTime_239Cycles5 );
ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 8, ADC_SampleTime_239Cycles5 );
ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 9, ADC_SampleTime_239Cycles5 );
ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 10, ADC_SampleTime_239Cycles5 );
ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 11, ADC_SampleTime_239Cycles5 );
ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 12, ADC_SampleTime_239Cycles5 );

// ??ADC?DMA??(???DMA??,??????DMA?????)
ADC_DMACmd(ADC1, ENABLE);


ADC_Cmd(ADC1, ENABLE); //?????ADC1

ADC_ResetCalibration(ADC1); //?????ADC1??????

while(ADC_GetResetCalibrationStatu

s(ADC1)); //??ADC1??????????,???????


ADC_StartCalibration(ADC1); //????ADC1?????

while(ADC_GetCalibrationStatus(ADC1)); //????ADC1?????,???????


}


void DMA_Configuration(void)
{

DMA_InitTypeDef DMA_InitStructure;
DMA_DeInit(DMA1_Channel1); //?DMA???1?????????
DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR; //DMA??ADC???
DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value; //DMA?????
DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //????????????
DMA_InitStructure.DMA_BufferSize = N*M; //DMA???DMA?????
DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //?????????
DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //?????????
DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //?????16?
DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //?????16?
DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //?????????
DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA?? x??????
DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //DMA??x????????????
DMA_Init(DMA1_Channel1, &DMA_InitStructure); //??DMA_InitStruct?????????DMA???

}


//??????
void Init_All_Periph(void)
{

RCC_Configuration();

GPIO_Configuration();

ADC1_Configuration();

DMA_Configuration();

//USART1_Configuration();
USART_Configuration(9600);


}



u16 GetVolt(u16 advalue)

{

return (u16)(advalue * 330 / 4096); //???????100?,????????

}




void filter(void)
{
int sum = 0;
u8 count;
for(i=0;i<12;i++)

{

for ( count=0;count<N;count++)

{

sum += AD_Value[count][i];

}

After_filter[i]=sum/N;

sum=0;
}

}




int main(void)
{

u16 value[M];

init_All_Periph();
SysTick_Initaize();


ADC_SoftwareStartConvCmd(ADC1, ENABLE);
DMA_Cmd(DMA1_Channel1, ENABLE); //??DMA??
while(1)
{
while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);//?????????????????

filter();
for(i=0;i<12;i++)
{
value[i]= GetVolt(After_filter[i]);

printf("value[%d]:\t%d.%dv\n",i,value[i]/100,value[i]0) ;
delay_ms(100);
}
}

}
