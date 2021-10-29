#include "sys.h"
#include "usart.h"
#include "LobotServoController.h"
//////////////////////////////////////////////////////////////////////////////////
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"                   //FreeRTOS使用
#include "task.h"
#endif
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F4探索者开发板
//串口1初始化
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/6/10
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
    int handle;
};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
    x = x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
    while((USART6->SR&0X40)==0);//循环发送,直到发送完毕
    USART6->DR = (u8) ch;
    return ch;
}

int fputc4(int ch)
{
    while((UART4->SR&0X40)==0);//循环发送,直到发送完毕
    UART4->DR = (u8) ch;
    return ch;
}

int fputc6(int ch)
{
    while((USART6->SR&0X40)==0);//循环发送,直到发送完毕
    USART6->DR = (u8) ch;
    return ch;
}

#endif

//static unsigned char TxBuffer[256];
//static unsigned char TxCounter=0;
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，    接收完成标志
//bit14，    接收到0x0d
//bit13~0，  接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记
u8 UART_RX_BUF[16];
bool isUartRxCompleted = false;
//初始化IO 串口1
//bound:波特率
void uart1_init(u32 bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟

    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1

    //USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
    USART_Init(USART1, &USART_InitStructure); //初始化串口1

    USART_Cmd(USART1, ENABLE);  //使能串口1

    //USART_ClearFlag(USART1, USART_FLAG_TC);

#if EN_USART1_RX
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;       //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器、

#endif

}

void uart2_init(u32 bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟

    //串口2对应引脚复用映射
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2

    //USART2端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2与GPIOA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA2，PA3

   //USART2 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
    USART_Init(USART2, &USART_InitStructure); //初始化串口2

    USART_Cmd(USART2, ENABLE);  //使能串口2

    //USART_ClearFlag(USART2, USART_FLAG_TC);

#if EN_USART2_RX
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断

    //USART2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;       //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器、

#endif

}

void uart4_init(u32 bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOC时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能UART4时钟

    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOA9复用为UART4
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOA10复用为UART4

    //UART4端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOA9与GPIOA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PA9，PA10

   //UART4 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
    USART_Init(UART4, &USART_InitStructure); //初始化串口1

    USART_Cmd(UART4, ENABLE);  //使能串口1

    //USART_ClearFlag(UART4, USART_FLAG_TC);

#if EN_UART4_RX
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启相关中断

    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;       //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器、

#endif

}
void uart3_init(u32 bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟

    //串口3对应引脚复用映射
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10复用为USART3
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11复用为USART3

    //USART3端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB10与GPIOB11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA2，PA3

   //USART3 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
    USART_Init(USART3, &USART_InitStructure); //初始化串口3

    USART_Cmd(USART3, ENABLE);  //使能串口3

    //USART_ClearFlag(USART3, USART_FLAG_TC);

#if EN_USART3_RX
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断

    //USART3 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口3中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;       //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器、

#endif

}


void uart6_init(u32 bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟

    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOA9复用为USART6
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOA10复用为USART6

    //USART6端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOA9与GPIOA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PA9，PA10

   //USART6 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
    USART_Init(USART6, &USART_InitStructure); //初始化串口1

    USART_Cmd(USART6, ENABLE);  //使能串口1

    //USART_ClearFlag(USART6, USART_FLAG_TC);

#if EN_USART6_RX
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断

    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;       //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器、

#endif

}
//static unsigned char count=0;
struct _rplidar_response_measurement_node_t node_dis;
struct BLOB         blobs;
struct STime        stcTime;
struct SAcc         stcAcc;
struct SGyro        stcGyro;
struct SAngle   stcAngle;
struct SMag         stcMag;
struct SDStatus stcDStatus;
struct SPress   stcPress;
struct SLonLat  stcLonLat;
struct SGPSV        stcGPSV;
struct small32  RF_s32,RM_s32,RB_s32,LF_s32,LM_s32,LB_s32;

//与陀螺仪通讯，获得欧拉角和角速度
void CopeSerial2Data(unsigned char ucData)
{
    static unsigned char ucRxBuffer[250];
    static unsigned char ucRxCnt = 0;

    ucRxBuffer[ucRxCnt++]=ucData;
    if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
    {
        ucRxCnt=0;
        return;
    }
    if (ucRxCnt<11) {return;}//数据不满11个，则返回
    else
    {
        switch(ucRxBuffer[1])
        {
            case 0x50:  memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据共同体里面，从而实现数据的解析。
            case 0x51:  memcpy(&stcAcc,&ucRxBuffer[2],8);break;
            case 0x52:  memcpy(&stcGyro,&ucRxBuffer[2],8);break;
            case 0x53:  memcpy(&stcAngle,&ucRxBuffer[2],8);break;
            case 0x54:  memcpy(&stcMag,&ucRxBuffer[2],8);break;
            case 0x55:  memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
            case 0x56:  memcpy(&stcPress,&ucRxBuffer[2],8);break;
            case 0x57:  memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
            case 0x58:  memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
        }


        ucRxCnt=0;
    }
}


//与摄像头通讯，获得目标点坐标
void CopeSerial3Data(unsigned char ucData)
{
    static unsigned char ucRxBuffer[250];
    static unsigned char ucRxCnt = 0;

    ucRxBuffer[ucRxCnt++]=ucData;
    if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
    {
        ucRxCnt=0;
        return;
    }
    if (ucRxCnt<4) {return;}//数据不满11个，则返回
    else
    {
        switch(ucRxBuffer[1])
        {
            case 0x50:  memcpy(&blobs,&ucRxBuffer[2],2);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据共同体里面，从而实现数据的解析。
        }


        ucRxCnt=0;
    }
}
//与小stm32板通讯，获取单足测距距离、三舵机电流
void CopeSerial4Data(unsigned char ucData)
{
    static unsigned char ucRxBuffer[250];
    static unsigned char ucRxCnt = 0;
    unsigned short distance;
    u16 adc1,adc2,adc3;

    ucRxBuffer[ucRxCnt++]=ucData;
    if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
    {
        ucRxCnt=0;
        return;
    }
    if (ucRxCnt<10) {return;}//数据不满10个，则返回
    else
    {
        switch(ucRxBuffer[1])
        {
            case 0x20 :
                distance=(ucRxBuffer[2]<<8)|ucRxBuffer[3];
                adc1=(ucRxBuffer[4]<<8)|ucRxBuffer[5];
                adc2=(ucRxBuffer[6]<<8)|ucRxBuffer[7];
                adc3=(ucRxBuffer[8]<<8)|ucRxBuffer[9];
                RF_s32.dist=distance;
                RF_s32.adc1=(float)adc1*(3.3/4096)*1000/3.0f;
                RF_s32.adc2=(float)adc2*(3.3/4096)*1000/3.0f;
                RF_s32.adc3=(float)adc3*(3.3/4096)*1000/3.0f;
                break;
            case 0x21 :
                distance=(ucRxBuffer[2]<<8)|ucRxBuffer[3];
                adc1=(ucRxBuffer[4]<<8)|ucRxBuffer[5];
                adc2=(ucRxBuffer[6]<<8)|ucRxBuffer[7];
                adc3=(ucRxBuffer[8]<<8)|ucRxBuffer[9];
                RM_s32.dist=distance;
                RM_s32.adc1=(float)adc1*(3.3/4096)*1000/3.0f;
                RM_s32.adc2=(float)adc2*(3.3/4096)*1000/3.0f;
                RM_s32.adc3=(float)adc3*(3.3/4096)*1000/3.0f;
                break;
            case 0x22 :
                distance=(ucRxBuffer[2]<<8)|ucRxBuffer[3];
                adc1=(ucRxBuffer[4]<<8)|ucRxBuffer[5];
                adc2=(ucRxBuffer[6]<<8)|ucRxBuffer[7];
                adc3=(ucRxBuffer[8]<<8)|ucRxBuffer[9];
                RB_s32.dist=distance;
                RB_s32.adc1=(float)adc1*(3.3/4096)*1000/3.0f;
                RB_s32.adc2=(float)adc2*(3.3/4096)*1000/3.0f;
                RB_s32.adc3=(float)adc3*(3.3/4096)*1000/3.0f;
                break;
            case 0x23 :
                distance=(ucRxBuffer[2]<<8)|ucRxBuffer[3];
                adc1=(ucRxBuffer[4]<<8)|ucRxBuffer[5];
                adc2=(ucRxBuffer[6]<<8)|ucRxBuffer[7];
                adc3=(ucRxBuffer[8]<<8)|ucRxBuffer[9];
                LF_s32.dist=distance;
                LF_s32.adc1=(float)adc1*(3.3/4096)*1000/3.0f;
                LF_s32.adc2=(float)adc2*(3.3/4096)*1000/3.0f;
                LF_s32.adc3=(float)adc3*(3.3/4096)*1000/3.0f;
                break;
            case 0x24 :
                distance=(ucRxBuffer[2]<<8)|ucRxBuffer[3];
                adc1=(ucRxBuffer[4]<<8)|ucRxBuffer[5];
                adc2=(ucRxBuffer[6]<<8)|ucRxBuffer[7];
                adc3=(ucRxBuffer[8]<<8)|ucRxBuffer[9];
                LM_s32.dist=distance;
                LM_s32.adc1=(float)adc1*(3.3/4096)*1000/3.0f;
                LM_s32.adc2=(float)adc2*(3.3/4096)*1000/3.0f;
                LM_s32.adc3=(float)adc3*(3.3/4096)*1000/3.0f;
                break;
            case 0x25 :
                distance=(ucRxBuffer[2]<<8)|ucRxBuffer[3];
                adc1=(ucRxBuffer[4]<<8)|ucRxBuffer[5];
                adc2=(ucRxBuffer[6]<<8)|ucRxBuffer[7];
                adc3=(ucRxBuffer[8]<<8)|ucRxBuffer[9];
                LB_s32.dist=distance;
                LB_s32.adc1=(float)adc1*(3.3/4096)*1000/3.0f;
                LB_s32.adc2=(float)adc2*(3.3/4096)*1000/3.0f;
                LB_s32.adc3=(float)adc3*(3.3/4096)*1000/3.0f;
                break;

        }

        ucRxCnt=0;
    }
}
//void USART2_IRQHandler(void)
//{

////    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
////  {
//      CopeSerial2Data((unsigned char)USART2->DR);//处理数据
////        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
////  }
//
//}
//解码激光雷达
void CopeSerial6Data(unsigned char ucData)
{
    static unsigned char ucRxBuffer[250];
    static unsigned char ucRxCnt = 0;
//  unsigned short distance;

    ucRxBuffer[ucRxCnt++]=ucData;
    if (!(ucRxBuffer[0]&0x01)^(ucRxBuffer[0]&0x02)>>1) //校验第一位
    {
        ucRxCnt=0;
        return;
    }
    if(ucRxCnt==2)
        {
        if (!(ucRxBuffer[1]&0x01))  //校验第二位
        {
            ucRxCnt=0;
            return;
        }
    }

    if (ucRxCnt<5) {return;}//数据不满10个，则返回
        else
        {
            ucRxCnt=0;
//          memcpy(&node_dis.,&ucRxBuffer[0],5);
        }

}

void USART2_IRQHandler(void)

{
    if(USART_GetITStatus (USART2,USART_IT_RXNE)!=RESET)

    {

        CopeSerial2Data((unsigned char)USART2->DR);//处理数据

    }

}


void USART3_IRQHandler(void)
{

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
  {

        CopeSerial3Data((unsigned char)USART3->DR);//处理数据
//      USART_ClearITPendingBit(USART3, USART_IT_RXNE);
  }

}

void UART4_IRQHandler(void)
{

    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
  {

        CopeSerial4Data((unsigned char)UART4->DR);//处理数据
//      USART_ClearITPendingBit(UART4, USART_IT_RXNE);
  }

}


void USART6_IRQHandler(void)
{

    if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
  {

        CopeSerial6Data((unsigned char)USART6->DR);
//      USART_ClearITPendingBit(UART4, USART_IT_RXNE);
  }

}


 void uartWriteBuf(uint8_t *buf, uint8_t len)
{
    while (len--) {
        while ((USART1->SR & 0x40) == 0);
        USART_SendData(USART1,*buf++);
    }
}

 extern uint8_t LobotRxBuf[16];

void USART1_IRQHandler(void)
{
    uint8_t Res;
    static bool isGotFrameHeader = false;
    static uint8_t frameHeaderCount = 0;
    static uint8_t dataLength = 2;
    static uint8_t dataCount = 0;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) { //判断接收中断
        Res = USART_ReceiveData(USART1);//(USART1->DR); //取出接收寄存器数据
        if (!isGotFrameHeader) {  //判断帧头
            if (Res == FRAME_HEADER) {
                frameHeaderCount++;
                if (frameHeaderCount == 2) {
                    frameHeaderCount = 0;
                    isGotFrameHeader = true;
                    dataCount = 1;
                }
            } else {
                isGotFrameHeader = false;
                dataCount = 0;
                frameHeaderCount = 0;
            }
        }
        if (isGotFrameHeader) { //接收接收数据部分
            UART_RX_BUF[dataCount] = Res;
            if (dataCount == 2) {
                dataLength = UART_RX_BUF[dataCount];
                if (dataLength < 2 || dataLength > 8) {
                    dataLength = 2;
                    isGotFrameHeader = false;
                }
            }
            dataCount++;
            if (dataCount == dataLength + 2) {
                if (isUartRxCompleted == false) {
                    isUartRxCompleted = true;
                    memcpy(LobotRxBuf, UART_RX_BUF, dataCount);
                }
                isGotFrameHeader = false;
            }
        }
    }
}





















#endif





