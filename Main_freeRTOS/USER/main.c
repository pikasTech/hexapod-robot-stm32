
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "lcd.h"
#include "FreeRTOS.h"
#include "task.h"
#include "key.h" 
#include "touch.h" 
#include "adc.h"
#include "current.h"
#include "mpuiic.h"
#include "LobotServoController.h"
//task
#include "adc_task.h"
#include "rtp_test_task.h"
#include "sanjiaobo_task.h"
#include "shiboqi_task.h"
#include "start_task.h"
#include "balance_task.h"
// #include "mpu6050.h"
#include "pstwo.h"
#include "ps2_task.h"
#include "action_task_723.h"



//全局变量定义
		int flag_hanshu1,flag_hanshu2=0;
	  int point_x,point_y,point_y_old;
		int hanshu=0;
		int test_x,test_y;
		int date_y[241];
		int test_point[18]={88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88};
		int test_ch;
		int n;
		int i_date_y;
		int num_x;
		int num_y;
		int tp_test_x;
		int tp_test_y;
		u8 shiboqi_ch;
		u16 adcx;
		u8 Way_Angle=1;
		float Angle_Balance,Gyro_Balance,Gyro_Turn,Gyro_Pitch,Gyro_Roll; //平衡倾角 平衡陀螺仪 转向陀螺仪
		
		//电流数组
		float current[18];
		float current_single;

//任务优先级
		#define START_TASK_PRIO		1
		//任务堆栈大小	
		#define START_STK_SIZE 		512  
		//任务句柄
		TaskHandle_t StartTask_Handler;
		//任务函数
		void start_task(void *pvParameters);


int main(void)
{ 

/*	接线表：
	PA0		——	LED1
	PA1		——		
	PA2		——	RX_陀螺仪
	PA3		——	TX_陀螺仪
	PA4		——
	PA5		——
	PA6		——
	PA7		——
	PA8		——	
	PA9		——	RX_舵机控制器
	PA10	——	TX_舵机控制器
	PA11	——
	PA12	——
	PA13	——	CLK_SWD
	PA14	——	DIO_SWD
	PA15	——

	PB0		——
	PB1		——
	PB2		——
	PB3		——
	PB4		——
	PB5		——
	PB6		——
	PB7		——
	PB8		——		
	PB9		——
	PB10	——	RX_摄像头
	PB11	——	TX_摄像头
	PB12	——	CLK_PS2手柄接收器
	PB13	——	CS_PS2手柄接收器
	PB14	——	DO_PS2手柄接收器
	PB15	——	DI_PS2手柄接收器


	PC10	——	RX_小32从机
	PC11	——	TX_小32从机
	
	PC6		——	RX_PC
	PC7		——	TX_PC
	
*/

/*

	任务快捷入口						右键—>Go To Defination of ‘XXXXX’
		start_task					任务创建
		task_enable_flag		任务开关
		action_task					多足协调控制
		banlance_task				运动学逆解器
		angle_task					姿态闭环控制
		systime_task				系统时间
		ps2_task						手柄
		s32_task						请求小stm32的数据
		genetic_task				遗传算法
		touch_task					触底反馈
	通讯快捷入口
		CopeSerial2Data 		与陀螺仪通讯，获得欧拉角和角速度
		CopeSerial3Data			与摄像头通讯，获得目标点坐标
		CopeSerial4Data			与小stm32板通讯，获取单足测距距离、三舵机电流
		USART1_IRQHandler		串口1接收
		CopeSerial6Data			激光雷达
*/

//初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	delay_init(168);					//初始化延时函数
	uart1_init(9600);      				//初始化串口
	uart2_init(115200);
	uart3_init(115200);
	uart4_init(9600);
	uart6_init(115200); 
	LED_Init();		        			//初始化LED端口
	PS2_Init();						//PS2手柄初始化
	PS2_SetInit();			 //配配置初始化,配置“红绿灯模式”，并选择是否可以修改
	T06_init();					//初始化末端位置
	T06_init_ideal();		//初始化末端位置理想值
	KEY_EXTI_Init();
	action_init();

#if 1
	//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
#endif
								
		while(1)
		{
//			adcx = Get_Adc(0);
		}

	
}



