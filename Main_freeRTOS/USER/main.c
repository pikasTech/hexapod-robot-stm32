
#include "FreeRTOS.h"
#include "LobotServoController.h"
#include "adc.h"
#include "current.h"
#include "delay.h"
#include "key.h"
#include "lcd.h"
#include "led.h"
#include "mpuiic.h"
#include "sys.h"
#include "task.h"
#include "timer.h"
#include "touch.h"
#include "usart.h"
// task
#include "adc_task.h"
#include "balance_task.h"
#include "rtp_test_task.h"
#include "sanjiaobo_task.h"
#include "shiboqi_task.h"
#include "start_task.h"
// #include "mpu6050.h"
#include "action_task_723.h"
#include "ps2_task.h"
#include "pstwo.h"

//ȫ�ֱ�������
int flag_hanshu1, flag_hanshu2 = 0;
int point_x, point_y, point_y_old;
int hanshu = 0;
int test_x, test_y;
int date_y[241];
int test_point[18] = {88, 88, 88, 88, 88, 88, 88, 88, 88,
                      88, 88, 88, 88, 88, 88, 88, 88, 88};
int test_ch;
int n;
int i_date_y;
int num_x;
int num_y;
int tp_test_x;
int tp_test_y;
u8 shiboqi_ch;
u16 adcx;
u8 Way_Angle = 1;
float Angle_Balance, Gyro_Balance, Gyro_Turn, Gyro_Pitch,
    Gyro_Roll;  //ƽ����� ƽ�������� ת��������

//��������
float current[18];
float current_single;

//�������ȼ�
#define START_TASK_PRIO 1
//�����ջ��С
#define START_STK_SIZE 512
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void* pvParameters);

int main(void) {
    /*	���߱���
            PA0		����	LED1
            PA1		����
            PA2		����	RX_������
            PA3		����	TX_������
            PA4		����
            PA5		����
            PA6		����
            PA7		����
            PA8		����
            PA9		����	RX_���������
            PA10	����	TX_���������
            PA11	����
            PA12	����
            PA13	����	CLK_SWD
            PA14	����	DIO_SWD
            PA15	����

            PB0		����
            PB1		����
            PB2		����
            PB3		����
            PB4		����
            PB5		����
            PB6		����
            PB7		����
            PB8		����
            PB9		����
            PB10	����	RX_����ͷ
            PB11	����	TX_����ͷ
            PB12	����	CLK_PS2�ֱ�������
            PB13	����	CS_PS2�ֱ�������
            PB14	����	DO_PS2�ֱ�������
            PB15	����	DI_PS2�ֱ�������


            PC10	����	RX_С32�ӻ�
            PC11	����	TX_С32�ӻ�

            PC6		����	RX_PC
            PC7		����	TX_PC

    */

    /*

            ���������						�Ҽ���>Go To Defination of
       ��XXXXX�� start_task					���񴴽�
                    task_enable_flag		���񿪹�
                    action_task					����Э������
                    banlance_task				�˶�ѧ�����
                    angle_task					��̬�ջ�����
                    systime_task				ϵͳʱ��
                    ps2_task						�ֱ�
                    s32_task
       ����Сstm32������ genetic_task				�Ŵ��㷨
                    touch_task ���׷��� ͨѶ������ CopeSerial2Data
       ��������ͨѶ�����ŷ���Ǻͽ��ٶ� CopeSerial3Data
       ������ͷͨѶ�����Ŀ������� CopeSerial4Data
       ��Сstm32��ͨѶ����ȡ��������롢��������� USART1_IRQHandler ����1����
                    CopeSerial6Data			�����״�
    */

    //��ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //����ϵͳ�ж����ȼ�����4
    delay_init(168);   //��ʼ����ʱ����
    uart1_init(9600);  //��ʼ������
    uart2_init(115200);
    uart3_init(115200);
    uart4_init(9600);
    uart6_init(115200);
    LED_Init();     //��ʼ��LED�˿�
    PS2_Init();     // PS2�ֱ���ʼ��
    PS2_SetInit();  //�����ó�ʼ��,���á����̵�ģʽ������ѡ���Ƿ�����޸�
    T06_init();     //��ʼ��ĩ��λ��
    T06_init_ideal();  //��ʼ��ĩ��λ������ֵ
    KEY_EXTI_Init();
    action_init();

#if 1
    //������ʼ����
    xTaskCreate((TaskFunction_t)start_task,  //������
                (const char*)"start_task",   //��������
                (uint16_t)START_STK_SIZE,    //�����ջ��С
                (void*)NULL,  //���ݸ��������Ĳ���
                (UBaseType_t)START_TASK_PRIO,  //�������ȼ�
                (TaskHandle_t*)&StartTask_Handler);  //������
    vTaskStartScheduler();  //�����������
#endif

    while (1) {
        //			adcx = Get_Adc(0);
    }
}
