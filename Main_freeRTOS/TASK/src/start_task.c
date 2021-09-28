#include "start_task.h"
#include "FreeRTOS.h"
#include "task.h"
extern TaskHandle_t StartTask_Handler;

//���������

//�������ȼ�
#define systime_TASK_PRIO 5
//�����ջ��С
#define systime_STK_SIZE 256
//������
TaskHandle_t systimeTask_Handler;
//������
void systime_task(void* pvParameters);

//�������ȼ�
#define angle_TASK_PRIO 4
//�����ջ��С
#define angle_STK_SIZE 256
//������
TaskHandle_t angleTask_Handler;
//������
void angle_task(void* pvParameters);

//�������ȼ�
#define action_TASK_PRIO 4
//�����ջ��С
#define action_STK_SIZE 256
//������
TaskHandle_t actionTask_Handler;
//������
void action_task(void* pvParameters);

//		//�������ȼ�
//		#define adc_TASK_PRIO		5
//		//�����ջ��С
//		#define adc_STK_SIZE 		128
//		//������
//		TaskHandle_t adcTask_Handler;
//		//������
//		void adc_task(void *pvParameters);
//
//�������ȼ�
#define ps2_TASK_PRIO 3
//�����ջ��С
#define ps2_STK_SIZE 256
//������
TaskHandle_t ps2Task_Handler;
//������
void ps2_task(void* pvParameters);

//�������ȼ�
#define s32_TASK_PRIO 3
//�����ջ��С
#define s32_STK_SIZE 256
//������
TaskHandle_t s32Task_Handler;
//������
void s32_task(void* pvParameters);

//�������ȼ�
#define s32high_TASK_PRIO 3
//�����ջ��С
#define s32high_STK_SIZE 256
//������
TaskHandle_t s32highTask_Handler;
//������
void s32high_task(void* pvParameters);

//		//�������ȼ�
//		#define shiboqi_TASK_PRIO		4
//		//�����ջ��С
//		#define shiboqi_STK_SIZE 		128
//		//������
//		TaskHandle_t shiboqiTask_Handler;
//		//������
//		void shiboqi_task(void *pvParameters);

//		//�������ȼ�
//		#define sanjiaobo_TASK_PRIO		3
//		//�����ջ��С
//		#define sanjiaobo_STK_SIZE 		128
//		//������
//		TaskHandle_t sanjiaoboTask_Handler;
//		//������
//		void sanjiaobo_task(void *pvParameters);
//
//		//�������ȼ�
//		#define rtp_test_TASK_PRIO		2
//		//�����ջ��С
//		#define rtp_test_STK_SIZE 		128
//		//������
//		TaskHandle_t rtp_testTask_Handler;
//		//������
//		void rtp_test_task(void *pvParameters);

//�������ȼ�
#define banlance_TASK_PRIO 1
//�����ջ��С
#define banlance_STK_SIZE 256
//������
TaskHandle_t banlanceTask_Handler;
//������
void banlance_task(void* pvParameters);

//�������ȼ�
#define blobs_TASK_PRIO 2
//�����ջ��С
#define blobs_STK_SIZE 256
//������
TaskHandle_t blobsTask_Handler;
//������
void blobs_task(void* pvParameters);

//�������ȼ�
#define genetic_TASK_PRIO 1
//�����ջ��С
#define genetic_STK_SIZE 256
//������
TaskHandle_t geneticTask_Handler;
//������
void genetic_task(void* pvParameters);

//�������ȼ�
#define touch_TASK_PRIO 1
//�����ջ��С
#define touch_STK_SIZE 256
//������
TaskHandle_t touchTask_Handler;
//������
void touch_task(void* pvParameters);

//�������ȼ�
#define simulization_TASK_PRIO 1
//�����ջ��С
#define simulization_STK_SIZE 256
//������
TaskHandle_t simulizationTask_Handler;
//������
void simulization_task(void* pvParameters);

struct Task_Enable_Flag_Type {
    u8 action_task_flag;
    u8 banlance_task_flag;
    u8 angle_task_flag;
    u8 systime_task_flag;
    u8 ps2_task_flag;
    u8 s32_task_flag;
    u8 shiboqi_task_flag;
    u8 rtp_test_task_flag;
    u8 sanjiaobo_task_flag;
    u8 adc_task_flag;
    u8 blobs_task_flag;
    u8 s32high_task_flag;
    u8 genetic_task_flag;
    u8 touch_task_flag;
    u8 simulization_task_flag;
} task_enable_flag = {
    /*action*/ 1,
    /*banlance*/ 1,
    /*angle*/ 1,
    /*systime*/ 1,
    /*ps2*/ 0,
    /*s32*/ 0,
    /*shiboqi*/ 0,
    /*rtp_test*/ 0,
    /*sanjiaobo*/ 0,
    /*adc*/ 0,
    /*blobs*/ 0,
    /*s32high*/ 0,
    /*genetic*/ 0,
    /*touch*/ 1,
    /*simulazation*/ 1};

//��ʼ������
void start_task(void* pvParameters) {
    taskENTER_CRITICAL();  //�����ٽ���
    if (task_enable_flag.action_task_flag) {
        //����action����
        xTaskCreate((TaskFunction_t)action_task, (const char*)"action_task",
                    (uint16_t)action_STK_SIZE, (void*)NULL,
                    (UBaseType_t)action_TASK_PRIO,
                    (TaskHandle_t*)&actionTask_Handler);
    }
    if (task_enable_flag.banlance_task_flag) {  //����banlance����
        xTaskCreate((TaskFunction_t)banlance_task, (const char*)"banlance_task",
                    (uint16_t)banlance_STK_SIZE, (void*)NULL,
                    (UBaseType_t)banlance_TASK_PRIO,
                    (TaskHandle_t*)&banlanceTask_Handler);
    }
    if (task_enable_flag.angle_task_flag) {
        //����angle����
        xTaskCreate((TaskFunction_t)angle_task, (const char*)"angle_task",
                    (uint16_t)angle_STK_SIZE, (void*)NULL,
                    (UBaseType_t)angle_TASK_PRIO,
                    (TaskHandle_t*)&angleTask_Handler);
    }
    if (task_enable_flag.systime_task_flag) {
        //����systime����
        xTaskCreate((TaskFunction_t)systime_task, (const char*)"systime_task",
                    (uint16_t)systime_STK_SIZE, (void*)NULL,
                    (UBaseType_t)systime_TASK_PRIO,
                    (TaskHandle_t*)&systimeTask_Handler);
    }
    if (task_enable_flag.ps2_task_flag) {
        //����ps2����
        xTaskCreate((TaskFunction_t)ps2_task, (const char*)"ps2_task",
                    (uint16_t)ps2_STK_SIZE, (void*)NULL,
                    (UBaseType_t)ps2_TASK_PRIO,
                    (TaskHandle_t*)&ps2Task_Handler);
    }
    if (task_enable_flag.s32_task_flag) {
        //����s32����
        xTaskCreate((TaskFunction_t)s32_task, (const char*)"s32_task",
                    (uint16_t)s32_STK_SIZE, (void*)NULL,
                    (UBaseType_t)s32_TASK_PRIO,
                    (TaskHandle_t*)&s32Task_Handler);
    }

    if (task_enable_flag.blobs_task_flag) {
        //����blobs����
        xTaskCreate((TaskFunction_t)blobs_task, (const char*)"blobs_task",
                    (uint16_t)blobs_STK_SIZE, (void*)NULL,
                    (UBaseType_t)blobs_TASK_PRIO,
                    (TaskHandle_t*)&blobsTask_Handler);
    }

    if (task_enable_flag.s32high_task_flag) {
        //����s32high����
        xTaskCreate((TaskFunction_t)s32high_task, (const char*)"s32high_task",
                    (uint16_t)s32_STK_SIZE, (void*)NULL,
                    (UBaseType_t)s32_TASK_PRIO,
                    (TaskHandle_t*)&s32Task_Handler);
    }

    if (task_enable_flag.genetic_task_flag) {
        //����genetic����
        xTaskCreate((TaskFunction_t)genetic_task, (const char*)"genetic_task",
                    (uint16_t)genetic_STK_SIZE, (void*)NULL,
                    (UBaseType_t)genetic_TASK_PRIO,
                    (TaskHandle_t*)&geneticTask_Handler);
    }

    if (task_enable_flag.touch_task_flag) {
        //����touch����
        xTaskCreate((TaskFunction_t)touch_task, (const char*)"touch_task",
                    (uint16_t)touch_STK_SIZE, (void*)NULL,
                    (UBaseType_t)touch_TASK_PRIO,
                    (TaskHandle_t*)&touchTask_Handler);
    }

    if (task_enable_flag.simulization_task_flag) {
        //����simulization����
        xTaskCreate((TaskFunction_t)simulization_task,
                    (const char*)"simulization_task",
                    (uint16_t)simulization_STK_SIZE, (void*)NULL,
                    (UBaseType_t)simulization_TASK_PRIO,
                    (TaskHandle_t*)&simulizationTask_Handler);
    }

    vTaskDelete(StartTask_Handler);  //ɾ����ʼ����
    taskEXIT_CRITICAL();             //�˳��ٽ���
}

/*
if (task_enable_flag.shiboqi_task_flag)
{
    //����shiboqi����
    xTaskCreate((TaskFunction_t )shiboqi_task,
                (const char*    )"shiboqi_task",
                (uint16_t       )shiboqi_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )shiboqi_TASK_PRIO,
                (TaskHandle_t*  )&shiboqiTask_Handler);
}
if (task_enable_flag.rtp_test_task_flag)
{
    //����rtp_test����
    xTaskCreate((TaskFunction_t )rtp_test_task,
                (const char*    )"rtp_test_task",
                (uint16_t       )rtp_test_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )rtp_test_TASK_PRIO,
                (TaskHandle_t*  )&rtp_testTask_Handler);
}
if (task_enable_flag.sanjiaobo_task_flag)
{
                //����sanjiaobo����
    xTaskCreate((TaskFunction_t )sanjiaobo_task,
                (const char*    )"sanjiaobo_task",
                (uint16_t       )sanjiaobo_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )sanjiaobo_TASK_PRIO,
                (TaskHandle_t*  )&sanjiaoboTask_Handler);
}
if (task_enable_flag.adc_task_flag)
{
                //����adc����
    xTaskCreate((TaskFunction_t )adc_task,
                (const char*    )"adc_task",
                (uint16_t       )adc_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )adc_TASK_PRIO,
                (TaskHandle_t*  )&adcTask_Handler);
}
*/
