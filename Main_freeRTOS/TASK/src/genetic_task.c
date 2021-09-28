#include "genetic_task.h"
#include "FreeRTOS.h"
#include "action_task_723.h"
#include "task.h"
// genetic������

int genetic_mode;
int genetic_start = 0;
void genetic_task(void* pvParameters) {
    while (1) {
        int i;
        while (STA_STOP == 1)
            vTaskDelay(30);
        while (genetic_start == 0)
            vTaskDelay(30);
        initiate();  //������ʼ����Ⱥ
        evaluation(0, genetic_mode);  //�Գ�ʼ����Ⱥ��������������

        for (i = 0; i < MAXloop; i++) {
            while (STA_STOP == 1)
                vTaskDelay(30);
            cross();  //���н������
            evaluation(1, genetic_mode);  //������Ⱥ��������������
            mutation(genetic_mode);  //�������
            selection();  //�Ը�����Ⱥ��ѡ�����ŵ�NUM����Ϊ�µĸ���Ⱥ
            if (record() == 1)  //������ֹ����1����flag=1��ֹͣѭ��
            {
                break;
            }
        }

        while (1)
            vTaskDelay(30);
    }
}
