#include "systime_task.h"
#include "FreeRTOS.h"
#include "action_task_723.h"
#include "balance_task.h"
#include "sys.h"
#include "task.h"
// systime������
long int systime = 0;
void systime_task(void* pvParameters) {
    while (1) {
        //ϵͳʱ�䣨��λms��
        if (STA_TIME_STOP == 0)
            systime++;
        vTaskDelay(1);
        if (systime % 100 == 0) {
            //			printf("%f\t%f\r\n",Roll_use,Pitch_use);
            printf("A %f\r\n", Roll_use);
            printf("B %f\r\n", Pitch_use);

            LED1 = ~LED1;
        }
    }
}
