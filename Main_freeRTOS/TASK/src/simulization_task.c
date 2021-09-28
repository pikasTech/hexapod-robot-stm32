#include "simulization_task.h"
#include "FreeRTOS.h"
#include "action_task_723.h"
#include "balance_task.h"
#include "sys.h"
#include "task.h"
// simulization������
void simulization_task(void* pvParameters) {
    while (1) {
        STA_STOP = 0;
        flag_action_run_continiue = 1;
        Order[0] = 4;
        Order[1] = 5;
        Order[2] = 6;
        vTaskDelay(10);
    }
}
