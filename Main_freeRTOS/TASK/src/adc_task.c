#include "adc_task.h"
#include "FreeRTOS.h"
#include "action_task.h"
#include "adc.h"
#include "current.h"
#include "task.h"
extern int current_single;
extern int test_point[];
extern u16 adcx;
extern __IO uint16_t ADC_ConvertedValue[];
int valuel_max_normal[6] = {118, 2045, 891, 1118, 2947, 706};
u8 flag_action_normal[6] = {0};
int value1_max[6] = {0};
// adc任务函数
void adc_task(void* pvParameters) {
    while (1) {
        int j;
        int i;
        int temp_val1 = 0;
        for (i = 0; i < 6; i++) {
            for (j = 0; j < 30; j++) {
                // temp_val1+=
                // //某个通道的电压值=DMA通道拿到的那个数组里面的值就是电压值
                temp_val1 += ADC_ConvertedValue[i];
                vTaskDelay(1);
            }
            test_point[i] = temp_val1;
            if (value1_max[i] <= temp_val1)
                value1_max[i] = temp_val1;
            temp_val1 = 0;
        }

        for (i = 0; i < 6; i++) {
            if ((test_point[i]) < 150) {
                flag_action_normal[i] = 1;
            }
            if ((test_point[i]) > 150) {
                flag_action_normal[i] = 0;
            }
        }
    }
}
