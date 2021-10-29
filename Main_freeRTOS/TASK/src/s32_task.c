#include "s32_task.h"
#include "FreeRTOS.h"
#include "adc.h"
#include "task.h"
// s32任务函数
//该任务为和s32进行通信
// 0x25可用

float ave_rate = 0.1f;
int ave_num = 10;
void s32_task(void* pvParameters) {
    while (1) {
        u8 i;
        float ADC_ave;
        //      u16 ADC_sum;
        u16 ADCX;
        //      float ADC_ave_nom;
        //      for (i=0;i<6;i++)
        //      {
        //          fputc4(0x55);
        //          fputc4(0x25-i);
        //          ////printf("[%f,%f,%d,%d,%d,%d]\r\n",Roll_use,Pitch_use,RF_s32.dist,RF_s32.adc1,RF_s32.adc2,RF_s32.adc3);

        //      }

        //          ADC_sum = 0;

        for (i = 0; i < ave_num; i++) {
            //              while(!DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
            //              //等待DMA2_Steam0转换完成
            //              {

            //              }
            //              ADCX = ADC_ConvertedValue[0];
            //              ADC_sum += ADCX;
        }
        //          ADC_ave_nom = ADC_sum/(float)ave_num;
        ADC_ave = ADC_ave * (1 - ave_rate) + ADCX * (ave_rate);
        //          printf("[%f,%f]\r\n",ADC_ave_nom - 400.0f,T06_RF[2]+10.0f);
        vTaskDelay(30);
        //          fputc6(0xA5);
        //          fputc6(0x20);
        //          while(1)vTaskDelay(30);
    }
}
