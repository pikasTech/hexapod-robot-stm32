#include "shiboqi_task.h"
// shiboqi������
void shiboqi_task(void* pvParameters) {
    int i;
    while (1) {
        POINT_COLOR = RED;
        point_y = 100 - (test_point[shiboqi_ch]) / 10;
        LCD_Fast_DrawPoint(240, date_y[240], BACK_COLOR);
        LCD_Fast_DrawPoint(0, date_y[0], BACK_COLOR);
        date_y[240] = point_y;
        for (i = 0; i <= 239; i++) {
            date_y[i] = date_y[i + 1];
            LCD_Fast_DrawPoint(i + 1, date_y[i + 1], BACK_COLOR);
            LCD_Fast_DrawPoint(i, date_y[i], POINT_COLOR);
        }
        LCD_Fast_DrawPoint(240, date_y[240], POINT_COLOR);
        vTaskDelay(50);
    }
}

// void shiboqi_task(void *pvParameters)
//{
//	int i;
//	while(1)
//	{
//		POINT_COLOR=RED;
//		point_y=100-test_point[shiboqi_ch];
//		LCD_Fast_DrawPoint(240,date_y[240],BACK_COLOR);
//		LCD_Fast_DrawPoint(0,date_y[0],BACK_COLOR);
//		date_y[240]=point_y;
//		for(i=0;i<=239;i++)
//		{
//			date_y[i]=date_y[i+1];
//			LCD_Fast_DrawPoint(i+1,date_y[i+1],BACK_COLOR);
//			LCD_Fast_DrawPoint(i,date_y[i],POINT_COLOR);
//		}
//		LCD_Fast_DrawPoint(240,date_y[240],POINT_COLOR);
//			vTaskDelay(2);
//	}

//}
