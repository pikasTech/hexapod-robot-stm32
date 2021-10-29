#include "rtp_test_task.h"
#include "action_task.h"
#include "key.h"
#include "lcd.h"
#include "touch.h"

extern int point_x;
extern int point_y;
extern int point_y_old;
extern int test[];
extern u8 shiboqi_ch;
extern int tp_test_x;
extern int tp_test_y;
// rtp_test任务函数
void rtp_test_task(void* pvParameters) {
    while (1) {
        volatile u8 key;
        point_x = 250, point_y = 50, point_y_old = 50;
        //  LCD_ShowxNum(tp_test_x,tp_test_y,action[action_n][0][0],4,12,16);
        LCD_ShowxNum(70, 130, test[0], 4, 12, 16);
        LCD_ShowxNum(40, 130, test[1], 4, 12, 16);
        LCD_ShowxNum(10, 130, test[2], 4, 12, 16);
        LCD_ShowxNum(70, 200, test[3], 4, 12, 16);
        LCD_ShowxNum(40, 200, test[4], 4, 12, 16);
        LCD_ShowxNum(10, 200, test[5], 4, 12, 16);
        LCD_ShowxNum(70, 270, test[6], 4, 12, 16);
        LCD_ShowxNum(40, 270, test[7], 4, 12, 16);
        LCD_ShowxNum(10, 270, test[8], 4, 12, 16);
        LCD_ShowxNum(190, 130, test[11], 4, 12, 16);
        LCD_ShowxNum(150, 130, test[10], 4, 12, 16);
        LCD_ShowxNum(110, 130, test[9], 4, 12, 16);
        LCD_ShowxNum(190, 200, test[14], 4, 12, 16);
        LCD_ShowxNum(150, 200, test[13], 4, 12, 16);
        LCD_ShowxNum(110, 200, test[12], 4, 12, 16);
        LCD_ShowxNum(190, 270, test[17], 4, 12, 16);
        LCD_ShowxNum(150, 270, test[16], 4, 12, 16);
        LCD_ShowxNum(110, 270, test[15], 4, 12, 16);

        LCD_ShowxNum(10, 0, shiboqi_ch, 4, 16, 16);
        LCD_ShowString(10, 0, 200, 16, 16, "ch");

        key = KEY_Scan(0);
        tp_dev.scan(0);
        if (tp_dev.sta & TP_PRES_DOWN)  //触摸屏被按下
        {
            if (tp_dev.x[0] < lcddev.width && tp_dev.y[0] < lcddev.height) {
                if (tp_dev.x[0] > (lcddev.width - 24) && tp_dev.y[0] < 16)
                    Load_Drow_Dialog();  //清除

                else if ((tp_dev.x[0] > 74) && (tp_dev.x[0] < 117) &&
                         (tp_dev.y[0] < 165) && (tp_dev.y[0] > 113))  //按键
                {
                    shiboqi_ch = 0;
                } else if ((tp_dev.x[0] > 34) && (tp_dev.x[0] < 72) &&
                           (tp_dev.y[0] < 165) && (tp_dev.y[0] > 117))  //按键
                {
                    shiboqi_ch = 1;
                } else if ((tp_dev.x[0] > 0) && (tp_dev.x[0] < 34) &&
                           (tp_dev.y[0] < 165) && (tp_dev.y[0] > 117))  //按键
                {
                    shiboqi_ch = 2;
                } else if ((tp_dev.x[0] > 0) && (tp_dev.x[0] < 34) &&
                           (tp_dev.y[0] < 240) && (tp_dev.y[0] > 165))  //按键
                {
                    shiboqi_ch = 5;
                } else if ((tp_dev.x[0] > 34) && (tp_dev.x[0] < 72) &&
                           (tp_dev.y[0] < 240) && (tp_dev.y[0] > 165))  //按键
                {
                    shiboqi_ch = 4;
                } else if ((tp_dev.x[0] > 72) && (tp_dev.x[0] < 117) &&
                           (tp_dev.y[0] < 240) && (tp_dev.y[0] > 165))  //按键
                {
                    shiboqi_ch = 3;
                } else if ((tp_dev.x[0] > 77) && (tp_dev.x[0] < 119) &&
                           (tp_dev.y[0] < 311) && (tp_dev.y[0] > 238))  //按键
                {
                    shiboqi_ch = 6;
                } else if ((tp_dev.x[0] > 34) && (tp_dev.x[0] < 77) &&
                           (tp_dev.y[0] < 311) && (tp_dev.y[0] > 238))  //按键
                {
                    shiboqi_ch = 7;
                } else if ((tp_dev.x[0] > 0) && (tp_dev.x[0] < 34) &&
                           (tp_dev.y[0] < 311) && (tp_dev.y[0] > 238))  //按键
                {
                    shiboqi_ch = 8;
                } else if ((tp_dev.x[0] > 121) && (tp_dev.x[0] < 158) &&
                           (tp_dev.y[0] < 165) && (tp_dev.y[0] > 117))  //按键
                {
                    shiboqi_ch = 9;
                } else if ((tp_dev.x[0] > 158) && (tp_dev.x[0] < 201) &&
                           (tp_dev.y[0] < 165) && (tp_dev.y[0] > 117))  //按键
                {
                    shiboqi_ch = 10;
                } else if ((tp_dev.x[0] > 201) && (tp_dev.x[0] < 236) &&
                           (tp_dev.y[0] < 165) && (tp_dev.y[0] > 117))  //按键
                {
                    shiboqi_ch = 11;
                } else if ((tp_dev.x[0] > 121) && (tp_dev.x[0] < 158) &&
                           (tp_dev.y[0] < 240) && (tp_dev.y[0] > 165))  //按键
                {
                    shiboqi_ch = 12;
                } else if ((tp_dev.x[0] > 158) && (tp_dev.x[0] < 201) &&
                           (tp_dev.y[0] < 240) && (tp_dev.y[0] > 165))  //按键
                {
                    shiboqi_ch = 13;
                } else if ((tp_dev.x[0] > 201) && (tp_dev.x[0] < 236) &&
                           (tp_dev.y[0] < 240) && (tp_dev.y[0] > 165))  //按键
                {
                    shiboqi_ch = 14;
                } else if ((tp_dev.x[0] > 121) && (tp_dev.x[0] < 158) &&
                           (tp_dev.y[0] < 311) && (tp_dev.y[0] > 238))  //按键
                {
                    shiboqi_ch = 15;
                } else if ((tp_dev.x[0] > 158) && (tp_dev.x[0] < 201) &&
                           (tp_dev.y[0] < 311) && (tp_dev.y[0] > 238))  //按键
                {
                    shiboqi_ch = 16;
                } else if ((tp_dev.x[0] > 201) && (tp_dev.x[0] < 236) &&
                           (tp_dev.y[0] < 311) && (tp_dev.y[0] > 238))  //按键
                {
                    //              shiboqi_ch=17;
                    // 1595 ,1000,2500 ,     1667 ,881 ,571 ,     1500 ,857
                    // ,2357 ,     1667 ,1095 ,2381 ,     1500 ,1071 ,2429 ,
                    // 1333 ,1143 ,500,
                    //                  get_action_from_list(0);
                    //                  move_single(1000);
                }

                tp_test_x = tp_dev.x[0], tp_test_y = tp_dev.y[0];
            }
        }
    }
}
