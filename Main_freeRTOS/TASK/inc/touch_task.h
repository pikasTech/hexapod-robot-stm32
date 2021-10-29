#ifndef __touch_TASK_H
#define __touch_TASK_H
#include "sys.h"

#define  ChuDi_rf  touch.rf == 0
#define  ChuDi_rm  touch.rm == 0
#define  ChuDi_rb  touch.rb == 0
#define  ChuDi_lf  touch.lf == 0
#define  ChuDi_lm  touch.lm == 0
#define  ChuDi_lb  touch.lb == 0


#define USE_STA_ChuDi 0
#define USE_STA_touch_ChuDi 0
extern u8 STA_ChuDi_rm;
extern u8 STA_ChuDi_lf;
extern u8 STA_ChuDi_lb;
extern u8 STA_ChuDi_lm;
extern u8 STA_ChuDi_rf;
extern u8 STA_ChuDi_rb;
struct TOUCH
{
    u8 rf;
    u8 rm;
    u8 rb;
    u8 lf;
    u8 lm;
    u8 lb;
};
extern struct TOUCH touch_use;

void get_touch(struct TOUCH *touch_fun);
void cheak_STA_ChuDi(void);

#endif

