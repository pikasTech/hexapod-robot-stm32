#ifndef __current_H
#define __current_H
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "adc.h"

#define A0_1 PFout(0)   //74LS138_1
#define A1_1 PFout(1)
#define A2_1 PFout(2)

#define A0_2 PFout(3)   //74LS138_2
#define A1_2 PFout(4)
#define A2_2 PFout(5)

#define CD4053_a PFout(6)   //CD4053
#define CD4053_b PFout(7)
#define CD4053_c PFout(8)

float get_current_init(void);
float get_current(u8 ch,float current_val);
void get_All_current(float current_val);
void ch_74LS138(u8 ch);
#endif

