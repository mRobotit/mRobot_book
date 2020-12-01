#ifndef _DELAY__H_
#define _DELAY__H_

#include "stm32f10x.h"

//void delay_us(u32 n);
void delayTime(u32 times);

void delay_us(u32 nus);
void delay_ms(u16 nms);
void delay_S(u16 ns);
void get_ms(unsigned long *time);
void SysTick_Init(void);

//void Delay_us(__IO u32 nTime);
#endif
