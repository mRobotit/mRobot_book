#ifndef __ADC_H
#define __ADC_H	
#include "stm32f10x.h"


void Adc_Init(void);
u16  Get_Adc(u8 ch); 
float Get_Valtage();
 
#endif 
