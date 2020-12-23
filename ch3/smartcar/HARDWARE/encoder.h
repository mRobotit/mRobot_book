#include "stm32f10x.h"

#define  ENCODER_COUNT_VALUE	   60000.0f
void Left_Encoder_Init(void);

void Right_Encoder_Init(void);

void TIM3_IRQHandler(void);

void TIM4_IRQHandler(void);

void Print_MotorSpeed();

void  Robot_Encoder_Get_CNT(void);