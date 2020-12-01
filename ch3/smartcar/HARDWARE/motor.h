
#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

typedef struct _Moto_
{
	int Encoder_Value;
	float Current_Speed;
	float Target_Speed;
	short ESC_Output_PWM;
	float L_Error;
	float LL_Error;
}Moto_Info;


extern Moto_Info Left_moto;
extern Moto_Info Right_moto;
	

#define ESC_OUTPUT_PWM_LIMT 800
#define CONTROL_TIMER_CYCLE			0.05f	
#define Pi						3.1415f
#define Base_Width					0.155f
#define ROBOT_INITIATIVE_DIAMETER_L 0.065f
#define ROBOT_INITIATIVE_DIAMETER_R 0.065f

#define MOTO_LEFT		1
#define MOTO_RIGHT		2

void TIM2_PWM_Init(void);

void MOTOR_GPIO_Config(void);

void PWM_OUTPUT(int Pwm,int MotorId);

#endif
