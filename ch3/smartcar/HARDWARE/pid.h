#include "motor.h"

#define MOTO_DEAD_TIMER_COMPENSATION 	200

void Pid_Control_speed(float current_speed,float target_speed, unsigned char Moto_ID);
