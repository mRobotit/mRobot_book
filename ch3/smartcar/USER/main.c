/**
 * 
 *   Website	:http://www.mrobotit.cn/
 * 	 Author 	: Xiao Dong
 * 	 Description: Main Function
 * 	 Date		: 2020/11
 * 	 
 */

#include "usart.h"
#include "i2c.h"
#include "mpu6050.h"
#include "delay.h"
#include "timer.h"
#include "motor.h"
#include "encoder.h"
#include "adc.h"
#include "string.h"
#include "pid.h"


int main(void)
{

	SystemInit();
	memset(&Left_moto, 0, sizeof(Left_moto));
	memset(&Right_moto, 0, sizeof(Right_moto));

	SysTick_Init();
	USART1_Config();

	TIM2_PWM_Init();	
	MOTOR_GPIO_Config(); 
	Adc_Init();
	
	i2cInit();		//IIC初始化
	USART1_Printf(USART1, "\r\n i2c over \r\n");
	delay_ms(1000); //delay 10ms
	MPU6050_Init(); //MPU6050 DMP初始化
	USART1_Printf(USART1, "\r\n  mpu over \r\n");
	Left_Encoder_Init();
	Right_Encoder_Init();
	USART1_Printf(USART1, "\r\n  encoder over \r\n");
	delay_ms(1000);
	Timerx_Init(500, 7199); //¶¨Ê±Æ÷TIM1
	delay_ms(500);

	while (1)
	{
		
		delay_ms(46);
		MPU6050_Pose();
		Robot_Encoder_Get_CNT();
	//	Send_Data.Sensor_Info.Adc_Voltage = Get_Valtage();
		//Usart_SendTo_Ubuntu();
		Pid_Control_speed(Left_moto.Current_Speed,Left_moto.Target_Speed,MOTO_LEFT);
		Pid_Control_speed(Right_moto.Current_Speed,Right_moto.Target_Speed,MOTO_RIGHT);
	
	}
	return 0;
}