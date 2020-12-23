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
#include "lcd.h"

//陀螺仪加速度与角速度分量，全局变量
float ax ,ay,az,gx,gy,gz;
//计数器
unsigned int Time_Count = 0;
int main(void)
{
	
	//下面为初始段的代码，主要做初始化工作
	//初始化变量
	memset(&Left_moto, 0, sizeof(Left_moto));
	memset(&Right_moto, 0, sizeof(Right_moto));
	Time_Count = 0;
	//系统初始化
	SystemInit();
	//初始化滴答计时器
	SysTick_Init();
	//USART1_Config();
	//初始化串口（蓝牙串口）
	USART3_Config();
	delay_ms(1000);
	//初始化PWM控制引脚
	TIM1_PWM_Init();	
	//初始化电机引脚
	MOTOR_GPIO_Config(); 
	//初始化ADC采样
	Adc_Init();
	//IIC初始化,MPU6050
	i2cInit();		
	delay_ms(500); //delay 10ms
	MPU6050_Init(); //MPU6050 DMP初始化
 //USART_Printf(USART3, "\r\n  mpu over \r\n");
 //初始化左右电机编码器
	Left_Encoder_Init();
	Right_Encoder_Init();
  //USART_Printf(USART3, "\r\n   encoder over \r\n");
	delay_ms(500);
	//初始化Tim1定时器
	Timerx_Init(100, 7199); 
	delay_ms(500);

	while (1)
	{
		delay_ms(1);
		if(Time_Count % 10 == 0)
		{
			//PID控制左右轮
		  Pid_Control_speed(Left_moto.Current_Speed,Left_moto.Target_Speed,MOTO_LEFT);
	    Pid_Control_speed(Right_moto.Current_Speed,Right_moto.Target_Speed,MOTO_RIGHT);
		
		}
		if(Time_Count % 20 == 0)
		{ 
			//获取MPU位姿
			MPU6050_Pose();
			//采样电压
		  Send_Data.Sensor_Info.Adc_Voltage = Get_Valtage();
			//发送到上位机
			Usart_SendTo_Ubuntu();
		}
		Time_Count++;
		(Time_Count >= 42949672) ?(Time_Count=0) : (Time_Count++);	//时间基数常数

	}
	return 0;
}