#include "stdio.h"	
#include <stdbool.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"


#define PROTOCOL_HEADER		0XFEFEFEFE
#define PROTOCOL_END		0XEE

#define PROTOCL_DATA_SIZE 33

#pragma pack(1)
typedef struct
{
	short X_data;
	short Y_data;
	short Z_data;
}Imu_Info;

typedef union   
{
	unsigned char buffer[PROTOCL_DATA_SIZE];
	struct
	{
		unsigned int Header;
		float X_speed;			
		float Y_speed;
		float Z_speed;
		float Adc_Voltage;
		
		Imu_Info Imu_Acc;
		Imu_Info Imu_Gyro;
		
		unsigned char End_flag;
	}Sensor_Info;
}Upload_Data;

#pragma pack(4)

extern Upload_Data Send_Data, Recive_Data;

void USART1_Config(void); 

void USART3_Config(void); 

void USART_SendChar(USART_TypeDef *USARTx,unsigned char b);

void USART_Printf(USART_TypeDef* USARTx, uint8_t *Data,...);

void USART1_Send_Byte(USART_TypeDef *USARTx,unsigned char byte);

void PrintChar(char *s);

void Vcan_sendware(uint8_t *wareaddr, uint32_t waresize);

void Kinematics_Positive(float vx,float vz);

void Usart_SendTo_Ubuntu();
