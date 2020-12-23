#include "stdio.h"	
#include <stdbool.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"



static const u16 W_HEADER = 0x5AA5;

static const u16 LCD_DATA_ADDR = 0x0100;

static const u8 LCD_DATA_LEN  = 0x30;

static const u8 LCD_W_OPCODE  = 0x82;

#define LCD_DATA_SIZE 34

#pragma pack(1)
typedef union   
{
	unsigned char buffer[LCD_DATA_SIZE];
	struct
	{
		u32 Header;
		uint8_t len;
		uint8_t opcode;
		
		u16 addr;
		float ax;			
		float ay;
		float az;
	
		float gx;			
		float gy;
		float gz;

	}data_info;
}LCD_Send_Data;
#pragma pack()

void USART2_Config(void); 
void Usart_SendTo_DWLCD(float ax,float ay,float az,float gx,float gy,float gz);
void USART2_IRQHandler();