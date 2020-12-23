/**
 * 
 *   Website	:http://www.mrobotit.cn/
 * 	 Author 	: Xiao Dong
 * 	 Description: 串口通信
 * 	 Date		: 2020/11
 * 	 
 */
#include "lcd.h"
#include <stdarg.h>
#include "misc.h"
#include  "usart.h"
#include "stdio.h"

void USART2_Config(void)
{
      GPIO_InitTypeDef GPIO_InitStructure;
      USART_InitTypeDef USART_InitStructure;
      NVIC_InitTypeDef NVIC_InitStructure;
      //|RCC_APB2Periph_AFIO
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能GPIOA时钟
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
  
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;    //PA2
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //复用推挽
      GPIO_Init(GPIOA, &GPIO_InitStructure);
  
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
      GPIO_Init(GPIOA, &GPIO_InitStructure);  
  
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);//复位串口2
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,DISABLE);//停止复位
  
      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    //设置NVIC中断分组2:2位抢占优先级，2位响应优先级   0-3;
      NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //使能串口2中断
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //先占优先级2级
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //从优先级2级
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能外部中断通道
      NVIC_Init(&NVIC_InitStructure); //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
  
      USART_InitStructure.USART_BaudRate = 115200;//波特率设置
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据长度
      USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
      USART_InitStructure.USART_Parity = USART_Parity_No;///奇偶校验位
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
      USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发模式
  
      USART_Init(USART2, &USART_InitStructure); ; //初始化串口
      USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
      USART_Cmd(USART2, ENABLE);                    //使能串口
}

void copy_data(void *dst,const void *src ,unsigned int len)
{
	int j=0;
	for(;j<len;j++)
	{
		((uint8_t*)dst)[j] = ((uint8_t*)src)[len-j-1];
	}
}

uint8_t send_buf[30];
void Usart_SendTo_DWLCD(float ax,float ay,float az,float gx,float gy,float gz)
{
	   int i;
		LCD_Send_Data send_data ;
		memset(send_buf,0,30);
		copy_data(send_buf+0,&W_HEADER,2);
		copy_data(send_buf+2,&LCD_DATA_LEN,1);
   	copy_data(send_buf+3,&LCD_W_OPCODE,1);
	  copy_data(send_buf+4,&LCD_DATA_ADDR,2);


	  copy_data(send_buf+6,&ax,4);	
	  copy_data(send_buf+10,&ay,4);	
	  copy_data(send_buf+14,&az,4);	
	  copy_data(send_buf+18,&gx,4);		
		copy_data(send_buf+22,&gy,4);	
		copy_data(send_buf+26,&gz,4);	
	
		/*send_data.data_info.Header = W_HEADER;
		send_data.data_info.len = LCD_DATA_LEN;
		send_data.data_info.opcode = LCD_W_OPCODE;
	  send_data.data_info.addr = LCD_DATA_ADDR;
	
		send_data.data_info.ax = ax;
		send_data.data_info.ay = ay;
		send_data.data_info.az = az;
		
		send_data.data_info.gx = gx;
		send_data.data_info.gy = gy;
		send_data.data_info.gz = gz;*/
	
	
    for (i = 0; i < 30; i++)
    {

        USART_Send_Byte(USART2, send_buf[i]);
    }
}

void USART2_IRQHandler(void)
{
	
	  if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
			if(1==1)
	{
			USART_ReceiveData(USART2);
			USART_Printf(USART1, "\r\n hello \r\n");
	}
		}

	
	
}
