/**
 * 
 *   Website	:http://www.mrobotit.cn/
 * 	 Author 	: Xiao Dong
 * 	 Description: 串口通信
 * 	 Date		: 2020/11
 * 	 
 */
#include "usart.h"
#include <stdarg.h>
#include "misc.h"
#include "motor.h"

Upload_Data Send_Data, Recive_Data;

void USART1_Config(void)
{
    //变量声明
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    //初始化USART1时钟
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    // 将 USART Tx 的 GPIO 配置为推挽复用模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // 将 USART Rx 的 GPIO 配置为浮空输入模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Pin = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // 配置串口的工作参数
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);   
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void Kinematics_Positive(float vx,float vz)
{
	if(vx == 0.0f){			//原地旋转或静止
		Right_moto.Target_Speed = vz * Base_Width / 2.0f;
		Left_moto.Target_Speed  = (-1) * Right_moto.Target_Speed;
	}
	else if(vz == 0.0f){	//静止或者前后运动
		Right_moto.Target_Speed = Left_moto.Target_Speed = vx;
	}	
	else{					//在前进或者后退过程中转弯
		Left_moto.Target_Speed  = vx - vz * Base_Width / 2.0f;
		Right_moto.Target_Speed = vx + vz * Base_Width / 2.0f;
	}

}
unsigned char Rcount = 0;
void USART1_IRQHandler(void)
{
     USART_SendData(USART1,1);
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {

				Recive_Data.buffer[Rcount] = USART_ReceiveData(USART1);
				(Recive_Data.buffer[0] == 0xFe)?(Rcount++):(Rcount = 0);
				if (Rcount == PROTOCL_DATA_SIZE)	//验证数据包的长度
				{
					if(Recive_Data.Sensor_Info.Header == PROTOCOL_HEADER)	//验证数据包的头部校验信息
					{
						if(Recive_Data.Sensor_Info.End_flag == PROTOCOL_END)	//验证数据包的尾部校验信息
						{
							//接收上位机控制命令，使机器人产生相应的运动
							Kinematics_Positive(Recive_Data.Sensor_Info.X_speed, Recive_Data.Sensor_Info.Z_speed);
						}
					}
					Rcount = 0;
				}
    }
}

static char *itoa(int value, char *string, int radix)
{
    int i, d;
    int flag = 0;
    char *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;
}

void USART1_SendChar(unsigned char b)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        ;
    USART_SendData(USART1, b);
}

void USART1_Printf(USART_TypeDef *USARTx, uint8_t *Data, ...)
{
    const char *s;
    int d;
    char buf[16];

    va_list ap;
    va_start(ap, Data);

    while (*Data != 0)
    {
        if (*Data == 0x5c)
        {
            switch (*++Data)
            {
            case 'r':
                USART_SendData(USARTx, 0x0d);
                Data++;
                break;

            case 'n':
                USART_SendData(USARTx, 0x0a);
                Data++;
                break;

            default:
                Data++;
                break;
            }
        }
        else if (*Data == '%')
        {
            switch (*++Data)
            {
            case 's':
                s = va_arg(ap, const char *);
                for (; *s; s++)
                {
                    USART_SendData(USARTx, *s);
                    while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
                        ;
                }
                Data++;
                break;

            case 'd':
                d = va_arg(ap, int);
                itoa(d, buf, 10);
                for (s = buf; *s; s++)
                {
                    USART_SendData(USARTx, *s);
                    while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
                        ;
                }
                Data++;
                break;
            default:
                Data++;
                break;
            }
        } // end of else if
        else
            USART_SendData(USARTx, *Data++);
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
            ;
    }
}

void USART1_Send_Byte(unsigned char byte)
{
    USART_SendData(USART1, byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
}

void PrintChar(char *s)
{
	//USART1_SendChar(s);
	char *p;
	p=s;
	while(*p != '\0')
	{
		USART1_Send_Byte(*p);
		p++;
	}
}


void Usart_putbuff(uint8_t *data,uint32_t size)
{
    uint8_t i=0;
    for(;i<size;i++)
    {
        USART1_Send_Byte(*(data+i));
    }
}


void Vcan_sendware(uint8_t *wareaddr, uint32_t waresize)
{
    uint8_t cmdf[2] = {0x03, 0xfc};    //串口调试 使用的前命令
    uint8_t cmdr[2] = {0xfc, 0x03};    //串口调试 使用的后命令

    Usart_putbuff(cmdf, sizeof(cmdf));    //先发送前命令
    Usart_putbuff(wareaddr, waresize);    //发送数据
    Usart_putbuff(cmdr, sizeof(cmdr));    //发送后命令
}


void Usart_SendTo_Ubuntu()
{
	unsigned char i = 0;
	Send_Data.Sensor_Info.Header   = PROTOCOL_HEADER;
	Send_Data.Sensor_Info.End_flag = PROTOCOL_END;
	
	Send_Data.Sensor_Info.X_speed = (Left_moto.Current_Speed + Right_moto.Current_Speed)/2.0f;
	Send_Data.Sensor_Info.Y_speed = 0.0;
	Send_Data.Sensor_Info.Z_speed = (Left_moto.Current_Speed - Right_moto.Current_Speed)/Base_Width;
	
	
	for(i=0; i<PROTOCL_DATA_SIZE; i++)
	{
		USART1_SendChar(Send_Data.buffer[i]);
	}
}

