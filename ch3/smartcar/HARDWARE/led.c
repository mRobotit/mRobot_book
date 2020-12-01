/***********************************************
** 文件名称：led.c
** 功能描述: led 应用函数库            
** 实验平台：两轮自平衡小车
** 硬件连接：----------------- 
**	   		|   PB3 - LED1     |
**			|   PB4 - LED2     |
**			 ----------------- 
**********************************************************************************/
#include "led.h"

/*
 * 函数名：LED_GPIO_Config
 * 描述  ：配置LED用到的I/O口
 * 输入  ：无
 * 输出  ：无
 */
void LED_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*先开启GPIOB和AFIO的外设时钟*/
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO,ENABLE);

	/*改变指定管脚的映射 GPIO_Remap_SWJ_JTAGDisable ，JTAG-DP 禁用 + SW-DP 使能*/
  //  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE); 

	/*选择要控制的GPIOB引脚*/															   
  //	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	
	/*设置引脚速率为50MHz */   
  //	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 	

	/*设置引脚模式为通用推挽输出*/
  //	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*调用库函数，初始化GPIOB*/
  //	GPIO_Init(GPIOB, &GPIO_InitStructure);			  

	/* 打开所有led灯	*/
	//  GPIO_SetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4); 	




	/*先开启GPIOB和AFIO的外设时钟*/
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO,ENABLE);

	/*改变指定管脚的映射 GPIO_Remap_SWJ_JTAGDisable ，JTAG-DP 禁用 + SW-DP 使能*/
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE); 

	/*选择要控制的GPIOB引脚*/															   
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 |GPIO_Pin_14;
	
	/*设置引脚速率为50MHz */   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 	

	/*设置引脚模式为通用推挽输出*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*调用库函数，初始化GPIOB*/
  	GPIO_Init(GPIOC, &GPIO_InitStructure);			  

	/* 打开所有led灯	*/
	GPIO_SetBits(GPIOC, GPIO_Pin_13|GPIO_Pin_14); 



}



/************************END OF FILE************/
