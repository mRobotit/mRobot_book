 /**
 * 
 *   Website	: http://www.mrobotit.cn/
 * 	 Author 	: Xiao Dong
 * 	 Description: 编码器，采样电机速率
 * 	 Date		: 2020/11
 * 	 
 */
 

#include "stm32f10x.h"
#include "encoder.h"

#include "stm32f10x.h"
#include "usart.h"
#include "motor.h"
//初始化左编码器
void Left_Encoder_Init(void)
{
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	//打开TIM3时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	//打开GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//配置引脚
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);				

  //打开TIM3中断
	TIM_DeInit(TIM3);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    //优先级组别
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	//编码器的定时器设置
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 
	TIM_TimeBaseStructure.TIM_Period =  0xFFFF;;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//???????3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	//重置编码器
	TIM_SetCounter(TIM3,0x7fff);
	TIM_Cmd(TIM3, ENABLE); 
	
}


void Right_Encoder_Init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure; 
  GPIO_InitTypeDef GPIO_InitStructure;

  
	//打开TIM3时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//打开GPIOB时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  GPIO_StructInit(&GPIO_InitStructure);
	//配置引脚
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //打开TIM3中断
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    //优先级组别
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
   
	//编码器的定时器设置
  TIM_DeInit(TIM4);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式 
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
 
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge); //TIM_ICPolarity_Rising上升沿捕获
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 6; //无滤波器
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);	  //使能中断
  
	//重置编码器
	TIM_SetCounter(TIM4,0x7fff);
  TIM_Cmd(TIM4, ENABLE);  	   //使能定时器3
}
//TIM3中断，当计数溢出会出发中断
void TIM3_IRQHandler(void)
{ 		    		  			    
	 u16 tsr;
	 tsr=TIM3->SR;	
	 if(tsr&0X0001)//溢出中断
	 {
																				
	 }				   
	 TIM3->SR&=~(1<<0);//清除中断标志位 	 

}
//TIM4中断，当计数溢出会出发中断
void TIM4_IRQHandler(void)
{ 		    		  			    
	 u16 tsr;
	 tsr=TIM4->SR;	
	 if(tsr&0X0001)//溢出中断
	 {
																				
	 }				   
	 TIM4->SR&=~(1<<0);//清除中断标志位 	 

}
//调试函数，输出到虚拟示波器中
void Print_MotorSpeed(int value,int MotorId)
{
  float t = Left_moto.Current_Speed*100;
	Vcan_sendware((u8* )&	(t), sizeof(t));
}
//采样左右编码器的脉冲数，并计算出转速
void  Robot_Encoder_Get_CNT(void)
{
	//采样脉冲
	Left_moto.Encoder_Value   = (TIM4->CNT-0x7fff);		
	Right_moto.Encoder_Value  = (TIM3->CNT-0x7fff);
  
	//计算转速
	Left_moto.Current_Speed 
		= ((ROBOT_INITIATIVE_DIAMETER_L *Pi * (Left_moto.Encoder_Value  / (ENCODER_COUNT_VALUE))  )/CONTROL_TIMER_CYCLE);
	Right_moto.Current_Speed
		= ((ROBOT_INITIATIVE_DIAMETER_R  *Pi * (Right_moto.Encoder_Value / (ENCODER_COUNT_VALUE)))/CONTROL_TIMER_CYCLE);
	//重置计数器
	TIM_SetCounter(TIM3,0x7fff);
	TIM_SetCounter(TIM4,0x7fff);
}
