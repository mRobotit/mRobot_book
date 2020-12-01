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
	float t,t1;
void Left_Encoder_Init(void)
{
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//?????4???
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//??PB????

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;;	//????
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //????
	GPIO_Init(GPIOA, &GPIO_InitStructure);					      //?????????GPIOB

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // ???? 
	TIM_TimeBaseStructure.TIM_Period =  0xFFFF;; //??????????
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//??????:???
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM????  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//???????3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);//??TIM??????
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM3,0x7fff);
	TIM_Cmd(TIM3, ENABLE); 
	
}


void Right_Encoder_Init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
/* Encoder unit connected to TIM3, 4X mode */    
  GPIO_InitTypeDef GPIO_InitStructure;
  //NVIC_InitTypeDef NVIC_InitStructure;
  
  /* TIM3 clock source enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  /* Enable GPIOA, clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  GPIO_StructInit(&GPIO_InitStructure);
  /* Configure PA.06,07 as encoder input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Enable the TIM3 Update Interrupt */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    //优先级组别
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
   
  /* Timer configuration in Encoder mode */
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
  
 // Clear all pending interrupts
 
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);	  //使能中断
  
  //Reset counter

	TIM_SetCounter(TIM4,0x7fff);
  
  TIM_Cmd(TIM4, ENABLE);  	   //使能定时器3
}

void TIM3_IRQHandler(void)
{ 		    		  			    
	 u16 tsr;
	 tsr=TIM3->SR;	
	 if(tsr&0X0001)//溢出中断
	 {
																				
	 }				   
	 TIM3->SR&=~(1<<0);//清除中断标志位 	 

}

void TIM4_IRQHandler(void)
{ 		    		  			    
	 u16 tsr;
	 tsr=TIM4->SR;	
	 if(tsr&0X0001)//溢出中断
	 {
																				
	 }				   
	 TIM4->SR&=~(1<<0);//清除中断标志位 	 

}



void Print_MotorSpeed(int value,int MotorId)
{
  t = Left_moto.Current_Speed*100;
	//Vcan_sendware((u8* )&	(Left_moto.Current_Speed), sizeof(Left_moto.Current_Speed));
	Vcan_sendware((u8* )&	(t), sizeof(t));
}
int lefts = 0;
void  Robot_Encoder_Get_CNT(void)
{
	Left_moto.Encoder_Value   = (TIM4->CNT-0x7fff);		
	Right_moto.Encoder_Value  = -(TIM3->CNT-0x7fff);
  

	Left_moto.Current_Speed 
		= ((ROBOT_INITIATIVE_DIAMETER_L *Pi * (Left_moto.Encoder_Value  / (ENCODER_COUNT_VALUE))  )/CONTROL_TIMER_CYCLE);
	Right_moto.Current_Speed
		= ((ROBOT_INITIATIVE_DIAMETER_R  *Pi * (Right_moto.Encoder_Value / (ENCODER_COUNT_VALUE)))/CONTROL_TIMER_CYCLE);
	lefts = Left_moto.Current_Speed*100 ;
//	USART1_Printf(USART1, "\r\n----------left---------%d \r\n", Left_moto.Encoder_Value);
	//USART1_Printf(USART1, "\r\n----------lefts---------%d \r\n", lefts);
	TIM_SetCounter(TIM3,0x7fff);
	TIM_SetCounter(TIM4,0x7fff);
}
