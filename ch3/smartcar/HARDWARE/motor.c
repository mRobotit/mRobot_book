#include "stm32f10x.h"
#include "motor.h"
#include "stm32f10x_exti.h"
#define ENCODER_TIM_PERIOD (u16)0xFFFF
#define MAX_COUNT          (u16)0x0FFF

int leftcount;
int rightcount;


//初始化电机引脚
void MOTOR_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

  	/*开启GPIOB的外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 

  	/*选择要控制的GPIOB引脚*/															   
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;	

		/*设置引脚模式为通用推挽输出*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

  	/*设置引脚速率为50MHz */    
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  	/*调用库函数，初始化GPIOB*/
  	GPIO_Init(GPIOB, &GPIO_InitStructure);		  

  /* 初始化低电平	*/
	GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14|GPIO_Pin_12 | GPIO_Pin_15);

}

//配置TIM1复用输出PWM时用到的I/O
static void TIM1_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

	/*打开GPIOA时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	/*打开TIM1时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);   
	
  /*GPIOA 引脚配置 */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  /*GPIOA 初始化 */
  GPIO_Init(GPIOA, &GPIO_InitStructure);

}

//配置TIM2输出的PWM信号的模式，如周期、极性、占空比
static void TIM1_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;


	u16 CCR1_Val = 0;
	u16 CCR4_Val = 0;


  /* 定时器基本设置*/		 
  TIM_TimeBaseStructure.TIM_Period = 1000 ;     						  // 当定时器从0计数到999，即为1000次，为一个定时周期
  TIM_TimeBaseStructure.TIM_Prescaler = 0;	  						    //设置预分频：不预分频，即为72MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;		//设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

   /*Chanle1配置 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//比较输出使能
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);	 //使能通道1

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /*Chanle4配置 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;	 //设置跳变值，当计数器计数到这个值时，电平发生跳变

  TIM_OC4Init(TIM1, &TIM_OCInitStructure);	  //使能通道4
	TIM_CtrlPWMOutputs(TIM1,ENABLE);    //MOE
  TIM_ARRPreloadConfig(TIM1, ENABLE); // 使能TI13重载寄存器ARR

  TIM_Cmd(TIM1, ENABLE);  // //使能定时器1

	

}

//初始化函数
void TIM1_PWM_Init(void)
{
	TIM1_GPIO_Config();
	TIM1_Mode_Config();	
}

//控制电机转速以及方向
void PWM_OUTPUT(int Pwm,int MotorId)
{

	if(MotorId == MOTO_LEFT )
	{
		if(Pwm >=0){
		  GPIO_SetBits(GPIOB, GPIO_Pin_14 );			   //电机控制模块BIN1端 PB14		    
      GPIO_ResetBits(GPIOB, GPIO_Pin_15 );       //电机控制模块BIN2端 PB15	
		}else
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_15 );			   //电机控制模块BIN1端 PB15		    
      GPIO_ResetBits(GPIOB, GPIO_Pin_14 );       //电机控制模块BIN2端 PB14	
	
			Pwm = - Pwm;
		}
			TIM_SetCompare1(TIM1,Pwm);			       
	}else if(MotorId == MOTO_RIGHT){
			if(Pwm >=0){
		  GPIO_SetBits(GPIOB, GPIO_Pin_13 );			   //电机控制模块BIN1端 PB13		    
      GPIO_ResetBits(GPIOB, GPIO_Pin_12 );       //电机控制模块BIN2端 PB12	
		}else
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_12 );			   //电机控制模块BIN1端 PB12		    
      GPIO_ResetBits(GPIOB, GPIO_Pin_13 );       //电机控制模块BIN2端 PB13	
	
			Pwm = - Pwm;
		}
			TIM_SetCompare4(TIM1,Pwm);			       

	}
}
