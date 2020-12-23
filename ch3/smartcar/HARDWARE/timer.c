/**
 * 
 *   Website	: http://www.mrobotit.cn/
 * 	 Author 	: Xiao Dong
 * 	 Description: 定时器，每间隔100ms计算电机转动速度
 * 	 Date		: 2020/11
 * 	 
 */

#include "timer.h"
#include "encoder.h"

#include "usart.h"

//通用定时器中断初始化
//这里时钟选择为APB2 72Mhz，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器1 ,定时时间t=arr*(psc*1/APB2(72Mhz))

void Timerx_Init(u16 arr,u16 psc)
{   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //TIM1时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到100为10ms
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(  //使能或者失能指定的TIM1中断
		TIM1, //TIM1
		TIM_IT_Update  |  //TIM1 中断源
		TIM_IT_Trigger,   //TIM1 触发中断源 
		ENABLE  //使能
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  //TIM1中断  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //先占优先级3级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
							 
}
//extern unsigned int Safeware_Count = 0;
void TIM1_UP_IRQHandler(void)   //TIM1中断
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) //检查指定的TIM1中断发生与否:TIM1 中断源 
		{
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM1 中断源 
		//在pid参数整定时，打开电机运动
		//(Safeware_Count >= 42949672) ?(Safeware_Count=0) : (Safeware_Count++);	//时间基数常数
		
		
		/*if(Safeware_Count == 50)
			Kinematics_Positive(0.5, 0.0);		//正向启动
		if(Safeware_Count == 100)
			Kinematics_Positive(-0.5, 0.0);	//反向切换
		if(Safeware_Count == 150)
			Kinematics_Positive(0.0, 0.0);		//反向停止
	 */	
		//每10ms采样一次左右电机速度
		Robot_Encoder_Get_CNT();
	}
}

