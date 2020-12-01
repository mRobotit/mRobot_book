#include "stm32f10x.h"
#include "motor.h"
#include "stm32f10x_exti.h"
#define ENCODER_TIM_PERIOD (u16)0xFFFF
#define MAX_COUNT          (u16)0x0FFF

int leftcount;
int rightcount;



void MOTOR_GPIO_Config(void)
{		
	/*¶¨ÒåÒ»¸öGPIO_InitTypeDefÀàÐÍµÄ½á¹¹Ìå*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*¿ªÆôGPIOBµÄÍâÉèÊ±ÖÓ*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 

	/*Ñ¡ÔñÒª¿ØÖÆµÄGPIOBÒý½Å*/															   
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;	

	/*ÉèÖÃÒý½ÅÄ£Ê½ÎªÍ¨ÓÃÍÆÍìÊä³ö*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*ÉèÖÃÒý½ÅËÙÂÊÎª50MHz */   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*µ÷ÓÃ¿âº¯Êý£¬³õÊ¼»¯GPIOB*/
  	GPIO_Init(GPIOB, &GPIO_InitStructure);		  

	/* µÍµçÆ½	*/
	GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14|GPIO_Pin_12 | GPIO_Pin_15);
	//GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14);
	//GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_15);	 
}

/*
 * º¯ÊýÃû£ºTIM2_GPIO_Config
 * ÃèÊö  £ºÅäÖÃTIM3¸´ÓÃÊä³öPWMÊ±ÓÃµ½µÄI/O
 * ÊäÈë  £ºÎÞ
 * Êä³ö  £ºÎÞ
 * µ÷ÓÃ  £ºÄÚ²¿µ÷ÓÃ
 */
static void TIM2_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

	/* TIM2 clock enable */
	//PCLK1¾­¹ý2±¶Æµºó×÷ÎªTIM2µÄÊ±ÖÓÔ´µÈÓÚ72MHz
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 

  /* GPIOA clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 

  /*GPIOA Configuration: TIM2 channel 3 and 4 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // ¸´ÓÃÍÆÍìÊä³ö
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

}

/*
 * º¯ÊýÃû£ºTIM2_Mode_Config
 * ÃèÊö  £ºÅäÖÃTIM3Êä³öµÄPWMÐÅºÅµÄÄ£Ê½£¬ÈçÖÜÆÚ¡¢¼«ÐÔ¡¢Õ¼¿Õ±È
 * ÊäÈë  £ºÎÞ
 * Êä³ö  £ºÎÞ
 * µ÷ÓÃ  £ºÄÚ²¿µ÷ÓÃ
 */
static void TIM2_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* PWMÐÅºÅµçÆ½Ìø±äÖµ */
	//u16 CCR1_Val = 500;        
	//u16 CCR2_Val = 800;
	u16 CCR3_Val = 0;
	u16 CCR4_Val = 0;
	//u16 PrescalerValue;
/* -----------------------------------------------------------------------
    TIM2 Configuration: generate 4 PWM signals with 4 different duty cycles:
    TIM3CLK = 72 MHz, Prescaler = 0x0, TIM3 counter clock = 72 MHz
    TIM3 ARR Register = 71999 => TIM3 Frequency = TIM3 counter clock/(ARR + 1)
    TIM3 Frequency = 1 KHz.
	CC1 update rate = TIM2 counter clock / CCR1_Val
    TIM3 Channelx duty cycle = (TIM2_CCRx/ TIM2_ARR)* 100 = x%
    
  ----------------------------------------------------------------------- */

  /* Time base configuration */		 
  TIM_TimeBaseStructure.TIM_Period = 1000 ;       //ARR µ±¶¨Ê±Æ÷´Ó0¼ÆÊýµ½999£¬¼´Îª1000´Î£¬ÎªÒ»¸ö¶¨Ê±ÖÜÆÚ
  TIM_TimeBaseStructure.TIM_Prescaler = 0;	    //ÉèÖÃÔ¤·ÖÆµ£º²»Ô¤·ÖÆµ£¬¼´Îª72MHz psc
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//ÉèÖÃÊ±ÖÓ·ÖÆµÏµÊý£º²»·ÖÆµ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //ÏòÉÏ¼ÆÊýÄ£Ê½

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* PWM2 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //ÅäÖÃÎªPWMÄ£Ê½1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//±È½ÏÊä³öÊ¹ÄÜ
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;	   //ÉèÖÃÌø±äÖµ£¬µ±¼ÆÊýÆ÷¼ÆÊýµ½Õâ¸öÖµÊ±£¬µçÆ½·¢ÉúÌø±ä
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //µ±¶¨Ê±Æ÷¼ÆÊýÖµÐ¡ÓÚCCR1_ValÊ±Îª¸ßµçÆ½

  TIM_OC3Init(TIM2, &TIM_OCInitStructure);	 //Ê¹ÄÜÍ¨µÀ2

  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;	  //ÉèÖÃÍ¨µÀ2µÄµçÆ½Ìø±äÖµ£¬Êä³öÁíÍâÒ»¸öÕ¼¿Õ±ÈµÄPWM

  TIM_OC4Init(TIM2, &TIM_OCInitStructure);	  //Ê¹ÄÜÍ¨µÀ3
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM2, ENABLE);			 // Ê¹ÄÜTIM3ÖØÔØ¼Ä´æÆ÷ARR

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);                   //Ê¹ÄÜ¶¨Ê±Æ÷2	
}

/*
 * º¯ÊýÃû£ºTIM2_PWM_Init
 * ÃèÊö  £ºTIM2 Êä³öPWMÐÅºÅ³õÊ¼»¯£¬Ö»Òªµ÷ÓÃÕâ¸öº¯Êý
 *         TIM2µÄËÄ¸öÍ¨µÀ¾Í»áÓÐPWMÐÅºÅÊä³ö
 * ÊäÈë  £ºÎÞ
 * Êä³ö  £ºÎÞ
 * µ÷ÓÃ  £ºÍâ²¿µ÷ÓÃ
 */


void TIM2_PWM_Init(void)
{
	TIM2_GPIO_Config();
	TIM2_Mode_Config();	
}




void PWM_OUTPUT(int Pwm,int MotorId)
{

	if(MotorId == MOTO_LEFT )
	{
		if(Pwm >=0){
		  GPIO_SetBits(GPIOB, GPIO_Pin_14 );			   //电机控制模块BIN1端 PB13		    
      GPIO_ResetBits(GPIOB, GPIO_Pin_15 );       //电机控制模块BIN2端 PB12	
		}else
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_15 );			   //电机控制模块BIN1端 PB13		    
      GPIO_ResetBits(GPIOB, GPIO_Pin_14 );       //电机控制模块BIN2端 PB12	
	
			Pwm = - Pwm;
		}
			TIM_SetCompare3(TIM2,Pwm);			       //TIM2与 Voltage对比，不相同则翻转波形，调节PWM占空比
	}else if(MotorId == MOTO_RIGHT){
			if(Pwm >=0){
		  GPIO_SetBits(GPIOB, GPIO_Pin_13 );			   //电机控制模块BIN1端 PB13		    
      GPIO_ResetBits(GPIOB, GPIO_Pin_12 );       //电机控制模块BIN2端 PB12	
		}else
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_12 );			   //电机控制模块BIN1端 PB13		    
      GPIO_ResetBits(GPIOB, GPIO_Pin_13 );       //电机控制模块BIN2端 PB12	
	
			Pwm = - Pwm;
		}
			TIM_SetCompare4(TIM2,Pwm);			       //TIM2与 Voltage对比，不相同则翻转波形，调节PWM占空比

	}
}
