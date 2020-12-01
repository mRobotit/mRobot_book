/*
 * @Author: your name
 * @Date: 2020-11-10 10:08:38
 * @LastEditTime: 2020-11-19 17:04:13
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: \USERc:\smartcar-project\HARDWARE\delay.c
 */
/**
 * 
 *   Website	: http://www.mrobotit.cn/
 * 	 Author 	: Xiao Dong
 * 	 Description: 系统滴答计时器，微秒、毫秒、秒级延时函数
 * 	 Date		: 2020/11
 * 	 
 */
#include "stm32f10x.h"
#include "delay.h"

static u8  fac_us=0;           //???????
static u16 fac_ms=0;           //???????
static unsigned long  timestamp = 0;

void get_ms(unsigned long *time)
{

}

void delayTime(u32 n)
{
		while(n--);
}


void delay_us(u32 nus)
{
    u32 temp;
    //nus*fac_us???????SysTick->LOAD(24?)-1
    SysTick->LOAD=nus*fac_us;    // ?????:n(us)*??1us?????SysTick????
    SysTick->VAL=0x00;                       // VAL????0
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; // ??SysTick???
    do
    {
        temp=SysTick->CTRL;
    }while((temp&0x01)&&!(temp&(1<<16)));    // ????????(?16)
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; // ????
    SysTick->VAL =0X00;                      // ??VAL
	}

/**
 * nms : ??????
 **/
void delay_ms(u16 nms)
{
    u32 temp;
    SysTick->LOAD=(u32)nms*fac_ms;
    SysTick->VAL =0x00;
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;
    do
    {
        temp=SysTick->CTRL;
    }while((temp&0x01)&&!(temp&(1<<16)));
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL =0X00;
	}

	void delay_S(u16 ns)
	{
		u16 i;
    for(i=0;i<ns;i++)
    {
        delay_ms(1000);
    }
	}
static __IO u32 TimingDelay;

/*
 * 函数名：SysTick_Init
 * 描述  ：启动系统滴答定时器 SysTick
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用 
 */
void SysTick_Init(void)		  //1ms定时时钟
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); //?????-????-HCLK/8
    fac_us=SystemCoreClock/8000000; // 72/8 ??1??9?????
    fac_ms=(u16)fac_us*1000;   // ??1??9000?Cystic????

}


