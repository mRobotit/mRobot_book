
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

static u8  fac_us=0;           
static u16 fac_ms=0;          
static unsigned long  timestamp = 0;

//未实现，适配DMP库所用
void get_ms(unsigned long *time)
{

}


//微妙级别延时
void delay_us(u32 nus)
{
    u32 temp;
    SysTick->LOAD=nus*fac_us;   						 //加载计数次数
    SysTick->VAL=0x00;                       //清空计时器
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; //打开计数器
    do
    {
        temp=SysTick->CTRL;
    }while((temp&0x01)&&!(temp&(1<<16)));    // 等待时间到达
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; // 关闭计数器
    SysTick->VAL =0X00;                      // 清空计数器
	}

//毫秒级别延时
void delay_ms(u16 nms)
{
    u32 temp;
    SysTick->LOAD=(u32)nms*fac_ms; 					//加载计数次数
    SysTick->VAL =0x00; 										//清空计时器
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; // 等待时间到达
    do
    {
        temp=SysTick->CTRL;
    }while((temp&0x01)&&!(temp&(1<<16)));		 // 等待时间到达
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; // 关闭计数器
    SysTick->VAL =0X00;											 // 清空计数器
	}	
//秒级延时 直接调用1000次delay_ms
void delay_S(u16 ns)
{
	u16 i;
	for(i=0;i<ns;i++)
	{
			delay_ms(1000);
	}
}
static __IO u32 TimingDelay;

//启动系统滴答定时器 SysTick
void SysTick_Init(void)	
{
   	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//选择外部时钟  HCLK的八分频
    fac_us=SystemCoreClock/8000000; 										 //1 us计时器需要计数次数
    fac_ms=(u16)fac_us*1000;   													 //1 ms计时器需要计数次数

}


