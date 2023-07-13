#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "oled.h"

int main()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	delay_init();	    	 //延时函数初始化	  
	OLED_Init();			//初始化OLED  
	OLED_Clear(); 
	OLED_ShowCHinese(18,0,3);//电
	OLED_ShowCHinese(36,0,4);//子
	OLED_ShowCHinese(72,0,5);//科
	OLED_ShowCHinese(90,0,6);//技

	while(1)
	{
	}
}

