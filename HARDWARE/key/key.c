#include "stm32f10x.h"
#include "key.h"
#include "delay.h"
#include "sys.h"
#include "stdio.h"
#include "beep.h"
#include "time.h"
#include "usart.h"	  

extern int mode; 
void KEY_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIOE_Stru;
	GPIOE_Stru.GPIO_Mode = GPIO_Mode_IPU;
	GPIOE_Stru.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_3;
	GPIO_Init(GPIOE,&GPIOE_Stru);
	
	GPIOE_Stru.GPIO_Pin = GPIO_Pin_0;
	GPIOE_Stru.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA,&GPIOE_Stru);
}

u8 KEY_Scan(void)
{
	if(KEY0==0||KEY1==0||KEY_UP==1)
	{
		delay_ms(10);
		if(KEY0==0)
		{
			delay_ms(20);			
			return 3;
		}
		else if(KEY1==0)
		{
			delay_ms(20);			
			return 2;
		}
		else if(KEY_UP==1)
		{
			delay_ms(20);
			return 5;

		}

	}
	return 0;
}



















