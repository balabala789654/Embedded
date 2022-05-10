#include "stm32f10x.h"
#include "led.h"
#include "sys.h"
void LED_Init(void)
{	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
	
	GPIO_InitTypeDef GPIO_Stru;
	GPIO_Stru.GPIO_Mode =  GPIO_Mode_Out_PP;
	GPIO_Stru.GPIO_Pin = GPIO_Pin_5;
	GPIO_Stru.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB,&GPIO_Stru);
	GPIO_SetBits(GPIOB,GPIO_Pin_5 );
		
	GPIO_Init(GPIOE,&GPIO_Stru);
	GPIO_SetBits(GPIOE,GPIO_Pin_5 );

    
}



