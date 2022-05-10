#include "beep.h"

void BEEP_Init(void)
{
	GPIO_InitTypeDef GPIO_stru;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	
	
	GPIO_stru.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_stru.GPIO_Pin=GPIO_Pin_8;
	GPIO_stru.GPIO_Speed=GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB,&GPIO_stru);
	GPIO_ResetBits(GPIOB,GPIO_Pin_8);
}



