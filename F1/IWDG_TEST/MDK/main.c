#include "stm32f10x.h"                  // Device header
#include "wdg.h"
#include "delay.h"
#include "gpio.h"

int main()
{
	LED_Init();
	delay_init();
	
	LED0=0;
	delay_ms(1000);
	LED0=1;
	IWDG_Init(4, 625);
	
	while(1)
	{
		
	}
}

