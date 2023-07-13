#include "stm32f10x.h"                  // Device header
#include "timer.h"
#include "gpio.h"
#include "delay.h"
int main()
{
	TIM1_PWM_Init(5000-1,72-1);
	gpio_init();
	delay_init();
	while(1)
	{
		TIM_SetCompare1(TIM1,5000);
		TIM_SetCompare2(TIM1,5000);
		TIM_SetCompare3(TIM1,5000);
		PB13(ON);
		delay_ms(1);
		PB13(OFF);
		delay_ms(1);

	}
}

