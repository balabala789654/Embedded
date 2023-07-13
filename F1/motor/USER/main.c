#include "stm32f10x.h"
#include "encoder.h"
#include "moter.h"
#include "delay.h"
#include "oled.h"
int main()
{
	Encoder_Init_TIM2();
	Encoder_Init_TIM4();
	MiniBalance_Motor_Init();
	MiniBalance_PWM_Init(7199,0);
	delay_init();	
	OLED_Init();
	while(1)
	{
		OLED_Display_On();
		OLED_ShowString(0,0,"DMP");
	}
}








