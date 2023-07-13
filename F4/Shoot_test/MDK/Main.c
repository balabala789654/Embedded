#include "stm32f4xx.h"                  // Device header
#include "Remote_Control.h"
#include "gpio.h"
#include "timer.h"
#include "usart.h"
#include "delay.h"

extern RC_ctrl_t rc_ctrl;

int main(void)
{
	
	gpio_Init();
	remote_control_init();
	TIM3_Int_Init(5000-1, 840-1);
	remote_control_init();
	TIM5_PWM_Init(5000-1, 84 - 1);
	delay_init(168);
	
	
	int compare=1000;
	
	while(1)
	{
		if(rc_ctrl.rc.s[1]==3 && rc_ctrl.rc.s[0]==3)
		{
			compare=1000;
		}
		else 
		{
			if(rc_ctrl.rc.s[0]==1)
			{
				if(rc_ctrl.rc.s[1]==1) compare=1200;
				else if(rc_ctrl.rc.s[1]==3) compare=1500;
				else if(rc_ctrl.rc.s[1]==2) compare=1800;
			}
			else	compare=1000;
		}
		
		TIM_SetCompare1(TIM5, compare);
	}
}

