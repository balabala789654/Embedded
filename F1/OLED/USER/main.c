#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "oled.h"
#include "key.h"


int k = 0;
int main()
{
	KEY_Init();
	delay_init();
	OLED_Init();	
	OLED_Refresh_Gram();
	while(1)
	{
		k = KEY_Scan(1);
		if(k == 1)
		{
			OLED_Clear();
			OLED_Refresh_Gram();
		}
		else if(k == 2)
		{
			OLED_Clear();
			OLED_ShowString(0,0,"SB",24);
			OLED_Refresh_Gram();
		}
		else if(k == 3)
		{
			OLED_Clear();
			OLED_ShowString(0,0,":-)",24 );
			OLED_Refresh_Gram();
		}
	}
}





