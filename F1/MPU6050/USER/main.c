#include "stm32f10x.h"                  // Device header
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "delay.h"
#include "oled.h"
#include "led.h"

float pitch, yaw, roll;
int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init();
	OLED_Init();	
	LED_Init();
	MPU_Init();
	while(mpu_dmp_init());
//	OLED_Display_On();
	OLED_ShowString(0,0,"pitch: ",12);
	OLED_ShowString(0,10,"roll: ",12);
	OLED_ShowString(0,20,"yaw: ",12);
	OLED_Refresh_Gram();
	while(1)
	{
		LED0=!LED0;
		mpu_dmp_get_data(&pitch,&roll,&yaw);
		OLED_ShowNum(40,0,pitch,2,12);
		OLED_ShowNum(40,10,roll,2,12);
		OLED_ShowNum(40,20,yaw,2,12);
		OLED_Refresh_Gram();
		delay_ms(1);
	}
}
	
	


