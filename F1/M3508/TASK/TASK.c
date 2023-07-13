#include "task.h"
#include "can.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "FreeRTOSConfig.h"
#include "oled.h"
#include "led.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

extern PID pid_control;
extern M3508 M3508_control;
extern GM6020 GM6020_control;
float pitch, yaw, roll;

void task1_task(void *pvParameters)
{
	
	
	GM6020_control.pid_set_angle_6020 = 8191;
	GM6020_control.pid_set_speed_6020 = 1000;
	M3508_control.pid_set_angle = 0;
	M3508_control.pid_set_speed = 1000;
	while(1)
	{
		mpu_dmp_get_data(&pitch,&roll,&yaw);
//		OLED_ShowString(0,0, "motor is running",12);
		OLED_Refresh_Gram();
		OLED_ShowNum(50,0,pitch,3,12);
		OLED_ShowNum(50,10,roll,3,12);
		OLED_ShowNum(50,20,yaw,3,12);
		M3508_control.pid_set_angle = pitch*1000;
		PID_calc(&pid_control.PID[0],M3508_control.M3508[0].all_ecd,M3508_control.pid_set_angle);
		PID_calc(&pid_control.PID[1],M3508_control.M3508[0].speed_rpm,pid_control.PID[0].out);
		moter_send_3508(pid_control.PID[1].out,pid_control.PID[1].out,pid_control.PID[1].out,pid_control.PID[1].out);
		delay_ms(5);
	}

}

void task2_task(void *pvParameters)
{
	while(1)
	{
		OLED_ShowString(0,0,"pitch: ",12);
		OLED_ShowString(0,10,"roll: ",12);
		OLED_ShowString(0,20,"yaw: ",12);

		delay_ms(1);
	}
}

void task3_task(void *pvParameters)
{
	while(1)
	{
		OLED_ShowNum(0,50,-30,4,12);
		LED0=!LED0;
//		OLED_ShowString(0,20,"task3 is running",12);
	}
}



