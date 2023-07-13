#include "can.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "delay.h"
#include "FreeRTOSConfig.h"
#include "oled.h"
#include "led.h"
#include "key.h"
#include "task1.h"
#include "task.h"
#include "usart.h"	  

extern int Res;
extern TaskHandle_t targettask_Handler;
extern TaskHandle_t usarttask_Handler;
extern TaskHandle_t PIDtask_Handler;

extern PID pid_control;
extern M3508 M3508_control;
extern GM6020 GM6020_control;

int key = 0;			/////////////
int key0_toggle=0;		/////////////
int key1_toggle=0;		//切换标志///
int key_up_toggle=0;	////////////
int key_function=0;		////////////

int key_pid_set=0;		//kp ki kd

/////////////////电机任务//////////////////
void motor_task(void *pvParameters)
{
	while(1)
	{
		OLED_Refresh_Gram();
		if(key_function==1)
		{
			switch(key0_toggle)
			{
				case 0:		//GM6020
				{
//					moter_send_3508(0,0,0,0);
					motor_send_6020(pid_control.PID[1].out,pid_control.PID[1].out,pid_control.PID[1].out,pid_control.PID[1].out);
					LED0 = 0;
					OLED_ShowNum(70,0,GM6020_control.GM6020.all_ecd,5,12);
					OLED_ShowNum(70,10,GM6020_control.GM6020.speed_rpm,4,12);	
				}
				break;
				case 1:		//M3508
				{
					moter_send_3508(pid_control.M3508_PID[1].out,pid_control.M3508_PID[1].out,pid_control.M3508_PID[1].out,pid_control.M3508_PID[1].out);
					LED0 = 1;
					OLED_ShowNum(70,0,M3508_control.M3508[2].all_ecd,5,12);
					OLED_ShowNum(70,10,M3508_control.M3508[2].speed_rpm,4,12);	
				}
				break;
			}
			
		}
		else 
		{
			if(key0_toggle==0)
			{
				LED0 = 0;
				motor_send_6020(pid_control.PID[1].out,pid_control.PID[1].out,pid_control.PID[1].out,pid_control.PID[1].out);
				OLED_ShowNum(70,0,GM6020_control.GM6020.all_ecd,5,12);
				OLED_ShowNum(70,10,GM6020_control.GM6020.speed_rpm,4,12);	
			}
			else 
			{
				LED0 = 1;
				moter_send_3508(pid_control.M3508_PID[1].out,pid_control.M3508_PID[1].out,pid_control.M3508_PID[1].out,pid_control.M3508_PID[1].out);
				OLED_ShowNum(70,0,GM6020_control.GM6020.all_ecd,5,12);
				OLED_ShowNum(70,10,GM6020_control.GM6020.speed_rpm,4,12);					
			}
		}
		vTaskDelay(1);
	}

}

//////////////pid计算任务/////////////////////
void PID_task(void *pvParameters)
{
	while(1)
	{
		switch (key0_toggle)
		{
			case 0:		//GM6020 PID_toggle
			{
				if(key1_toggle==0)
				{
					LED1=0;
					PID_calc(&pid_control.PID[0],GM6020_control.GM6020.all_ecd,GM6020_control.pid_set_angle_6020);
					PID_calc(&pid_control.PID[1],GM6020_control.GM6020.speed_rpm,pid_control.PID[0].out);
				}
				else
				{
					LED1=1;
					PID_calc(&pid_control.PID[1],GM6020_control.GM6020.speed_rpm,GM6020_control.pid_set_speed_6020);
				}
			}
			break;
			case 1:		//M3508 PID_toggle
			{
				if(key1_toggle==0)
				{
					LED1=0;
					PID_calc(&pid_control.M3508_PID[0],M3508_control.M3508[2].all_ecd,M3508_control.pid_set_angle);
					PID_calc(&pid_control.M3508_PID[1],M3508_control.M3508[2].speed_rpm,pid_control.M3508_PID[0].out);
				}
				else
				{
					LED1=1;
					PID_calc(&pid_control.M3508_PID[1],M3508_control.M3508[2].speed_rpm,M3508_control.pid_set_speed);
				}				
			}
			break;
		}
		vTaskDelay(1);
	}
}

/////////////期望任务////////////
void target_task(void *pvParameters)
{
	GM6020_control.pid_set_angle_6020 = 0;
	GM6020_control.pid_set_speed_6020 = 1000;
	M3508_control.pid_set_angle = 0;
	M3508_control.pid_set_speed = 1000;
	while(1)
	{
		switch(key0_toggle)
		{
			case 0:
			{
				if(key1_toggle==0) GM6020_control.pid_set_angle_6020 += 1000;
				else GM6020_control.pid_set_speed_6020 += 100;
			}
			break;
			case 1:
			{
				if(key1_toggle==0) M3508_control.pid_set_angle += 1000;
				else M3508_control.pid_set_speed += 100;
			}
		}
		vTaskSuspend(targettask_Handler);
		vTaskDelay(1);
	}
}

/////////////////按键扫描任务/////////////
void KEY_task(void *pvParameters)
{
	while(1)
	{
		key = KEY_Scan(0);
		if(key_function==1)
		{
			switch(key)
			{
				case 1:
				{
					if(key0_toggle==0) key0_toggle = 1;
					else key0_toggle = 0;
				}
				break;
				case 2:
				{
					pid_clear(&pid_control.PID[0]);
					pid_clear(&pid_control.PID[1]);
					pid_clear(&pid_control.M3508_PID[0]);
					pid_clear(&pid_control.M3508_PID[1]);
					
					if(key1_toggle==0) key1_toggle = 1;
					else key1_toggle = 0;
				}
				break;
				case 3:
				{
					vTaskResume(targettask_Handler);
					if(key_up_toggle==0) key_up_toggle = 1;
					else key_up_toggle = 0;
				}
				break;
			}
				
		}
		else if(key_function==2)
		{
			LED1 =0;
			vTaskSuspend(targettask_Handler);
			if(key_pid_set==1&&key1_toggle==0&&key0_toggle==0)
			{
//				printf("kp \r");
				switch(key)
				{
					case 1: pid_control.PID[0].Kp++;vTaskResume(usarttask_Handler);break;
					case 2: pid_control.PID[0].Kp--;vTaskResume(usarttask_Handler);break;
					case 3: pid_reset(&pid_control.PID[0],3.0f,0.01f,0.4f);vTaskResume(usarttask_Handler);break;
				}
			}
			else if(key_pid_set==2&&key1_toggle==0&&key0_toggle==0)
			{
//				printf("ki \r");				
				switch(key)
				{
					case 1: pid_control.PID[0].Ki++;vTaskResume(usarttask_Handler);break;
					case 2: pid_control.PID[0].Ki--;vTaskResume(usarttask_Handler);break;
					case 3: pid_reset(&pid_control.PID[0],3.0f,0.01f,0.4f);vTaskResume(usarttask_Handler);break;
				}
			}
			else if(key_pid_set==3&&key1_toggle==0&&key0_toggle==0)
			{
//				printf("kd \r");				
				switch(key)
				{
					case 1: pid_control.PID[0].Kd++;vTaskResume(usarttask_Handler);break;
					case 2: pid_control.PID[0].Kd--;vTaskResume(usarttask_Handler);break;
					case 3: pid_reset(&pid_control.PID[0],3.0f,0.01f,0.4f);vTaskResume(usarttask_Handler);break;
				}
				
			}
			else if(key_pid_set==1&&key1_toggle==1&&key0_toggle==0)
			{
				switch(key)
				{
					case 1: pid_control.PID[1].Kp++;vTaskResume(usarttask_Handler);break;
					case 2: pid_control.PID[1].Kp--;vTaskResume(usarttask_Handler);break;
					case 3: pid_reset(&pid_control.PID[1],6.5f,0.02f,0.1f);vTaskResume(usarttask_Handler);break;
				}
				
			}
			else if(key_pid_set==2&&key1_toggle==1&&key0_toggle==0)
			{
				switch(key)
				{
					case 1: pid_control.PID[1].Ki++;vTaskResume(usarttask_Handler);break;
					case 2: pid_control.PID[1].Ki--;vTaskResume(usarttask_Handler);break;
					case 3: pid_reset(&pid_control.PID[1],6.5f,0.02f,0.1f);vTaskResume(usarttask_Handler);break;
				}
				
			}
			else if(key_pid_set==3&&key1_toggle==1&&key0_toggle==0)
			{
				switch(key)
				{
					case 1: pid_control.PID[1].Kd++;vTaskResume(usarttask_Handler);break;
					case 2: pid_control.PID[1].Kd--;vTaskResume(usarttask_Handler);break;
					case 3: pid_reset(&pid_control.PID[1],6.5f,0.02f,0.1f);vTaskResume(usarttask_Handler);break;
				}
				
			}
			else if(key_pid_set==1&&key1_toggle==0&&key0_toggle==1)
			{
				switch(key)
				{
					case 1: pid_control.M3508_PID[0].Kp++;vTaskResume(usarttask_Handler);break;
					case 2: pid_control.M3508_PID[0].Kp--;vTaskResume(usarttask_Handler);break;
					case 3: pid_reset(&pid_control.M3508_PID[0],0.1f,0.1f,0.1f);vTaskResume(usarttask_Handler);break;		
				}
			}
			else if(key_pid_set==2&&key1_toggle==0&&key0_toggle==1)
			{
				switch(key)
				{
					case 1: pid_control.M3508_PID[0].Ki++;vTaskResume(usarttask_Handler);break;
					case 2: pid_control.M3508_PID[0].Ki--;vTaskResume(usarttask_Handler);break;
					case 3: pid_reset(&pid_control.M3508_PID[0],0.1f,0.1f,0.1f);vTaskResume(usarttask_Handler);break;		
				}
				
			}
			else if(key_pid_set==3&&key1_toggle==0&&key0_toggle==1)
			{
				switch(key)
				{
					case 1: pid_control.M3508_PID[0].Kd++;vTaskResume(usarttask_Handler);break;
					case 2: pid_control.M3508_PID[0].Kd--;vTaskResume(usarttask_Handler);break;
					case 3: pid_reset(&pid_control.M3508_PID[0],0.1f,0.1f,0.1f);vTaskResume(usarttask_Handler);break;		
				}
				
			}
			else if(key_pid_set==1&&key1_toggle==1&&key0_toggle==1)
			{
				switch(key)
				{
					case 1: pid_control.M3508_PID[1].Kp++;vTaskResume(usarttask_Handler);break;
					case 2: pid_control.M3508_PID[1].Kp--;vTaskResume(usarttask_Handler);break;
					case 3: pid_reset(&pid_control.M3508_PID[1],11.0f,1.0f,3.0f);vTaskResume(usarttask_Handler);break;		
				}
				
			}
			else if(key_pid_set==2&&key1_toggle==1&&key0_toggle==1)
			{
				switch(key)
				{
					case 1: pid_control.M3508_PID[1].Ki++;vTaskResume(usarttask_Handler);break;
					case 2: pid_control.M3508_PID[1].Ki--;vTaskResume(usarttask_Handler);break;
					case 3: pid_reset(&pid_control.M3508_PID[1],11.0f,1.0f,3.0f);vTaskResume(usarttask_Handler);break;		
				}
				
			}
			else if(key_pid_set==3&&key1_toggle==1&&key0_toggle==1)
			{
				switch(key)
				{
					case 1: pid_control.M3508_PID[1].Kd++;vTaskResume(usarttask_Handler);break;
					case 2: pid_control.M3508_PID[1].Kd--;vTaskResume(usarttask_Handler);break;
					case 3: pid_reset(&pid_control.M3508_PID[1],11.0f,1.0f,3.0f);vTaskResume(usarttask_Handler);break;		
				}
				
			}
			
		}
		vTaskDelay(1);
	}

}

////////////串口发送任务///////////////
void usart_task(void *pvParameters)
{
	while(1)
	{	
		switch(key0_toggle)
		{
			case 0:
			{		
				OLED_ShowString(0,0,"motor angle; ",12);
				OLED_ShowString(0,10,"motor speed; ",12);
				OLED_ShowString(0,30,"key_function: ",12);
				OLED_ShowString(0,40,"pid_set: ",12);
				printf("GM6020 is running \r");
				printf("角度环 kp= %f ki= %f kd= %f \r", pid_control.PID[0].Kp, pid_control.PID[0].Ki, pid_control.PID[0].Kd);
				printf("速度环 kp= %f ki= %f kd= %f \r", pid_control.PID[1].Kp, pid_control.PID[1].Ki, pid_control.PID[1].Kd);	
			}
			break;
			case 1:
			{
				OLED_ShowString(0,0,"motor angle; ",12);
				OLED_ShowString(0,10,"motor speed; ",12);
				OLED_ShowString(0,30,"key_function: ",12);
				printf("M3508 is running \r");
				printf("角度环 kp= %f ki= %f kd= %f \r", pid_control.M3508_PID[0].Kp, pid_control.M3508_PID[0].Ki, pid_control.M3508_PID[0].Kd);
				printf("速度环 kp= %f ki= %f kd= %f \r", pid_control.M3508_PID[1].Kp, pid_control.M3508_PID[1].Ki, pid_control.M3508_PID[1].Kd);	
			}
			break;
		}
		vTaskSuspend(usarttask_Handler);
		vTaskDelay(1);
	}
}



