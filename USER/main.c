#include "stm32f10x.h"                  // Device header
#include "time.h"
#include "led.h"
#include "usart1.h"
#include "key.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "can.h"
#include "exti.h"
#include "beep.h"
#include "usart.h"

void TASK1_T(void *pvParameters);
TaskHandle_t task1_Handler;

void TASK2_T(void *pvParameters);
TaskHandle_t task2_Handler;

void TASK3_T(void *pvParameters);
TaskHandle_t task3_Handler;


u32 massge[2];														//串口接受数组
u16 canbuf[2]={0,0};											//can的发送数组
u16 R_canbuf[2]={0,0};										//can的接受数组
u8 task1_Status,task2_Status;							//任务的挂起与运行的状态变量    
u8 k,mode;																//k 为数据是否更新变量  k=1即为数据已更新 k=0即为数据没有更新   mode 为任务状态变量 mode=1时 任务一挂起 =2时 任务2挂起 
u8 i=0;
u8 can_mode;															//判断can是否进行发送的参数

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	usart3_Init();
	LED_Init();	
	KEY_Init();
	can1_Init();
	EXTIX_Init();
	BEEP_Init();
		
	/////////     任务1的创建       //////////
  xTaskCreate(  (TaskFunction_t )TASK1_T,            //任务函数
                (const char*    )"task1",       		 //任务名称
                (uint16_t       )128,       				 //任务堆栈大小
                (void*          )NULL,               //传递给任务函数的参数
                (UBaseType_t    )3,       					 //任务优先级
                (TaskHandle_t*  )&task1_Handler);		 //任务句柄 
						
	/////////     任务2的创建       //////////
  xTaskCreate(  (TaskFunction_t )TASK2_T,            //任务函数
                (const char*    )"task2",       		 //任务名称
                (uint16_t       )128,       				 //任务堆栈大小
                (void*          )NULL,               //传递给任务函数的参数
                (UBaseType_t    )2,       					 //任务优先级
                (TaskHandle_t*  )&task2_Handler);		 //任务句柄  

	/////////     任务3的创建       //////////
  xTaskCreate(  (TaskFunction_t )TASK3_T,            //任务函数
                (const char*    )"task3",       		 //任务名称
                (uint16_t       )128,       				 //任务堆栈大小
                (void*          )NULL,               //传递给任务函数的参数
                (UBaseType_t    )1,       					 //任务优先级
                (TaskHandle_t*  )&task3_Handler);		 //任务句柄  
								
	vTaskStartScheduler();
}

void TASK1_T(void *pvParameters)
{	
	
	printf("task1 is running \r\n");
	task1_Status=task1_run;															//task1 为运行态
	while(1)
	{
		vTaskDelay(10);
		if(massge[0]==0x7A&&massge[1]==0x10)          /*** LED 1秒闪烁1次 ***/
		{
			LED0=0;
			vTaskDelay(10);
			LED0=1;
			vTaskDelay(990);
			
			if(canbuf[1]!=0x74||k==1) can_mode=1;
			else can_mode=0;
			if(can_mode)
			{
				canbuf[0]=massge[0]^0x64;                      /*  CAN的发送   */
				canbuf[1]=massge[1]^0x64;
				can_Send(canbuf,2);	
				k=0;				
			}
		}
		else if(massge[0]==0x7A&&massge[1]==0x20)     /*** LED 1秒闪烁2次 ***/
		{

			LED0=0;
			vTaskDelay(10);
			LED0=1;
			vTaskDelay(100);
			LED0=0;
			vTaskDelay(10);
			LED0=1;
			vTaskDelay(880);	
			if(canbuf[1]!=0x44||k==1) can_mode=1;
			else can_mode=0;

			if(can_mode)
			{			
				canbuf[0]=massge[0]^0x64;                      /*  CAN的发送   */
				canbuf[1]=massge[1]^0x64;
				can_Send(canbuf,2);
				k=0;
			}
		}
		else if(massge[0]==0x7A&&massge[1]==0x30)      /*** LED 1秒闪烁4次 ***/
		{			
			LED0=0;
			vTaskDelay(10);
			LED0=1;
			vTaskDelay(100);
			LED0=0;
			vTaskDelay(10);
			LED0=1;
			vTaskDelay(100);
			LED0=0;
			vTaskDelay(10);
			LED0=1;
			vTaskDelay(100);
			LED0=0;
			vTaskDelay(10);
			LED0=1;
			vTaskDelay(660);
			
			if(canbuf[1]!=0x54||k==1) can_mode=1;
			else can_mode=0;
			if(can_mode)
			{
				canbuf[0]=massge[0]^0x64;                      /*  CAN的发送   */
				canbuf[1]=massge[1]^0x64;
				can_Send(canbuf,2);
				k=0;
			}
			
		}

	}
}


void TASK2_T(void *pvParameters)
{
	task2_Status=task2_run;															//task2 为运行态
	printf("task2 is running\r\n");
	while(1)
	{
		vTaskDelay(10);
		if(R_canbuf[0]==0x7A&&R_canbuf[1]==0x10)
		{
			LED1=0;
			vTaskDelay(10);
			LED1=1;
			vTaskDelay(990);

		}
		else if(R_canbuf[0]==0x7A&&R_canbuf[1]==0x20)
		{
			LED1=0;
			vTaskDelay(10);
			LED1=1;
			vTaskDelay(100);
			LED1=0;
			vTaskDelay(10);
			LED1=1;
			vTaskDelay(880);
			
		}
		else if(R_canbuf[0]==0x7A&&R_canbuf[1]==0x30)
		{
			LED1=0;
			vTaskDelay(10);
			LED1=1;
			vTaskDelay(100);
			LED1=0;
			vTaskDelay(10);
			LED1=1;
			vTaskDelay(100);
			LED1=0;
			vTaskDelay(10);
			LED1=1;
			vTaskDelay(100);
			LED1=0;
			vTaskDelay(10);
			LED1=1;
			vTaskDelay(660);
			
		}
	}
}


void TASK3_T(void *pvParameters)
{
		while(1)
		{
			if(i==2) i=0;
			u8 BT=1;
			if(task1_Status==task1_sus&&task2_Status==task2_sus)			//task1 与 task2 同时为挂起态
			{
				while(BT)
				{		
					BEEP=1;
					vTaskDelay(1000);
					BEEP=0;
					BT--;
					printf("恢复任务1与任务2\r\n");
					printf("LED0 与LED1 均常亮，需重新发送数据\r\n");
				}
				vTaskResume(task1_Handler);															//恢复task1
				vTaskResume(task2_Handler);															//恢复task2
				task1_Status=task1_run;
				task2_Status=task2_run; 
				LED1=0;
				LED0=0;
			}
			else if(mode==1&&task1_Status==task1_run)									//task1 挂起时恢复task1
			{
  			LED0=0;
				mode=3;
				printf("LED0常亮，重新发送数据即可恢复频闪\r\n");
			}
			else if(mode==2&&task2_Status==task2_run)									//task2 挂起时恢复task2
			{
				LED1=0;
				R_canbuf[0]=0;
				R_canbuf[1]=0;
				mode=3;
				printf("LED1常亮，重新发送数据即可恢复频闪\r\n");
				
			}
		}
}
















