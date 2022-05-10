#include "exti.h"
#include "key.h"
#include "delay.h"
#include "beep.h"
#include "FreeRTOS.h"
#include "task.h"
#include "beep.h"
#include "led.h"

void EXTIX_Init(void)
{
	EXTI_InitTypeDef EXTI_Stru;
	NVIC_InitTypeDef NVIC_Stru;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource3);
	EXTI_Stru.EXTI_Line = EXTI_Line3;
	EXTI_Stru.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_Stru.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Stru.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_Stru);                 //GPIOE.3
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource4);
	EXTI_Stru.EXTI_Line=EXTI_Line4;
	EXTI_Init(&EXTI_Stru);                 //GPIOE.4
		
	NVIC_Stru.NVIC_IRQChannel=EXTI3_IRQn;
	NVIC_Stru.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Stru.NVIC_IRQChannelPreemptionPriority=0x08;
	NVIC_Stru.NVIC_IRQChannelSubPriority=0x00;
	NVIC_Init(&NVIC_Stru);
	
	NVIC_Stru.NVIC_IRQChannel=EXTI4_IRQn;
	NVIC_Stru.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Stru.NVIC_IRQChannelPreemptionPriority=0x07;
	NVIC_Stru.NVIC_IRQChannelSubPriority=0x00;
	NVIC_Init(&NVIC_Stru);

}

extern void TASK1_T(void *pvParameters);
extern TaskHandle_t task1_Handler;
extern TaskHandle_t task2_Handler;
extern u8 task1_Status,task2_Status;
extern u8 task3_L0;
extern u8 mode;
extern u32 massge[2];

void EXTI3_IRQHandler(void)
{
	BaseType_t YieldRequired1;
	u8 status_value;
	status_value=taskENTER_CRITICAL_FROM_ISR();	
	if(KEY1==0)
	{
		while(KEY1==0);
		printf("KEY1按下\r\n");
		switch(task2_Status)
		{
			case task2_run:vTaskSuspend(task2_Handler);printf("任务2挂起\r\n");task2_Status=task2_sus;break;
			case task2_sus:YieldRequired1=xTaskResumeFromISR(task2_Handler);printf("任务2恢复 \r\n");task2_Status=task2_run;break;
		}
		if(YieldRequired1==pdTRUE)
		{
			portYIELD_FROM_ISR(YieldRequired1);
		}
		if(task2_Status==task2_sus)
		{	
			LED1=1;
			mode=2;
		}

	}
	EXTI_ClearITPendingBit(EXTI_Line3);
	taskEXIT_CRITICAL_FROM_ISR(status_value);	
}
void EXTI4_IRQHandler(void)
{
	BaseType_t YieldRequired;
	u8 status_value;
	status_value=taskENTER_CRITICAL_FROM_ISR();
	if(KEY0==0)
	{	
		while(KEY0==0);
		printf("KEY0按下\r\n");
		switch(task1_Status)
		{
			case task1_run:vTaskSuspend(task1_Handler);printf("任务1挂起\r\n");task1_Status=task1_sus;break;
			case task1_sus:YieldRequired=xTaskResumeFromISR(task1_Handler);printf("任务1恢复 \r\n");task1_Status=task1_run;break;
		}
		if(YieldRequired==pdTRUE)
		{
			portYIELD_FROM_ISR(YieldRequired);
		}
		if(task1_Status==task1_sus)
		{	
			mode=1;
			massge[1]=0;
			massge[0]=0;
			LED0=1;
		}			
	}
	
	EXTI_ClearITPendingBit(EXTI_Line4);
	taskEXIT_CRITICAL_FROM_ISR(status_value);
}






