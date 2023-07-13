#include "stm32f10x.h"                  // Device header
#include "pid.h"
#include "can.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "TASK.h"
#include "oled.h"
#include "led.h"
#include "usart.h"	
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 


#define START_TASK_PRIO	1		//优先级
#define START_STK_SIZE	128		//堆栈
TaskHandle_t StartTask_Handler;	//句柄
void start_task(void *pvParameters);

#define TASK1_TASK_PRIO	30		//优先级
#define TASK1_STK_SIZE	128		//堆栈
static TaskHandle_t Task1task_Handler;	//句柄
void task1_task(void *pvParameters);

#define TASK2_TASK_PRIO	29		//优先级
#define TASK2_STK_SIZE	128		//堆栈
static TaskHandle_t Task2task_Handler;	//句柄
void task2_task(void *pvParameters);

#define TASK3_TASK_PRIO	28		//优先级
#define TASK3_STK_SIZE	128		//堆栈
static TaskHandle_t Task3task_Handler;	//句柄
void task3_task(void *pvParameters);

PID pid_control;
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4	 
	delay_init();	    				//延时函数初始化	 
	OLED_Init();
	MPU_Init();
	mpu_dmp_init();
	LED_Init();
	uart_init(115200);
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,2,CAN_Mode_Normal);
	PID_init(&pid_control.PID[0],PID_POSITION,0.1f,0.1f,0.1f,1000,100,1000);
	PID_init(&pid_control.PID[1],PID_POSITION,11.0f,1.0f,3.0f,16000,200,200);
	
	xTaskCreate((TaskFunction_t)start_task,			//任务函数
				(const char*   )"start_task",		//名称
				(uint16_t	   )START_STK_SIZE,		//堆栈
				(void*         )NULL,				//传递参数
				(UBaseType_t   )START_TASK_PRIO,	//优先级
				(TaskHandle_t* )&StartTask_Handler);//句柄
	vTaskStartScheduler();		//开启调度
	while(1);
}

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
	
	xTaskCreate((TaskFunction_t)task1_task,			//任务函数
				(const char*   )"task1_task",		//名称
				(uint16_t	   )TASK1_STK_SIZE,		//堆栈
				(void*         )NULL,				//传递参数
				(UBaseType_t   )TASK1_TASK_PRIO,	//优先级
				(TaskHandle_t* )&Task1task_Handler);//句柄

	xTaskCreate((TaskFunction_t)task2_task,			//任务函数
				(const char*   )"task2_task",		//名称
				(uint16_t	   )TASK2_STK_SIZE,		//堆栈
				(void*         )NULL,				//传递参数
				(UBaseType_t   )TASK2_TASK_PRIO,	//优先级
				(TaskHandle_t* )&Task2task_Handler);//句柄

	xTaskCreate((TaskFunction_t)task3_task,			//任务函数
				(const char*   )"task3_task",		//名称
				(uint16_t	   )TASK3_STK_SIZE,		//堆栈
				(void*         )NULL,				//传递参数
				(UBaseType_t   )TASK3_TASK_PRIO,	//优先级
				(TaskHandle_t* )&Task3task_Handler);//句柄
				
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
	
}





