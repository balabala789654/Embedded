#include "stm32f10x.h"                  // Device header
#include "pid.h"
#include "can.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "oled.h"
#include "led.h"
#include "usart.h"	  
#include "key.h"

#define START_TASK_PRIO	1		//优先级
#define START_STK_SIZE	128		//堆栈
TaskHandle_t StartTask_Handler;	//句柄
void start_task(void *pvParameters);

#define motor_TASK_PRIO	28		//优先级
#define motor_STK_SIZE	128		//堆栈
static TaskHandle_t motortask_Handler;	//句柄
void motor_task(void *pvParameters);

#define PID_TASK_PRIO	29		//优先级
#define PID_STK_SIZE	128		//堆栈
TaskHandle_t PIDtask_Handler;	//句柄
void PID_task(void *pvParameters);

#define target_TASK_PRIO 27		//优先级
#define target_STK_SIZE	128		//堆栈
TaskHandle_t targettask_Handler;	//句柄
void target_task(void *pvParameters);

#define KEY_TASK_PRIO	30		//优先级
#define KEY_STK_SIZE	128		//堆栈
static TaskHandle_t Keytask_Handler;	//句柄
void KEY_task(void *pvParameters);

#define usart_TASK_PRIO	30		//优先级
#define usart_STK_SIZE	128		//堆栈
TaskHandle_t usarttask_Handler;	//句柄
void usart_task(void *pvParameters);

PID pid_control;
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4	 
	delay_init();	    				//延时函数初始化	 
	OLED_Init();
	KEY_Init();
	LED_Init();
	uart_init(115200);
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,2,CAN_Mode_Normal);
	PID_init(&pid_control.PID[0],PID_POSITION,3.0f,0.01f,0.4f,16000,100,200);
	PID_init(&pid_control.PID[1],PID_POSITION,6.5f,0.02f,0.1f,30000,200,200);
	PID_init(&pid_control.M3508_PID[0],PID_POSITION,0.1f,0.1f,0.1f,1000,100,1000);
	PID_init(&pid_control.M3508_PID[1],PID_POSITION,11.0f,1.0f,3.0f,16000,200,200);

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
	
	xTaskCreate((TaskFunction_t)motor_task,				//任务函数
				(const char*   )"motor_task",			//名称
				(uint16_t	   )motor_STK_SIZE,			//堆栈
				(void*         )NULL,					//传递参数
				(UBaseType_t   )motor_TASK_PRIO,		//优先级
				(TaskHandle_t* )&motortask_Handler);	//句柄

	xTaskCreate((TaskFunction_t)PID_task,				//任务函数
				(const char*   )"PID_task",				//名称
				(uint16_t	   )PID_STK_SIZE,			//堆栈
				(void*         )NULL,					//传递参数
				(UBaseType_t   )PID_TASK_PRIO,			//优先级
				(TaskHandle_t* )&PIDtask_Handler);		//句柄

	xTaskCreate((TaskFunction_t)target_task,			//任务函数
				(const char*   )"target_task",			//名称
				(uint16_t	   )target_STK_SIZE,		//堆栈
				(void*         )NULL,					//传递参数
				(UBaseType_t   )target_TASK_PRIO,		//优先级
				(TaskHandle_t* )&targettask_Handler)	;//句柄

	xTaskCreate((TaskFunction_t)KEY_task,				//任务函数
				(const char*   )"KEY_task",				//名称
				(uint16_t	   )KEY_STK_SIZE,			//堆栈
				(void*         )NULL,					//传递参数
				(UBaseType_t   )KEY_TASK_PRIO,			//优先级
				(TaskHandle_t* )&Keytask_Handler);		//句柄
				
	xTaskCreate((TaskFunction_t)usart_task,				//任务函数
				(const char*   )"usart_task",			//名称
				(uint16_t	   )usart_STK_SIZE,			//堆栈
				(void*         )NULL,					//传递参数
				(UBaseType_t   )usart_TASK_PRIO,		//优先级
				(TaskHandle_t* )&usarttask_Handler);	//句柄

    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
	
}





