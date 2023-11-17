#include "main.h"

#define START_TASK_PRIO	10		//优先级
#define START_STK_SIZE	128		//堆栈
static TaskHandle_t StartTask_Handler;	//句柄
void start_task(void *pvParameters);

#define TASK1_TASK_PRIO	25		//优先级
#define TASK1_STK_SIZE	128		//堆栈
static TaskHandle_t task1Task_Handler;	//句柄

#define INTERACTION_TASK_PRIO	30		//优先级
#define INTERACTION_STK_SIZE	128		//堆栈
static TaskHandle_t interactionTask_Handler;	//句柄

#define MPU_TASK_PRIO	28		//优先级
#define MPU_STK_SIZE	128		//堆栈
static TaskHandle_t MPUTask_Handler;	//句柄

PID pid_control;
extern M3508 M3508_control;
extern GM6020 GM6020_control;
int8_t SendBuff[20]={0};

int main()
{   
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4	
	TIM3_Int_Init(100-1,8400-1);	//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数100次为10ms
	delay_init(168);					//初始化延时函数
	LED_Init();
	remote_control_init();
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	PID_init(&pid_control.M3508_PID[0],PID_POSITION,7.0f,1.0f,0.0f,8000,200,200);
	PID_init(&pid_control.M3508_PID[1],PID_POSITION,7.0f,1.0f,0.0f,8000,200,200);
	PID_init(&pid_control.M3508_PID[2],PID_POSITION,7.0f,1.0f,0.0f,8000,200,200);
	PID_init(&pid_control.M3508_PID[3],PID_POSITION,7.0f,1.0f,0.0f,8000,200,200);
	uart_init(115200);
	//MPU_Init();
	//while(mpu_dmp_init());

	
	
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
				(TaskHandle_t* )&task1Task_Handler);//句柄

	xTaskCreate((TaskFunction_t)interaction_task,			//任务函数
				(const char*   )"interaction_task",		//名称
				(uint16_t	   )INTERACTION_STK_SIZE,		//堆栈
				(void*         )NULL,				//传递参数
				(UBaseType_t   )INTERACTION_TASK_PRIO,	//优先级
				(TaskHandle_t* )&interactionTask_Handler);//句柄

	xTaskCreate((TaskFunction_t)MPU_task,			//任务函数
				(const char*   )"MPU_task",		//名称
				(uint16_t	   )MPU_STK_SIZE,		//堆栈
				(void*         )NULL,				//传递参数
				(UBaseType_t   )MPU_TASK_PRIO,	//优先级
				(TaskHandle_t* )&MPUTask_Handler);//句柄
				
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区

}













