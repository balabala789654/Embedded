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


#define START_TASK_PRIO	1		//���ȼ�
#define START_STK_SIZE	128		//��ջ
TaskHandle_t StartTask_Handler;	//���
void start_task(void *pvParameters);

#define TASK1_TASK_PRIO	30		//���ȼ�
#define TASK1_STK_SIZE	128		//��ջ
static TaskHandle_t Task1task_Handler;	//���
void task1_task(void *pvParameters);

#define TASK2_TASK_PRIO	29		//���ȼ�
#define TASK2_STK_SIZE	128		//��ջ
static TaskHandle_t Task2task_Handler;	//���
void task2_task(void *pvParameters);

#define TASK3_TASK_PRIO	28		//���ȼ�
#define TASK3_STK_SIZE	128		//��ջ
static TaskHandle_t Task3task_Handler;	//���
void task3_task(void *pvParameters);

PID pid_control;
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4	 
	delay_init();	    				//��ʱ������ʼ��	 
	OLED_Init();
	MPU_Init();
	mpu_dmp_init();
	LED_Init();
	uart_init(115200);
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,2,CAN_Mode_Normal);
	PID_init(&pid_control.PID[0],PID_POSITION,0.1f,0.1f,0.1f,1000,100,1000);
	PID_init(&pid_control.PID[1],PID_POSITION,11.0f,1.0f,3.0f,16000,200,200);
	
	xTaskCreate((TaskFunction_t)start_task,			//������
				(const char*   )"start_task",		//����
				(uint16_t	   )START_STK_SIZE,		//��ջ
				(void*         )NULL,				//���ݲ���
				(UBaseType_t   )START_TASK_PRIO,	//���ȼ�
				(TaskHandle_t* )&StartTask_Handler);//���
	vTaskStartScheduler();		//��������
	while(1);
}

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
	
	xTaskCreate((TaskFunction_t)task1_task,			//������
				(const char*   )"task1_task",		//����
				(uint16_t	   )TASK1_STK_SIZE,		//��ջ
				(void*         )NULL,				//���ݲ���
				(UBaseType_t   )TASK1_TASK_PRIO,	//���ȼ�
				(TaskHandle_t* )&Task1task_Handler);//���

	xTaskCreate((TaskFunction_t)task2_task,			//������
				(const char*   )"task2_task",		//����
				(uint16_t	   )TASK2_STK_SIZE,		//��ջ
				(void*         )NULL,				//���ݲ���
				(UBaseType_t   )TASK2_TASK_PRIO,	//���ȼ�
				(TaskHandle_t* )&Task2task_Handler);//���

	xTaskCreate((TaskFunction_t)task3_task,			//������
				(const char*   )"task3_task",		//����
				(uint16_t	   )TASK3_STK_SIZE,		//��ջ
				(void*         )NULL,				//���ݲ���
				(UBaseType_t   )TASK3_TASK_PRIO,	//���ȼ�
				(TaskHandle_t* )&Task3task_Handler);//���
				
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
	
}





