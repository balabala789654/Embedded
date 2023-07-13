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

#define START_TASK_PRIO	1		//���ȼ�
#define START_STK_SIZE	128		//��ջ
TaskHandle_t StartTask_Handler;	//���
void start_task(void *pvParameters);

#define motor_TASK_PRIO	28		//���ȼ�
#define motor_STK_SIZE	128		//��ջ
static TaskHandle_t motortask_Handler;	//���
void motor_task(void *pvParameters);

#define PID_TASK_PRIO	29		//���ȼ�
#define PID_STK_SIZE	128		//��ջ
TaskHandle_t PIDtask_Handler;	//���
void PID_task(void *pvParameters);

#define target_TASK_PRIO 27		//���ȼ�
#define target_STK_SIZE	128		//��ջ
TaskHandle_t targettask_Handler;	//���
void target_task(void *pvParameters);

#define KEY_TASK_PRIO	30		//���ȼ�
#define KEY_STK_SIZE	128		//��ջ
static TaskHandle_t Keytask_Handler;	//���
void KEY_task(void *pvParameters);

#define usart_TASK_PRIO	30		//���ȼ�
#define usart_STK_SIZE	128		//��ջ
TaskHandle_t usarttask_Handler;	//���
void usart_task(void *pvParameters);

PID pid_control;
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4	 
	delay_init();	    				//��ʱ������ʼ��	 
	OLED_Init();
	KEY_Init();
	LED_Init();
	uart_init(115200);
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,2,CAN_Mode_Normal);
	PID_init(&pid_control.PID[0],PID_POSITION,3.0f,0.01f,0.4f,16000,100,200);
	PID_init(&pid_control.PID[1],PID_POSITION,6.5f,0.02f,0.1f,30000,200,200);
	PID_init(&pid_control.M3508_PID[0],PID_POSITION,0.1f,0.1f,0.1f,1000,100,1000);
	PID_init(&pid_control.M3508_PID[1],PID_POSITION,11.0f,1.0f,3.0f,16000,200,200);

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
	
	xTaskCreate((TaskFunction_t)motor_task,				//������
				(const char*   )"motor_task",			//����
				(uint16_t	   )motor_STK_SIZE,			//��ջ
				(void*         )NULL,					//���ݲ���
				(UBaseType_t   )motor_TASK_PRIO,		//���ȼ�
				(TaskHandle_t* )&motortask_Handler);	//���

	xTaskCreate((TaskFunction_t)PID_task,				//������
				(const char*   )"PID_task",				//����
				(uint16_t	   )PID_STK_SIZE,			//��ջ
				(void*         )NULL,					//���ݲ���
				(UBaseType_t   )PID_TASK_PRIO,			//���ȼ�
				(TaskHandle_t* )&PIDtask_Handler);		//���

	xTaskCreate((TaskFunction_t)target_task,			//������
				(const char*   )"target_task",			//����
				(uint16_t	   )target_STK_SIZE,		//��ջ
				(void*         )NULL,					//���ݲ���
				(UBaseType_t   )target_TASK_PRIO,		//���ȼ�
				(TaskHandle_t* )&targettask_Handler)	;//���

	xTaskCreate((TaskFunction_t)KEY_task,				//������
				(const char*   )"KEY_task",				//����
				(uint16_t	   )KEY_STK_SIZE,			//��ջ
				(void*         )NULL,					//���ݲ���
				(UBaseType_t   )KEY_TASK_PRIO,			//���ȼ�
				(TaskHandle_t* )&Keytask_Handler);		//���
				
	xTaskCreate((TaskFunction_t)usart_task,				//������
				(const char*   )"usart_task",			//����
				(uint16_t	   )usart_STK_SIZE,			//��ջ
				(void*         )NULL,					//���ݲ���
				(UBaseType_t   )usart_TASK_PRIO,		//���ȼ�
				(TaskHandle_t* )&usarttask_Handler);	//���

    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
	
}





