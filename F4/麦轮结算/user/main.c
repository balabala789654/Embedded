#include "main.h"

#define START_TASK_PRIO	10		//���ȼ�
#define START_STK_SIZE	128		//��ջ
static TaskHandle_t StartTask_Handler;	//���
void start_task(void *pvParameters);

#define TASK1_TASK_PRIO	25		//���ȼ�
#define TASK1_STK_SIZE	128		//��ջ
static TaskHandle_t task1Task_Handler;	//���

#define INTERACTION_TASK_PRIO	30		//���ȼ�
#define INTERACTION_STK_SIZE	128		//��ջ
static TaskHandle_t interactionTask_Handler;	//���

#define MPU_TASK_PRIO	28		//���ȼ�
#define MPU_STK_SIZE	128		//��ջ
static TaskHandle_t MPUTask_Handler;	//���

PID pid_control;
extern M3508 M3508_control;
extern GM6020 GM6020_control;
int8_t SendBuff[20]={0};

int main()
{   
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4	
	TIM3_Int_Init(100-1,8400-1);	//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����100��Ϊ10ms
	delay_init(168);					//��ʼ����ʱ����
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
				(TaskHandle_t* )&task1Task_Handler);//���

	xTaskCreate((TaskFunction_t)interaction_task,			//������
				(const char*   )"interaction_task",		//����
				(uint16_t	   )INTERACTION_STK_SIZE,		//��ջ
				(void*         )NULL,				//���ݲ���
				(UBaseType_t   )INTERACTION_TASK_PRIO,	//���ȼ�
				(TaskHandle_t* )&interactionTask_Handler);//���

	xTaskCreate((TaskFunction_t)MPU_task,			//������
				(const char*   )"MPU_task",		//����
				(uint16_t	   )MPU_STK_SIZE,		//��ջ
				(void*         )NULL,				//���ݲ���
				(UBaseType_t   )MPU_TASK_PRIO,	//���ȼ�
				(TaskHandle_t* )&MPUTask_Handler);//���
				
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���

}













