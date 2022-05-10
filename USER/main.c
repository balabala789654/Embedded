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


u32 massge[2];														//���ڽ�������
u16 canbuf[2]={0,0};											//can�ķ�������
u16 R_canbuf[2]={0,0};										//can�Ľ�������
u8 task1_Status,task2_Status;							//����Ĺ��������е�״̬����    
u8 k,mode;																//k Ϊ�����Ƿ���±���  k=1��Ϊ�����Ѹ��� k=0��Ϊ����û�и���   mode Ϊ����״̬���� mode=1ʱ ����һ���� =2ʱ ����2���� 
u8 i=0;
u8 can_mode;															//�ж�can�Ƿ���з��͵Ĳ���

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	usart3_Init();
	LED_Init();	
	KEY_Init();
	can1_Init();
	EXTIX_Init();
	BEEP_Init();
		
	/////////     ����1�Ĵ���       //////////
  xTaskCreate(  (TaskFunction_t )TASK1_T,            //������
                (const char*    )"task1",       		 //��������
                (uint16_t       )128,       				 //�����ջ��С
                (void*          )NULL,               //���ݸ��������Ĳ���
                (UBaseType_t    )3,       					 //�������ȼ�
                (TaskHandle_t*  )&task1_Handler);		 //������ 
						
	/////////     ����2�Ĵ���       //////////
  xTaskCreate(  (TaskFunction_t )TASK2_T,            //������
                (const char*    )"task2",       		 //��������
                (uint16_t       )128,       				 //�����ջ��С
                (void*          )NULL,               //���ݸ��������Ĳ���
                (UBaseType_t    )2,       					 //�������ȼ�
                (TaskHandle_t*  )&task2_Handler);		 //������  

	/////////     ����3�Ĵ���       //////////
  xTaskCreate(  (TaskFunction_t )TASK3_T,            //������
                (const char*    )"task3",       		 //��������
                (uint16_t       )128,       				 //�����ջ��С
                (void*          )NULL,               //���ݸ��������Ĳ���
                (UBaseType_t    )1,       					 //�������ȼ�
                (TaskHandle_t*  )&task3_Handler);		 //������  
								
	vTaskStartScheduler();
}

void TASK1_T(void *pvParameters)
{	
	
	printf("task1 is running \r\n");
	task1_Status=task1_run;															//task1 Ϊ����̬
	while(1)
	{
		vTaskDelay(10);
		if(massge[0]==0x7A&&massge[1]==0x10)          /*** LED 1����˸1�� ***/
		{
			LED0=0;
			vTaskDelay(10);
			LED0=1;
			vTaskDelay(990);
			
			if(canbuf[1]!=0x74||k==1) can_mode=1;
			else can_mode=0;
			if(can_mode)
			{
				canbuf[0]=massge[0]^0x64;                      /*  CAN�ķ���   */
				canbuf[1]=massge[1]^0x64;
				can_Send(canbuf,2);	
				k=0;				
			}
		}
		else if(massge[0]==0x7A&&massge[1]==0x20)     /*** LED 1����˸2�� ***/
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
				canbuf[0]=massge[0]^0x64;                      /*  CAN�ķ���   */
				canbuf[1]=massge[1]^0x64;
				can_Send(canbuf,2);
				k=0;
			}
		}
		else if(massge[0]==0x7A&&massge[1]==0x30)      /*** LED 1����˸4�� ***/
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
				canbuf[0]=massge[0]^0x64;                      /*  CAN�ķ���   */
				canbuf[1]=massge[1]^0x64;
				can_Send(canbuf,2);
				k=0;
			}
			
		}

	}
}


void TASK2_T(void *pvParameters)
{
	task2_Status=task2_run;															//task2 Ϊ����̬
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
			if(task1_Status==task1_sus&&task2_Status==task2_sus)			//task1 �� task2 ͬʱΪ����̬
			{
				while(BT)
				{		
					BEEP=1;
					vTaskDelay(1000);
					BEEP=0;
					BT--;
					printf("�ָ�����1������2\r\n");
					printf("LED0 ��LED1 �������������·�������\r\n");
				}
				vTaskResume(task1_Handler);															//�ָ�task1
				vTaskResume(task2_Handler);															//�ָ�task2
				task1_Status=task1_run;
				task2_Status=task2_run; 
				LED1=0;
				LED0=0;
			}
			else if(mode==1&&task1_Status==task1_run)									//task1 ����ʱ�ָ�task1
			{
  			LED0=0;
				mode=3;
				printf("LED0���������·������ݼ��ɻָ�Ƶ��\r\n");
			}
			else if(mode==2&&task2_Status==task2_run)									//task2 ����ʱ�ָ�task2
			{
				LED1=0;
				R_canbuf[0]=0;
				R_canbuf[1]=0;
				mode=3;
				printf("LED1���������·������ݼ��ɻָ�Ƶ��\r\n");
				
			}
		}
}
















