#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"  
#include "dma.h"


//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.

u8 SendBuff[SEND_BUF_SIZE];	//�������ݻ�����

  
int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);     //��ʼ����ʱ����
	uart_init(115200);	//��ʼ�����ڲ�����Ϊ115200
	LED_Init();					//��ʼ��LED 
	KEY_Init(); 				//��7��ʼ�� 
 	MYDMA_Config(DMA2_Stream2,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff,SEND_BUF_SIZE);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //ʹ�ܴ���1��DMA����.
	while(1)
	{
//		for()
//		{
//			
//		}
		if(DMA_GetFlagStatus(DMA2_Stream2,DMA_FLAG_TCIF2)!=RESET)//�ȴ�DMA2_Steam7�������
		{ 
			DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);//���DMA2_Steam7������ɱ�־
			printf("transfer well down\r\n");
			for(int i=0; i<SEND_BUF_SIZE; i++)
			{
				printf("SendBuff[%d]--%x\r\n",i, SendBuff[i]);
				
			}
		}		

		LED0=0;
		delay_ms(50);
		LED0=1;
		delay_ms(50);
	}		    
}

