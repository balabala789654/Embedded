#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"  
#include "dma.h"


//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.

u8 SendBuff[SEND_BUF_SIZE];	//发送数据缓冲区

  
int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);     //初始化延时函数
	uart_init(115200);	//初始化串口波特率为115200
	LED_Init();					//初始化LED 
	KEY_Init(); 				//按7初始化 
 	MYDMA_Config(DMA2_Stream2,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff,SEND_BUF_SIZE);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //使能串口1的DMA发送.
	while(1)
	{
//		for()
//		{
//			
//		}
		if(DMA_GetFlagStatus(DMA2_Stream2,DMA_FLAG_TCIF2)!=RESET)//等待DMA2_Steam7传输完成
		{ 
			DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);//清除DMA2_Steam7传输完成标志
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

