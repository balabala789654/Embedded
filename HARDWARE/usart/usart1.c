#include "usart1.h"
#include "led.h"
#include "FreeRTOS.h"
#include "stm32f10x.h"                  // Device header
#include "stdio.h"
#include "usart.h"
#include "task.h"

void usart3_Init(void)
{
	GPIO_InitTypeDef GPIO_Stru;
	USART_InitTypeDef USART_Stru;
	NVIC_InitTypeDef NVIC_Stru;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	USART_DeInit(USART3);
	GPIO_Stru.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Stru.GPIO_Pin=GPIO_Pin_10;
	GPIO_Stru.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Stru);
	
	GPIO_Stru.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Stru.GPIO_Pin=GPIO_Pin_11;
	GPIO_Stru.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Stru);
	
	USART_Stru.USART_BaudRate=50000;
	USART_Stru.USART_WordLength=USART_WordLength_8b;
	USART_Stru.USART_StopBits=USART_StopBits_1;
	USART_Stru.USART_Parity=USART_Parity_No;
	USART_Stru.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_Stru.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_Init(USART3,&USART_Stru);
	
	NVIC_Stru.NVIC_IRQChannel=USART3_IRQn;
	NVIC_Stru.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Stru.NVIC_IRQChannelPreemptionPriority=4;
	NVIC_Stru.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_Stru);
	
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);

	USART_Cmd(USART3,ENABLE);
}


extern u32 massge[USART_REC_LEN];
extern u8 i,k;
void USART3_IRQHandler(void)															//串口中断 接受数据到massge[]
{
	u8 Res,status_value;	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{   
		status_value=taskENTER_CRITICAL_FROM_ISR();						//进入临界 保护中断
		Res =USART_ReceiveData(USART3);	//读取接收到的数据	
		massge[i]=Res ;
		i++;
		k=1;
		taskEXIT_CRITICAL_FROM_ISR(status_value);							//退出临界
	}

	USART_ClearITPendingBit(USART3,USART_IT_RXNE);					//清除标志位
}


	

