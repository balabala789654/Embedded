#include "time.h"
#include "sys.h"
#include "led.h"
#include "FreeRTOS.h"
#include "usart.h"	  


void TIM2_Int_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_Stru;
	NVIC_InitTypeDef NVIC_Stru;
	TIM_ICInitTypeDef TIM2_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	
	TIM_Stru.TIM_Prescaler=psc;
	TIM_Stru.TIM_Period=arr;
	TIM_Stru.TIM_ClockDivision=TIM_CKD_DIV1;	
	TIM_Stru.TIM_CounterMode=TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM2,&TIM_Stru);
	
	TIM2_ICInitStructure.TIM_Channel=TIM_Channel_1;
	TIM2_ICInitStructure.TIM_ICFilter=0x00;
	TIM2_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;
	TIM2_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM2_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);

	NVIC_Stru.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_Stru.NVIC_IRQChannelPreemptionPriority=0x00;
	NVIC_Stru.NVIC_IRQChannelSubPriority=0x03;
	NVIC_Stru.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_Stru);
	
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1,ENABLE);
	
	TIM_Cmd(TIM2,ENABLE);
}



void TIM3_PWM_Init(u16 a,u16 b)
{
	GPIO_InitTypeDef GPIO_Stru;
	TIM_TimeBaseInitTypeDef TIM3_Stru;
	TIM_OCInitTypeDef TIM_PWMStru;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);
	
	
	GPIO_Stru.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Stru.GPIO_Pin=GPIO_Pin_5; 
	GPIO_Stru.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_Stru);
	
	TIM3_Stru.TIM_Period=a;
	TIM3_Stru.TIM_Prescaler=b;
	TIM3_Stru.TIM_ClockDivision=0;
	TIM3_Stru.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3,&TIM3_Stru);
	
	
	TIM_PWMStru.TIM_OCMode=TIM_OCMode_PWM2;
	TIM_PWMStru.TIM_OutputState=TIM_OutputState_Enable;
	TIM_PWMStru.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OC2Init(TIM3,&TIM_PWMStru);
	
	
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_Cmd(TIM3,ENABLE);
}

//u8  TIM2CH1_CAPTURE_STA=0;	    				
//u16	TIM2CH1_CAPTURE_VAL;

//void TIM2_IRQHandler(void)
//{

// 	if((TIM2CH1_CAPTURE_STA&0X80)==0)
//	{	  
//		if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
//		 
//		{	   
//			if(TIM2CH1_CAPTURE_STA&0X40)
//			{
//				if((TIM2CH1_CAPTURE_STA&0X3F)==0X3F)
//				{
//					TIM2CH1_CAPTURE_STA|=0X80;
//					TIM2CH1_CAPTURE_VAL=0XFFFF;
//				}else TIM2CH1_CAPTURE_STA++;
//			}	 
//		}
//	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
//		{	
//			if(TIM2CH1_CAPTURE_STA&0X40)
//			{	  			
//				TIM2CH1_CAPTURE_STA|=0X80;
//				TIM2CH1_CAPTURE_VAL=TIM_GetCapture1(TIM2);
//		   		TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Rising); 
//			}else  				
//			{
//				TIM2CH1_CAPTURE_STA=0;	
//				TIM2CH1_CAPTURE_VAL=0;
//	 			TIM_SetCounter(TIM2,0);
//				TIM2CH1_CAPTURE_STA|=0X40;
//		   	TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Falling);
//			}		    
//		}			     	    					   
// 	}
// 
//    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1|TIM_IT_Update);
// 
//	
//}



