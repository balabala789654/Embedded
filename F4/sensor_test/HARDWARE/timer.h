#ifndef _TIMER_H
#define _TIMER_H
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK STM32F407������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/6/16
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

#include "stdlib.h"
#include "sys.h"
void TIM3_Int_Init(u16 arr,u16 psc);

void TIM3_IRQHandler(void);
void TIM5_PWM_Init(u32 arr,u32 psc);
void TIM1_Int_Init(u16 arr,u16 psc);


#endif