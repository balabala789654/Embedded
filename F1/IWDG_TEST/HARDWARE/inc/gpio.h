#ifndef __GPIO_H
#define __GPIO_H

#include "sys.h"

//LED�˿ڶ���
#define LED0 PBout(5)// PB5
#define LED1 PEout(5)// PE5

void LED_Init(void);//��ʼ��

void KEY_Init(void); //IO��ʼ��
u8 KEY_Scan(u8 mode);

#define KEY0  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)//��ȡ����0
#define KEY1  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)//��ȡ����1
#define WK_UP   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//��ȡ����3(WK_UP) 

#define KEY0_PRES 	1	//KEY0����
#define KEY1_PRES	  2	//KEY1����
#define WKUP_PRES   3	//KEY_UP����(��WK_UP/KEY_UP)
#endif