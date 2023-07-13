#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "oled.h"

int main()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	delay_init();	    	 //��ʱ������ʼ��	  
	OLED_Init();			//��ʼ��OLED  
	OLED_Clear(); 
	OLED_ShowCHinese(18,0,3);//��
	OLED_ShowCHinese(36,0,4);//��
	OLED_ShowCHinese(72,0,5);//��
	OLED_ShowCHinese(90,0,6);//��

	while(1)
	{
	}
}

