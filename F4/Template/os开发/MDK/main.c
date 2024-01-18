#include "stm32f4xx.h"                  // Device header
#include "CubeMars_AK80_8.h"
#include "delay.h"


int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	//设置系统中断优先级分组4
	delay_init(168);
	
	AK80_8_init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	
	while(1){
		AK80_8_control();
		delay_ms(1);
	}
}

