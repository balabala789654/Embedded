#include "stm32f4xx.h"                  // Device header
#include "spi.h"
#include "delay.h"
#include "gpio.h"
#include "timer.h"
#include "usart.h"
#include "stdio.h"
#include "as5047.h"
#include "rpm_feedback.h"
#include "can.h"

float K=0.0504;

float rpm_L;
float rpm_R;

int state;
int main()
{
	SPI2_Init();
	SPI3_Init();
	
	delay_init(168);
	gpio_Init();
	uart_init(115200);
	TIM1_Int_Init(1000-1,168-1);
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	
	int L_target_angle=8000;
	int L_check_angle=4000;
		
	int R_target_angle=8000;
	int R_check_angle=4000;
	
	int data_L, data_R;
	
	while(1)
	{
		rpm_L = rpm_cul_L(L_target_angle, L_check_angle);
		rpm_R = rpm_cul_R(R_target_angle, R_check_angle);
		
		data_L = (int)(((rpm_L-4.3f)/0.0504f)+1200);
		data_R = (int)(((rpm_R-4.3f)/0.0504f)+1200);
		
		state=rpm_send_to_master(0x220, data_L, data_R);
		
		//printf("%f\r\n", rpm);
	}
}


