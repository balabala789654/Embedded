#include "stm32f4xx.h"                  // Device header
#include "spi.h"
#include "delay.h"
#include "gpio.h"
#include "timer.h"
#include "usart.h"
#include "stdio.h"
#include "as5047.h"
#include "can.h"
#include "location_sensor.h"

float distance_L_wheel;
float distance_R_wheel;

float distance_Y;
float distance_X;
int main()
{
	SPI2_Init();
	SPI3_Init();
	
	delay_init(168);
	gpio_Init();
	uart_init(115200);
	TIM3_Int_Init(5000-1,840-1);
	TIM1_Int_Init(1000-1,168-1);
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
		
	delay_ms(1000);
	
	int initial_angle_L = read_as5047_L();
	int initial_angle_R = read_as5047_R();
	while(1)
	{		
		distance_L_wheel=cul_location_L(4000, initial_angle_L);
		distance_R_wheel=cul_location_R(4000, initial_angle_R);
		
		distance_X=out_coordinate_x(&distance_L_wheel, &distance_R_wheel);
		distance_Y=out_coordinate_y(&distance_L_wheel, &distance_R_wheel);
		
	}
}


