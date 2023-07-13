#include "stm32f4xx.h"                  // Device header
#include "spi.h"
#include "delay.h"
#include "gpio.h"
#include "timer.h"
#include "usart.h"
#include "stdio.h"
#include "kalman.h"

void read_as5047(void);

uint16_t data2;
uint16_t send_data=0x3fff;

int16_t angle;
int16_t angle_kal;
int16_t last_angle;
int16_t check_angle;

float rpm;

extKalman_t _kalman;
extKalman_t rpm_kalman;
extKalman_t cur_kalman;

uint16_t target_angle=0;
char flag=1;
int _time=0;

int count1=0;

int cur=0;
int cur_kal;

int main()
{
	SPI2_Init();
	delay_init(168);
	gpio_Init();
	//TIM3_Int_Init(5000-1, 8400-1);
	uart_init(115200);
	TIM1_Int_Init(50000-1,1680-1);
	
	KalmanCreate(&_kalman, 1, 300);
	KalmanCreate(&rpm_kalman, 1, 300);
	KalmanCreate(&cur_kalman, 10, 300);
	
	target_angle=8000;
	check_angle=4000;
	
	while(1)
	{
		//delay_us(1);
		read_as5047();
		//angle_kal = KalmanFilter(&_kalman, angle);

		if(((angle >= (target_angle-1000)) && (angle <= (target_angle+1000)))&&flag) 
		{
			cur++;
			flag=0;
			//printf("%d\r\n", cur);
			
		}
		if(((angle >= (check_angle-1000)) && (angle <= (check_angle+1000)))&&(flag!=1))
		{
			flag=1;
		}
//		count1++;
		
//		last_angle=angle_kal;
		
		//printf("%d\r\n", angle);
	}
}

void read_as5047(void)
{
	send_data=0x3fff;
	send_data |= 0x4000;
	send_data |= (parity_even(send_data)<<15);
	data2 = SPI2_ReadWriteByte(send_data);
	angle = data2 & 0x3fff;				
}


