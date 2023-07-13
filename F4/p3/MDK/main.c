#include "stm32f4xx.h"                  // Device header
#include "usart.h"
#include "gpio.h"
#include "timer.h"
#include "delay.h"
#include "p3.h"
#include "stdio.h"
#include "Low_pass.h"

uint8_t rec[13];
uint8_t rec2[13];
LOW_PASS remote;
extern uint8_t verify_data[13];

int main()
{
	gpio_Init();
	usart1_init(115200);
	usart6_init(115200);
	delay_init(168);
	
	while(1)
	{
		remote=Low_pass_output(p3_remote_output);
		printf("%f,%d\r\n", remote.ch[3], p3_remote_output(verify_data).ch[3]);
		delay_ms(1);
	}
}




