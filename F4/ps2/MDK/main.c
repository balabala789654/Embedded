#include "stm32f4xx.h"                  // Device header
#include "spi.h"
#include "gpio.h"
#include "timer.h"
#include "usart.h"
#include "delay.h"
uint8_t rec1;
uint8_t rec2;
uint8_t rec3;
uint8_t rec4;
uint8_t rec5;
uint8_t rec6;
uint8_t rec7;
uint8_t rec8;


int main()
{
	gpio_Init();
	delay_init(168);
	TIM3_Int_Init(2000-1, 840-1);
	SPI2_Init();
	usart1_init(115200);
	while(1)
	{
		rec1 = SPI2_ReadWriteByte(0x01);
		rec2 = SPI2_ReadWriteByte(0X42);
//		rec3 = SPI2_ReadWriteByte(0xff);
//		rec4 = SPI2_ReadWriteByte(0x00);
//		rec5 = SPI2_ReadWriteByte(0x00);
//		rec6 = SPI2_ReadWriteByte(0x00);
//		rec7 = SPI2_ReadWriteByte(0x00);
		delay_ms(1);
		
	}
}

