#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include <stdio.h>
#include "canrecive.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define D11_ON 0x0012
#define D11_OFF 0x0011
#define motor_ON 0x0013
#define motor_OFF 0x0014

int8_t send_data[8]={0xff,0,0,0,0,0,0,0xfe};
extern uint16_t rx_buffer[256];
float pitch=0;
float yaw=0;
float roll=0;
int main(void)
{ 
 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);		//延时初始化 
	uart_init(115200);	//串口初始化波特率为115200
	LED_Init();		  		//初始化与LED连接的硬件接口  
	MPU_Init();
	while(mpu_dmp_init());
	printf("working\r\n");
	static int i = 0;
	while(1)
	{
		mpu_dmp_get_data(&pitch, &roll, &yaw);
		send_data[1] = pitch;
		send_data[2] = yaw;
		send_data[3] = roll;
		
		send_data[4] = (pitch-send_data[1])*100;
		send_data[5] = (pitch-send_data[2])*100;
		send_data[6] = (pitch-send_data[3])*100;
		for(int i=0;i<8;i++)
		{
			delay_ms(1);
			USART_SendData(USART1,send_data[i]);
		}
//		USART_SendData(USART1, *send_data);
		if(rx_buffer[0]==D11_ON)
		{
			if(i==0)
			{
				GPIO_ResetBits(GPIOD, GPIO_Pin_2);
				i=1;
			}
			else
			{
				GPIO_SetBits(GPIOD, GPIO_Pin_2);
				i=0;
				
			}
			
		}
		else if(rx_buffer[0]==D11_OFF)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_2);
		}
		if(rx_buffer[1]==motor_ON || rx_buffer[1]==motor_OFF)
		{
			if(rx_buffer[1]==motor_ON) moter_send_3508(0,0,0,0);
			else moter_send_3508(0,0,0,0);
		}
		delay_ms(100);
	}
}

