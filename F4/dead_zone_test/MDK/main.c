#include "stm32f4xx.h"                  // Device header
#include "Remote_Control.h"
#include "dead_zone.h"
#include "timer.h"
#include "gpio.h"
#include "kalman.h"
#include "usart.h"
#include "stdio.h"


REMOTE remote;
extKalman_t kalman;
float sensor1;
float sensor2;

int main()
{
	usart1_init(115200);
	gpio_Init();
	remote_control_init();
	TIM3_Int_Init(5000-1, 840-1);
	KalmanCreate(&kalman, 10, 500);
	while(1)
	{
		sensor1 = dead_zone_output(&rc_ctrl).ch[3];
		sensor2 = KalmanFilter(&kalman,sensor1);
		
		printf("%f,%f\r\n", sensor1, sensor2);
		
	}
}
