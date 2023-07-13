#include "stm32f4xx.h"                  // Device header
#include "arm_math.h"
#include "timer.h"
#include "gpio.h"
#include "delay.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "stdio.h"
#include "usart.h"
#include "kalman.h"
#include "KalmanFilter.h"
#include "stdlib.h"

float pitch, yaw, roll;

extern KALMAN_FILTER_MAT kalman_filter_mat;
extern KALMAN_FILTER_DATA kalman_filter_data;
int main(void)
{
	uart_init(115200);
	TIM3_Int_Init(1000-1, 8400-1);
	gpio_Init();
	delay_init(168);
	MPU_Init();
	//while(mpu_dmp_init()) LED2=0; 
	//LED2=1;
	
	mat_data_init(&kalman_filter_data);
	kalman_mat_init(&kalman_filter_mat, &kalman_filter_data);
	
	//kalman_filter_data.Z_data[0]=1.45;
	
	
	extKalman_t p;
	KalmanCreate(&p,1,400);
	
	int i;
	float SersorData;
	while(1)
	{
		
//		mpu_dmp_get_data(&pitch, &roll, &yaw);
//		SersorData = KalmanFilter(&p,yaw);
//		printf("%f,%f\r\n", yaw, SersorData);
		
		kalman_filter_loop();
		i+=0.01;
		//kalman_filter_data.Z_data[0]=0.5;
		kalman_filter_data.Z_data[0]=cos(i);
		
		
		int asd = rand()%200;
		float _rand = (float)(asd-100.0f)/100.0f;
		kalman_filter_data.Z_data[0]+=_rand*0;
		
		srand(i++);
		
		printf("%f,%f\r\n", kalman_filter_data.Z_data[0],kalman_filter_data.X_data[0]);
		delay_ms(10);
	}
}

