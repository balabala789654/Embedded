#include "can.h"
#include "delay.h"
#include "pid.h"
#include "stm32f10x.h"                  // Device header

extern motor_measure_t M3508[4];

PidType PID[2];

float pid_set_speed = 1000;
float pid_set_angle = 10; //   round/s6
int main(void)
{
	delay_init();
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,2,CAN_Mode_Normal);
	PID_init(&PID[0],PID_POSITION,0.2,0.0f,0.0f,16000,800);
	PID_init(&PID[1],PID_POSITION,15.0f,0.2f,13.0f,5000,300);
	pid_set_angle *= 8191;
	
	while(1)
	{
		PID_calc(&PID[0],M3508[1].all_ecd, pid_set_angle);
		PID_calc(&PID[1],M3508[1].speed_rpm, PID[0].out);   
		moter_send_3508(0,PID[1].out,PID[1].out,0);
		delay_ms(5);
	}
}





