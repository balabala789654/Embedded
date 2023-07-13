#include "main.h"
#include "IMUTask.h"

void Init() //初始化
{
	delay_init(168);														   //延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);							   //设置系统中断优先级分组4
																			   // CAN接口初始化//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
	CAN1_Mode_Init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal); // 1Mbps
	CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal);
	remote_control_init();
	TIM5_PWM_Init(20000 - 1, 84 - 1); // 84M/84/20000=50hz. 此处的200就是计数器的比较值 e.g.175/200*20ms=17.5ms    得脉冲宽度：20ms-17.5ms=2.5ms

	Device_Usart1_ENABLE_Init(115200); //初始化裁判系统通讯串口
	Device_Usart6_ENABLE_Init(921600); //初始化视觉通讯串口

	//初始化LED端口和激光
	LED_Init();
	LASER_Init();

	// MPU初始化
	while (MPU6500_Init());
	IMU_Calibration();
	accel_mat_init();
	ACCEL_Calibration();
	MPUHEAT_configuration();
}

int main(void)
{
	Init();
	start1();
	while (1)
		;
}
