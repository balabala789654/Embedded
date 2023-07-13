#include "main.h"
#include "IMUTask.h"

void Init() //��ʼ��
{
	delay_init(168);														   //��ʱ������ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);							   //����ϵͳ�ж����ȼ�����4
																			   // CAN�ӿڳ�ʼ��//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
	CAN1_Mode_Init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal); // 1Mbps
	CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal);
	remote_control_init();
	TIM5_PWM_Init(20000 - 1, 84 - 1); // 84M/84/20000=50hz. �˴���200���Ǽ������ıȽ�ֵ e.g.175/200*20ms=17.5ms    �������ȣ�20ms-17.5ms=2.5ms

	Device_Usart1_ENABLE_Init(115200); //��ʼ������ϵͳͨѶ����
	Device_Usart6_ENABLE_Init(921600); //��ʼ���Ӿ�ͨѶ����

	//��ʼ��LED�˿ںͼ���
	LED_Init();
	LASER_Init();

	// MPU��ʼ��
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
