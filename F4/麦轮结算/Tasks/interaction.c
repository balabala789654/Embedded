#include "interaction.h"


/*
rviz 坐标系中红色为x轴正方向， 绿色为y轴正方向
			x
			|
			|
			|
			|
  y-------------------
			|
			|
			|
			|

*/

int8_t send_data[MAX_interaction_byte]={0xAA,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xBB};//帧头0XAA 帧尾0XBB 20字节

uint16_t yaw_data;

float speed_yaw = 0;
float conversion(int RPM)
{
	float n = 0;
	n = ((float)RPM*PI*wheel_radius*2.0f)/1200.0f;
	return n;
	
}
float _vx, _vy;
void interaction_task(void *pvParameters)
{
	
	
	while(1)
	{	
		float a,b,c;//别问这是什么 问就是重要的东西
		float average_vy = ((conversion(-M3508_control.M3508[0].speed_rpm)+conversion(M3508_control.M3508[2].speed_rpm))/2.0f);
		float average_vx = ((conversion(M3508_control.M3508[1].speed_rpm)+conversion(-M3508_control.M3508[3].speed_rpm))/2.0f);
		
		_vy = average_vy*cos(PI/4)-average_vx*cos(PI/4);
		_vx = average_vy*sin(PI/4)+average_vx*sin(PI/4);
		
		if(yaw<0) {yaw_data=-yaw;yaw_data|=0x8000;}
		else yaw_data=yaw;
		
		send_data[__pitch] = pitch;
		
		send_data[__yaw] = yaw_data>>8; //16bit数据的高八位，其中最高位为符号位,1为负数,0为正数
		send_data[__yaw+1] = yaw_data&0x00ff;	//16bit数据的低八位
		
		send_data[__roll] = roll;
		
		a = _vx;
		b = _vy;
		c = speed_yaw;
		
		if(a < 0.0f) {send_data[__vx_sign] = 0; a = -a;} else send_data[__vx_sign] = 1;
		if(b < 0.0f) {send_data[__vy_sign] = 0; b = -b;} else send_data[__vy_sign] = 1;
		if(c < 0.0f) {send_data[__vw_sign] = 0; c = -c;} else send_data[__vw_sign] = 1;
		
		send_data[__vx] = a;
		send_data[__vx+1] = a * 100;
		send_data[__vy] = b;
		send_data[__vy+1] = b * 100;
		send_data[__vw] = c;
		send_data[__vw+1] = c * 100;
		
		
		for(int i=0;i<MAX_interaction_byte;i++)
		{
			delay_ms(1);
			USART_SendData(USART1,send_data[i]);
		}
		
		vTaskDelay(1);
	}
}


