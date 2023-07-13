#include "ros_control.h"
#include "main.h"

ROS_CONTROL ros_control;

uint8_t verify_data[MAX_interaction_byte]={0};
char verify_state;

void ros_control_task(void *pvParameters)
{
	while(1)
	{
		verify_state = data_verify();
		
		if(verify_data[2]>>7 == 0x01) {
			ros_control.vx = -(verify_data[1]+(verify_data[2]&0x7f)/100.0f);
		} 		
		else 
			ros_control.vx = verify_data[1]+verify_data[2]/100.0f;
		
		if(verify_data[4]>>7 == 0x01) {
			ros_control.vy = -(verify_data[3]+(verify_data[4]&0x7f)/100.0f);
		} 		
		else 
			ros_control.vy = verify_data[3]+verify_data[4]/100.0f;
		
		if(verify_data[12]>>7 == 0x01) {
			ros_control.vz = -(verify_data[11]+(verify_data[12]&0x7f)/100.0f);
		}
		else 
			ros_control.vz = verify_data[11]+verify_data[12]/100.0f;
		
		ros_control.motor_rpm_vx = anti_conversion(ros_control.vx);
		ros_control.motor_rpm_vy = anti_conversion(ros_control.vy);
		ros_control.motor_rpm_vz = anti_conversion(ros_control.vz);
		
		vTaskDelay(1);
		
	}
	
}

char data_frame_head=0;
char data_verify(void)
{
	for(int i=0;i<MAX_interaction_byte;i++)
	{
		if(rx_buffer[i]==0xaa)
		{
			data_frame_head=i;
		}
	}
	for(int i=0, j = data_frame_head;i<MAX_interaction_byte;i++)
	{
		verify_data[i] = rx_buffer[j++];
		if(j==MAX_interaction_byte) j=0;
	}
	
	if(verify_data[0]!=0xaa) return 0;//数据出错
	else return 1;//数据正确
	
		
}














