#include "rpm_feedback.h"

int cur_L=0;
int cur_R=0;
char flag_L=1;
char flag_R=1;
int _time_L=0;
int _time_R=0;

static float rpm_L=0;
static float rpm_R=0;


int read_initial_angle(void)
{
	int angle;
	angle = read_as5047_L();
}


float rpm_cul_L(int check_angle, int target_angle)
{
	int angle = read_as5047_L();
	
	if(((angle >= (target_angle-1000)) && (angle <= (target_angle+1000)))&&flag_L) 
	{
		cur_L++;
		flag_L=0;
			
	}
	if(((angle >= (check_angle-1000)) && (angle <= (check_angle+1000)))&&(flag_L!=1))
	{
		flag_L=1;
	}		
		
	if(cur_L==1)
	{
		rpm_L=(float)(cur_L/(0.001f*_time_L));
		_time_L=0;
		cur_L=0;
	}
	
	printf("%d\r\n", angle);
	return rpm_L;
	
}


float rpm_cul_R(int check_angle, int target_angle)
{
	int angle1 = read_as5047_R();
	
	if(((angle1 >= (target_angle-1000)) && (angle1 <= (target_angle+1000)))&&flag_R) 
	{
		cur_R++;
		flag_R=0;
			
	}
	if(((angle1 >= (check_angle-1000)) && (angle1 <= (check_angle+1000)))&&(flag_R!=1))
	{
		flag_R=1;
	}		
		
	if(cur_R==1)
	{
		rpm_R=(float)(cur_R/(0.001f*_time_R));
		_time_R=0;
		cur_R=0;
	}
	//printf("%d\r\n", angle1);
	return rpm_R;
	
}


int rpm_send_to_master(int std, int n1, int n2)
{
	
	CanTxMsg rpm_msg;
	rpm_msg.StdId=std;
	rpm_msg.DLC=4;
	rpm_msg.IDE=CAN_Id_Standard;
	rpm_msg.RTR=CAN_RTR_Data;
	
	rpm_msg.Data[0]=n1>>8;
	rpm_msg.Data[1]=n1 & 0x00ff;
	rpm_msg.Data[2]=n2>>8;
	rpm_msg.Data[3]=n2 & 0x00ff;
	rpm_msg.Data[4]=0xaa;
	rpm_msg.Data[5]=0xaa;
	rpm_msg.Data[6]=0xaa;
	rpm_msg.Data[7]=0xaa;  
	
	return CAN_Transmit(CAN1,&rpm_msg);
}

