#include "motortask.h"


int speed = 2000;
int spin_speed = 2000;
int spin_parameter = 1000;

void task1_task(void *pvParameters)
{
	LED0=!LED0;
	LED1=!LED1;
	while(1)
	{
		remote_contorl_speed(&rc_ctrl);
		motor_speed_compute();
		moter_send_3508(pid_control.M3508_PID[3].out,pid_control.M3508_PID[2].out,pid_control.M3508_PID[0].out,pid_control.M3508_PID[1].out);
		vTaskDelay(1);
	}
}

////////////电机期望速度计算////////////
void remote_contorl_speed(RC_ctrl_t* RC)
{
	int x,y=0;
	int vx,vy;

	y = RC->rc.ch[3]*speed/660;
	x = RC->rc.ch[2]*speed/660;
	vy = y*cos(PI/4)-x*cos(PI/4);
	vx = y*sin(PI/4)+x*sin(PI/4);
	
	if(RC->rc.s[0]==2&&RC->rc.s[1]==2)
	{
		M3508_control.M3508[0].pid_set_speed=0;
		M3508_control.M3508[1].pid_set_speed=0;
		M3508_control.M3508[2].pid_set_speed=0;
		M3508_control.M3508[3].pid_set_speed=0;
	}
	else
	{
		if(RC->rc.ch[2]==0&&RC->rc.ch[3]==0&&RC->rc.ch[0]==0&&RC->rc.ch[1]==0)
		{
			M3508_control.M3508[0].pid_set_speed=0;
			M3508_control.M3508[1].pid_set_speed=0;
			M3508_control.M3508[2].pid_set_speed=0;
			M3508_control.M3508[3].pid_set_speed=0;
		}

		else if(RC->rc.ch[2]!=0||RC->rc.ch[3]!=0)
		{			
			M3508_control.M3508[0].pid_set_speed=vy+RC->rc.ch[0]*spin_speed/660;
			M3508_control.M3508[2].pid_set_speed=-vy+RC->rc.ch[0]*spin_speed/660;
			
			M3508_control.M3508[1].pid_set_speed=-vx+RC->rc.ch[0]*spin_speed/660;
			M3508_control.M3508[3].pid_set_speed=vx+RC->rc.ch[0]*spin_speed/660;
		}
		else if(RC->rc.ch[0]!=0||RC->rc.ch[1]!=0)
		{
			M3508_control.M3508[0].pid_set_speed=RC->rc.ch[0]*spin_speed/660;
			M3508_control.M3508[1].pid_set_speed=RC->rc.ch[0]*spin_speed/660;
			M3508_control.M3508[2].pid_set_speed=RC->rc.ch[0]*spin_speed/660;
			M3508_control.M3508[3].pid_set_speed=RC->rc.ch[0]*spin_speed/660;
		}

	}

}

////////////pid 计算//////////
void motor_speed_compute(void)
{
	PID_calc(&pid_control.M3508_PID[0],M3508_control.M3508[0].speed_rpm,M3508_control.M3508[0].pid_set_speed);
	PID_calc(&pid_control.M3508_PID[1],M3508_control.M3508[1].speed_rpm,M3508_control.M3508[1].pid_set_speed);
	PID_calc(&pid_control.M3508_PID[2],M3508_control.M3508[2].speed_rpm,M3508_control.M3508[2].pid_set_speed);
	PID_calc(&pid_control.M3508_PID[3],M3508_control.M3508[3].speed_rpm,M3508_control.M3508[3].pid_set_speed);
}
