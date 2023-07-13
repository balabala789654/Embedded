#include "main.h"

int speed = 2000;
int spin_speed = 1000;
int spin_parameter = 1000;


static float x,y,z=0;
static	float vx,vy,vz;

void task1_task(void *pvParameters)
{
	LED0=!LED0;
	LED1=!LED1;
	while(1)
	{
		remote_contorl_speed(&rc_ctrl);
		motor_speed_compute();
		moter_send_3508(pid_control.M3508_PID[0].out,pid_control.M3508_PID[1].out,pid_control.M3508_PID[2].out,pid_control.M3508_PID[3].out);
		
		vTaskDelay(1);
	}
}
	
////////////电机期望速度计算////////////
void remote_contorl_speed(RC_ctrl_t* RC)
{

	
	
	if(RC->rc.s[0]==2&&RC->rc.s[1]==2)
	{
		M3508_control.M3508[0].pid_set_speed=0;
		M3508_control.M3508[1].pid_set_speed=0;
		M3508_control.M3508[2].pid_set_speed=0;
		M3508_control.M3508[3].pid_set_speed=0;
	}
	else
	{
		if(RC->rc.s[1]==3)
		{
			
			y = RC->rc.ch[3]*speed/660;
			x = RC->rc.ch[2]*speed/660;
			
			vy = y*cos(PI/4)-x*cos(PI/4);
			vx = y*sin(PI/4)+x*sin(PI/4);

			if(RC->rc.ch[2]==0&&RC->rc.ch[3]==0&&RC->rc.ch[0]==0&&RC->rc.ch[1]==0)
			{
				M3508_control.M3508[0].pid_set_speed=0;
				M3508_control.M3508[1].pid_set_speed=0;
				M3508_control.M3508[2].pid_set_speed=0;
				M3508_control.M3508[3].pid_set_speed=0;
			}

			else if(RC->rc.ch[2]!=0||RC->rc.ch[3]!=0||RC->rc.ch[1]!=0||RC->rc.ch[0]!=0)
			{			
					M3508_control.M3508[0].pid_set_speed=-vy+RC->rc.ch[0]*spin_speed/660;
					M3508_control.M3508[2].pid_set_speed=vy+RC->rc.ch[0]*spin_speed/660;
					
					M3508_control.M3508[1].pid_set_speed=vx+RC->rc.ch[0]*spin_speed/660;
					M3508_control.M3508[3].pid_set_speed=-vx+RC->rc.ch[0]*spin_speed/660;
				
//				M3508_control.M3508[0].pid_set_speed=-vy;
//				M3508_control.M3508[2].pid_set_speed=vy;
//				
//				M3508_control.M3508[1].pid_set_speed=vx;
//				M3508_control.M3508[3].pid_set_speed=-vx;

			}

		}
		else if(RC->rc.s[1]==1)
		{
						
			y = (ros_control.x - ros_control.current_x);
			x = -(ros_control.y - ros_control.current_y);
			
			x = (x*10000/4);
			y = (y*10000/4);
			
//			if(x>0) x = (x*10000/4); else x = (x*10000/4);
//			if(y>0) y = (y*10000/4); else y = (y*10000/4);
			
			if(x>2000) x=2000; else if(x<-2000) x=-2000;
			if(y>2000) y=2000; else if(y<-2000) y=-2000;
			
			vy = y*cos(PI/4)-x*cos(PI/4);
			vx = y*sin(PI/4)+x*sin(PI/4);
			
			M3508_control.M3508[0].pid_set_speed=-vy;
			M3508_control.M3508[2].pid_set_speed=vy;
			
			M3508_control.M3508[1].pid_set_speed=vx;
			M3508_control.M3508[3].pid_set_speed=-vx;
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
