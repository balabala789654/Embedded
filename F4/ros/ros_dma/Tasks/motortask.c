//#include "motortask.h"

//CHASSIS chassis_contorl;
//int speed=0;
//int spin_parameter = 1000;

////void task1_task(void *pvParameters)
////{
////	LED0=!LED0;
////	LED1=!LED1;
////	while(1)
////	{
////		remote_contorl_speed(&rc_ctrl);
////		motor_speed_compute();
////		moter_send_3508(pid_control.M3508_PID[0].out,pid_control.M3508_PID[1].out,pid_control.M3508_PID[2].out,pid_control.M3508_PID[3].out);
////		
////		vTaskDelay(1);
////	}
////}
//	
//////////////电机期望速度计算////////////
//void remote_contorl_speed(RC_ctrl_t* RC)
//{
//	
//	if(RC->rc.s[1]==1)
//	{
//		speed=1000;
//	}
//	else if(RC->rc.s[1]==2)
//	{
//		speed=3000;
//	}
//	else if(RC->rc.s[1]==3)
//	{
//		speed=5000;
//	}

//	
//	if(RC->rc.ch[2]==0&&RC->rc.ch[3]==0&&RC->rc.ch[0]==0&&RC->rc.ch[1]==0)
//	{
//		chassis_contorl.M3508[0].speed_set=0;
//		chassis_contorl.M3508[1].speed_set=0;
//		chassis_contorl.M3508[2].speed_set=0;
//		chassis_contorl.M3508[3].speed_set=0;
//	}

//	else if(RC->rc.ch[2]!=0||RC->rc.ch[3]!=0)
//	{			
//		chassis_contorl.M3508[0].speed_set = sin(angle_com(&rc_ctrl))*speed+spin_speed(&rc_ctrl);
//		chassis_contorl.M3508[1].speed_set = sin(angle_com(&rc_ctrl)-(2*PI/3))*speed+spin_speed(&rc_ctrl);
//		chassis_contorl.M3508[2].speed_set = sin(angle_com(&rc_ctrl)+(2*PI/3))*speed+spin_speed(&rc_ctrl);
//		chassis_contorl.M3508[3].speed_set = sin(angle_com(&rc_ctrl)+(2*PI/3))*speed+spin_speed(&rc_ctrl);
//	}
//	else if(RC->rc.ch[0]!=0||RC->rc.ch[1]!=0)
//	{
//		chassis_contorl.M3508[0].speed_set = spin_speed(&rc_ctrl);
//		chassis_contorl.M3508[1].speed_set = spin_speed(&rc_ctrl);
//		chassis_contorl.M3508[2].speed_set = spin_speed(&rc_ctrl);
//		chassis_contorl.M3508[3].speed_set = spin_speed(&rc_ctrl);
//	}

//}

//////////////pid 计算//////////
//void motor_speed_compute(void)
//{
//	PID_calc(&chassis_contorl.M3508[0].pid, chassis_contorl.M3508[0].feed_back.speed_rpm,chassis_contorl.M3508[0].speed_set);
//	PID_calc(&chassis_contorl.M3508[1].pid, chassis_contorl.M3508[1].feed_back.speed_rpm,chassis_contorl.M3508[1].speed_set);
//	PID_calc(&chassis_contorl.M3508[2].pid, chassis_contorl.M3508[2].feed_back.speed_rpm,chassis_contorl.M3508[2].speed_set);
//}

////////小陀螺//////
//float spin_speed(RC_ctrl_t* RC)
//{
//	float spinning = 0;
//	
//	spinning = RC->rc.ch[0]*spin_parameter/660;
//	return spinning;
//}

