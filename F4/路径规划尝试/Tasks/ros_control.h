#ifndef _ROS_CONTROL_H
#define _ROS_CONTROL_H

typedef struct
{
	float vx;
	float vy;
	float vz;
	
	float motor_rpm_vx;
	float motor_rpm_vy;
	float motor_rpm_vz;
	
	
}ROS_CONTROL;
void ros_control_task(void *pvParameters);
char data_verify(void);

extern ROS_CONTROL ros_control;




#endif







