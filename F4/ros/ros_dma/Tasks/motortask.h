#ifndef __MOTORTASK_H
#define __MOTORTASK_H

#include "canrecive.h"
#include "can.h"
#include "pid.h"
#include "math.h"
#include "angle_compute.h"

#define translate 1
#define spin 2
#define spin_translate 3



typedef struct
{
	PidType pid;
//	motor_measure_t feed_back;
	float speed_set;
	
}motor;

typedef struct
{
	motor M3508[4];
	
}CHASSIS;

//extern CHASSIS chassis_contorl;
//extern void task1_task(void *pvParameters);
//void remote_contorl_speed(RC_ctrl_t* RC);
//void motor_speed_compute(void);
//float spin_speed(RC_ctrl_t* RC);


#endif


