#ifndef __MOTORTASK_H
#define __MOTORTASK_H

#include "canrecive.h"
#include "can.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "FreeRTOSConfig.h"
#include "math.h"
#include "angle_compute.h"

#define translate 1
#define spin 2
#define spin_translate 3

#define wheel_radius 0.075
extern M3508 M3508_control;
extern PID pid_control;

extern void task1_task(void *pvParameters);
void remote_contorl_speed(RC_ctrl_t* RC);
void motor_speed_compute(void);

#endif


