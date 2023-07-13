#ifndef __MAIN_H
#define __MAIN_H

//SYSTEM
#include "sys.h"
#include "delay.h"
//STM
#include "stm32f4xx.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
//OS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "FreeRTOSConfig.h"
//arm_math
#include "pid.h"
#include "arm_math.h"
#include "user_lib.h"
//TASK
#include "start_task.h"
#include "IMUTask.h"
#include "remote_control.h"
#include "Judge_Task.h"
#include "detect_task.h"
#include "shoot_task.h"
#include "get_judge_measure.h"
//HARDWARE
#include "can.h"
#include "remote_control.h"
#include "rc.h"
#include "timer.h"
#include "led.h"
#endif
