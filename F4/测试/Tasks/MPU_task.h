#ifndef _MPU_TASK
#define _MPU_TASK
#include "stm32f4xx.h"                  // Device header
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "MPU_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"

extern void MPU_task(void *pvParameters);

extern  float pitch;
extern  float yaw;
extern  float roll;

extern short accx,accy,accz; 	//各个轴的加速度原始值
extern float ax,ay,az;		//各个轴的计算后的加速度值

#endif

