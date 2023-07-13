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

#endif

