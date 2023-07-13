#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx.h"                  // Device header
#include "can.h"
#include "pid.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rc.h"
#include "led.h"
#include "motortask.h"
#include "FreeRTOSConfig.h"
#include "Remote_Control.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "MPU_task.h"
#include "usart.h"	
#include "dma.h"
#include "interaction.h"
#include "timer.h"
#include "ros_control.h"
#include "stdio.h"

#define SEND_BUF_SIZE 20

#endif

