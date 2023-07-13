#ifndef __TASK1_H
#define __TASK1_H	 

#include "task.h"
extern TaskHandle_t targettask_Handler;


void motor_task(void *pvParameters);
void PID_task(void *pvParameters);
void target_task(void *pvParameters);
void KEY_task(void *pvParameters);
void usart_task(void *pvParameters);

#endif

