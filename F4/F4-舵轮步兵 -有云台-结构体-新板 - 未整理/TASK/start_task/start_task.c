#include "start_task.h"
#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "gimbal_task.h"
#include "ui_task.h"
#define START_TASK_PRIO 1
#define START_STK_SIZE 128
TaskHandle_t StartTask_Handler;
void start_task(void *pvParameters);

#define CHASSIS_TASK_PRIO 18
#define CHASSIS_STK_SIZE 256
TaskHandle_t ChassisTask_Handler;
void chassis_task(void *pvParameters);

#define GIMBAL_TASK_PRIO 18
#define GIMBAL_STK_SIZE 256
TaskHandle_t GimbalTask_Handler;
void gimbal_task(void *pvParameters);

#define IMU_TASK_PRIO 32
#define IMU_TASK_SIZE 512
static TaskHandle_t MPUTask_Handler;

#define JUDGE_TASK_PRIO 19
#define JUDGE_TASK_SIZE 256
static TaskHandle_t JUDGE_TASK_Handler;

#define DETECT_TASK_PRIO 10
#define DETECT_TASK_SIZE 128
static TaskHandle_t DETECT_TASK_Handler;

#define SHOOT_TASK_PRIO 17
#define SHOOT_TASK_SIZE 256
static TaskHandle_t SHOOTTask_Handler;

#define TX2_TASK_PRIO 20
#define TX2_TASK_SIZE 256
static TaskHandle_t TX2_TASK_Handler;

#define UI_TASK_PRIO 12
#define UI_TASK_SIZE 512
TaskHandle_t UI_TASK_Handler;

#define INTER_TASK_PRIO 16
#define INTER_TASK_SIZE 512
static TaskHandle_t INTER_TASK_Handler;

void start_task(void *pvParameters)
{
  xTaskCreate((TaskFunction_t)IMU_task,
              (const char *)"IMU_task",
              (uint16_t)IMU_TASK_SIZE,
              (void *)NULL,
              (UBaseType_t)IMU_TASK_PRIO,
              (TaskHandle_t *)&MPUTask_Handler);

  xTaskCreate((TaskFunction_t)gimbal_task,
              (const char *)"gimbal_task",
              (uint16_t)GIMBAL_STK_SIZE,
              (void *)NULL,
              (UBaseType_t)GIMBAL_TASK_PRIO,
              (TaskHandle_t *)&GimbalTask_Handler);

  xTaskCreate((TaskFunction_t)chassis_task,
              (const char *)"chassis_task",
              (uint16_t)CHASSIS_STK_SIZE,
              (void *)NULL,
              (UBaseType_t)CHASSIS_TASK_PRIO,
              (TaskHandle_t *)&ChassisTask_Handler);

  xTaskCreate((TaskFunction_t)Judge_task,
              (const char *)"Judge_task",
              (uint16_t)JUDGE_TASK_SIZE,
              (void *)NULL,
              (UBaseType_t)JUDGE_TASK_PRIO,
              (TaskHandle_t *)&JUDGE_TASK_Handler);

  xTaskCreate((TaskFunction_t)detect_task,
              (const char *)"detect_task",
              (uint16_t)DETECT_TASK_SIZE,
              (void *)NULL,
              (UBaseType_t)DETECT_TASK_PRIO,
              (TaskHandle_t *)&DETECT_TASK_Handler);

  xTaskCreate((TaskFunction_t)shoot_task,
              (const char *)"shoot_task",
              (uint16_t)SHOOT_TASK_SIZE,
              (void *)NULL,
              (UBaseType_t)SHOOT_TASK_PRIO,
              (TaskHandle_t *)&SHOOTTask_Handler);

  // MINIPC发送任务
 xTaskCreate((TaskFunction_t)Interactive_task,
             (const char *)"Interactive_task",
             (uint16_t)INTER_TASK_SIZE,
             (void *)NULL,
             (UBaseType_t)INTER_TASK_PRIO,
             (TaskHandle_t *)&INTER_TASK_Handler);

 // TX2任务
 xTaskCreate((TaskFunction_t)Tx2_task,
             (const char *)"Tx2_task",
             (uint16_t)TX2_TASK_SIZE,
             (void *)NULL,
             (UBaseType_t)TX2_TASK_PRIO,
             (TaskHandle_t *)&TX2_TASK_Handler);

 xTaskCreate((TaskFunction_t)ui_task,
             (const char *)"ui_task",
             (uint16_t)UI_TASK_SIZE,
             (void *)NULL,
             (UBaseType_t)UI_TASK_PRIO,
             (TaskHandle_t *)&UI_TASK_Handler);

  vTaskDelete(StartTask_Handler);
}

void start1(void)
{
  xTaskCreate((TaskFunction_t)start_task,
              (const char *)"start_task",
              (uint16_t)START_STK_SIZE,
              (void *)NULL,
              (UBaseType_t)START_TASK_PRIO,
              (TaskHandle_t *)&StartTask_Handler);
  vTaskStartScheduler();
}
