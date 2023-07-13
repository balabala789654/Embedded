#ifndef __START_TASK_H
#define __START_TASK_H
#include "sys.h"
extern void start1(void);
extern void start_task(void *pvParameters);



#define JUDGE_NVIC 9// ∑¿÷πFreeRTOS±®¥Ì£∫Error:..\FreeRTOS\port\RVDS\ARM_CM4F\port.c,768
#define PC_NVIC 8	 // ∑¿÷πFreeRTOS±®¥Ì£∫Error:..\FreeRTOS\port\RVDS\ARM_CM4F\port.c,768
#define RC_NVIC 7
#define CAN1_NVIC 4
#define CAN2_NVIC 4

#define PI					3.14159265358979f

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;
/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#endif
