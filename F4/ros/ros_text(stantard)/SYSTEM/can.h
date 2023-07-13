#ifndef __CAN_H
#define __CAN_H

#include "pid.h"
#include "stm32f4xx.h"                  // Device header
#include "sys.h"
#include "stdint.h"

uint8_t CAN1_Mode_Init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode);
uint8_t CAN2_Mode_Init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode);


typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    int32_t  all_ecd;
    int32_t  count;

    uint8_t temperate;
    int16_t last_ecd;
	float pid_set_speed;
} motor_measure_t;

typedef struct
{
	motor_measure_t M3508;
	PidType M3508_PID[2];
	float pid_set_speed_3508;
}M3508;

//typedef struct
//{
//	motor_measure_t GM6020;
//	float pid_set_angle_6020;

//}GM6020;

typedef struct
{
	motor_measure_t  M2006;
	PidType M2006_PID[2];
	float pid_set_angle_2006;

}M2006;

#define get_motor_measure(ptr, rx_message)                                              \
{                                                                                       \
    if((ptr)->ecd - (ptr)->last_ecd > 4096) (ptr)->count-- ;                            \
		else if((ptr)->ecd - (ptr)->last_ecd < -4096 ) (ptr)->count ++ ;											\
    (ptr)->last_ecd = (ptr)->ecd;                                                       \
    (ptr)->ecd = (uint16_t)((rx_message).Data[0] << 8 | (rx_message).Data[1]);          \
    (ptr)->speed_rpm = (uint16_t)((rx_message).Data[2] << 8 |(rx_message).Data[3]);     \
    (ptr)->given_current = (uint16_t)((rx_message).Data[4] << 8 | (rx_message).Data[5]); \
    (ptr)->temperate = (rx_message).Data[6];                                             \
    (ptr)->all_ecd=(ptr)->count*8191+(ptr)->ecd;                                     \
}


#endif

