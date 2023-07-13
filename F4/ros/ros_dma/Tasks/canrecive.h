#ifndef __CANTASK_H
#define __CANTASK_H

#include "can.h"
#include "motortask.h"

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


void moter_send_3508(int i1,int i2,int i3,int i4);
void motor_send_6020(int i1,int i2,int i3,int i4);
//void CAN1_RX0_IRQnHandler(void);
//void CAN2_RX1_IRQnHandler(void);


#endif
