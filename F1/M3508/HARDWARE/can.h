#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    

u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
void USB_LP_CAN1_RX0_IRQHandler(void);
void moter_send_3508(int i1,int i2,int i3,int i4);
void motor_send_6020(int i1,int i2,int i3,int i4);



typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    int32_t  all_ecd;
    int32_t  count;

    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef struct
{
	motor_measure_t M3508[4];
	float pid_set_speed;
	float pid_set_angle;
}M3508;

typedef struct
{
	motor_measure_t GM6020;
	float pid_set_speed_6020;
	float pid_set_angle_6020;

}GM6020;


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

#define get_motor_measure_6020(ptr, rx_message)                                              \
{                                                                                       \
    if((ptr)->ecd - (ptr)->last_ecd > 4096) (ptr)->count-- ;                            \
		else if((ptr)->ecd - (ptr)->last_ecd < -4096 ) (ptr)->count ++ ;											\
    (ptr)->last_ecd = (ptr)->ecd;                                                       \
    (ptr)->ecd = (float)((rx_message).Data[0] << 8 | (rx_message).Data[1]);          \
    (ptr)->speed_rpm = (float)((rx_message).Data[2] << 8 |(rx_message).Data[3]);     \
    (ptr)->given_current = (float)((rx_message).Data[4] << 8 | (rx_message).Data[5]); \
    (ptr)->temperate = (rx_message).Data[6];                                             \
    (ptr)->all_ecd=(ptr)->count*8191+(ptr)->ecd;                                   \
}

#endif









