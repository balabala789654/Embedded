#ifndef __INTERACTION_H
#define __INTERACTION_H

#define MAX_interaction_byte 20 //上位机与下位机数据传输字节数
#define __pitch 1
#define __yaw 3
#define __roll 5
#define __vx 6
#define __vy 8
#define __vw 10
#define __vx_sign 12
#define __vy_sign 13
#define __vw_sign 14


#define __ros_vx 1
#define __ros_vy 3


void interaction_task(void *pvParameters);
float anti_conversion(float n);
float conversion(int RPM);

extern float speed_yaw;
extern float _vx, _vy;
#endif

