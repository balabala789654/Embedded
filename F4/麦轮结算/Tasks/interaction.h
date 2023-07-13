#ifndef __INTERACTION_H
#define __INTERACTION_H

#include "main.h"

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

void interaction_task(void *pvParameters);
extern float speed_yaw;
extern float _vx, _vy;
#endif

