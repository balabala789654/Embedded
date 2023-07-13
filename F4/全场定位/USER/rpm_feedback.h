#ifndef _RPM_FEEDBACK_H
#define _RPM_FEEDBACK_H

#include "as5047.h"
#include "can.h"
#include "usart.h"

int rpm_send_to_master(int std, int n1, int n2);
float rpm_cul_L(int check_angle, int target_angle);
float rpm_cul_R(int check_angle, int target_angle);
int read_initial_angle(void);


#endif

