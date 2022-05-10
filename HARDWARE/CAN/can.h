#ifndef __CAN_H
#define __CAN_H

#include "sys.h"

void can1_Init(void);
u16 can_Send(u16* msg,u16 len);
u8 can_Receive(u8* buf);

#endif



