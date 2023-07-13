#ifndef __GPIO_H
#define __GPIO_H
#include "sys.h"

#define ON 1
#define OFF 0

#define PB13(a)  if(a)\
				 GPIO_SetBits(GPIOB,GPIO_Pin_13);\
		 else    GPIO_ResetBits(GPIOB,GPIO_Pin_13);\

#define PB14(a)  if(a)\
				 GPIO_SetBits(GPIOB,GPIO_Pin_14);\
		 else    GPIO_ResetBits(GPIOB,GPIO_Pin_14);\

#define PB15(a)  if(a)\
				 GPIO_SetBits(GPIOB,GPIO_Pin_15);\
		 else    GPIO_ResetBits(GPIOB,GPIO_Pin_15);\

void gpio_init(void);

#endif

