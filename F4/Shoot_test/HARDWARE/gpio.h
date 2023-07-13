#ifndef __GPIO_H
#define __GPIO_H

#include "sys.h"


//LED端口定义
#define LED1 PDout(2)	
#define LED2 PDout(3)	

void gpio_Init(void);//初始化		 				    
#endif
