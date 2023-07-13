#ifndef _AS5047_H
#define _AS5047_H

#include "sys.h"
#include "spi.h"

int read_as5047_L(void);
int read_as5047_R(void);

uint8_t parity_even(uint16_t v);
unsigned int parity(unsigned x);

	
#endif

