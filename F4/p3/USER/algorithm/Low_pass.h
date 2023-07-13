#ifndef _LOW_PASS_H
#define _LOW_PASS_H

#include "Low_pass.h"
#include "dead_zone.h"
#include "p3.h"

typedef struct 
{
	float ch[5];
}LOW_PASS;

LOW_PASS Low_pass_output(p3 (*input)(uint8_t _data[13]));

#endif

