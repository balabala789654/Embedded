#ifndef _LOW_PASS_FILTER_H
#define _LOW_PASS_FILTER_H

typedef struct
{
	float output[2];
}LOW_PASS_FILTER;

float low_pass_filter(float _p, int input);

#endif

