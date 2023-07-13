#include "Low_pass.h"
#include "p3.h"

static LOW_PASS Low_pass_output_last;
static float p = 0.005;
extern uint8_t verify_data[13];

LOW_PASS Low_pass_output(p3 (*input)(uint8_t _data[13]))
{
	LOW_PASS _low_pass;
	for(int i=0; i<4; i++)
		_low_pass.ch[i] = ((*input)(verify_data).ch[i]*p + (Low_pass_output_last.ch[i]*(1-p)));
	Low_pass_output_last = _low_pass;
	return _low_pass;
}


