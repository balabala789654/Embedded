#ifndef _ARM_4_H
#define _ARM_4_H

#define pi 3.14159
#include "math.h"
#include "CyberGear.h"
#include "CubeMars_AK80_8.h"

typedef struct{
	float l1,l2,l3;
	float theta_ori, theta_1, theta_2, theta_3;
	float alpha;
	float beta;
	
	float joint_position[4];
	
}_arm;

extern _arm arm;

#endif

