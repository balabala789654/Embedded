#include "arm_4.h"

_arm arm;
void joint_position_cal(float _x, float _y, float _z, float _angle){
	arm.joint_position[0] = _x;
	arm.joint_position[1] = _y;
	arm.joint_position[2] = _z;
}

float alpha_cal(float _x, float _y, float _z){
	float hypotenuse = sqrtf(_x*_x+_y*_y);
	return atan2f(_z, hypotenuse);
}

float beta_cal(float _x, float _y, float _z){
	float hypotenuse = sqrtf(_x*_x+_y*_y);
	return acosf((powf(hypotenuse,2)+powf(_z,2)+powf(arm.l1,2)-powf(arm.l2,2))/(2*sqrtf(powf(hypotenuse,2)+powf(_z,2))*arm.l1));
}
float* theta_cal(float _x, float _y, float _z){
	static float theta[4];
	float hypotenuse = sqrtf(_x*_x+_y*_y);
	theta[0] = atan2f(_y, _x);
	theta[1] = alpha_cal(_x, _y, _z)+beta_cal(_x, _y, _z);
	theta[2] = acosf((powf(hypotenuse,2)+ powf(_z,2)-powf(arm.l1,2)-powf(arm.l2,2))/(2*arm.l1*arm.l2));
	theta[3] = pi/2 - theta[1] - theta[2];
	return theta;
}



