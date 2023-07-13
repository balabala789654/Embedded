#include "location_sensor.h"
#include "as5047.h"
/*
全场定位中
			左轮对应SPI3
			右轮对应SPI2
			
			轮子的直径为58mm
				  周长为0.18m左右



                              ----------------
                               \            /
								\          /
								 \        / 
								  \      /
								   \    /
									\  / 
									 \/

*/
#define wheel_R 0.029
#define pi 3.14159

//左轮参数
static char L_compare_angle_set_flag=1;
static int L_compare_angle_L;
static int L_compare_angle_R;
static int L_current_angle;
float L_location;

//右轮参数
static char R_compare_angle_set_flag=1;
static int R_compare_angle_L;
static int R_compare_angle_R;
static int R_current_angle;
float R_location;

int read_initial_angle_L(void)
{
	int angle;
	angle = read_as5047_L();
	return angle;
}

int read_initial_angle_R(void)
{
	int angle;
	angle = read_as5047_R();
	return angle;
}

/*
	*****************************左轮********************************
*/
float cul_location_L(int _diff_angle, int _initial_angle)
{
	L_current_angle = read_initial_angle_L();
	
	switch(compare_angle_set_L(_diff_angle, _initial_angle, L_current_angle))
	{
		case 1:
		{
			L_location-=(2*pi*wheel_R/4);
			break;
		}
		case 2:
		{
			L_location+=(2*pi*wheel_R/4);
			break;
		}
		case 0:break;
	}
	return L_location;
	
	
}
int compare_angle_set_L(int _diff_angle, int _initial_angle, int _current_angle)
{
	if(L_compare_angle_set_flag)
	{
		L_compare_angle_L = _current_angle + _diff_angle;
		L_compare_angle_R = _current_angle - _diff_angle;
		L_compare_angle_set_flag=0;
	}

	if(L_compare_angle_L>16383)
		L_compare_angle_L = L_compare_angle_L-16383;

	if(L_compare_angle_R<0)
		L_compare_angle_R = 16383+L_compare_angle_R;
	
	
	
	
	if(_current_angle >= L_compare_angle_L-100 && _current_angle <= L_compare_angle_L+100)
	{
		L_compare_angle_R = L_compare_angle_L-_diff_angle;
		L_compare_angle_L = L_compare_angle_L+_diff_angle;
		return 1;
		
	}
	else if(_current_angle >= L_compare_angle_R-100 && _current_angle <= L_compare_angle_R+100)
	{
		L_compare_angle_L = L_compare_angle_R+_diff_angle;
		L_compare_angle_R = L_compare_angle_R-_diff_angle;
		return 2;
	}
	else return 0;
	
}

/*
	*****************************左轮********************************
*/

/*
	*****************************右轮********************************
*/
float cul_location_R(int _diff_angle, int _initial_angle)
{
	R_current_angle = read_initial_angle_R();
	
	switch(compare_angle_set_R(_diff_angle, _initial_angle, R_current_angle))
	{
		case 1:
		{
			R_location+=(2*pi*wheel_R/4);
			break;
		}
		case 2:
		{
			R_location-=(2*pi*wheel_R/4);
			break;
		}
		case 0:break;
	}
	return R_location;
	
}

int compare_angle_set_R(int _diff_angle, int _initial_angle, int _current_angle)
{
	if(R_compare_angle_set_flag)
	{
		R_compare_angle_L = _current_angle + _diff_angle;
		R_compare_angle_R = _current_angle - _diff_angle;
		R_compare_angle_set_flag=0;
	}

	if(R_compare_angle_L>16383)
		R_compare_angle_L = R_compare_angle_L-16383;

	if(R_compare_angle_R<0)
		R_compare_angle_R = 16383+R_compare_angle_R;
	
	
	
	
	if(_current_angle >= R_compare_angle_L-100 && _current_angle <= R_compare_angle_L+100)
	{
		R_compare_angle_R = R_compare_angle_L-_diff_angle;
		R_compare_angle_L = R_compare_angle_L+_diff_angle;
		return 1;
		
	}
	else if(_current_angle >= R_compare_angle_R-100 && _current_angle <= R_compare_angle_R+100)
	{
		R_compare_angle_L = R_compare_angle_R+_diff_angle;
		R_compare_angle_R = R_compare_angle_R-_diff_angle;
		return 2;
	}
	else return 0;
	
}


/*
	*****************************右轮********************************
*/

/*
	*****************************世界坐标系解算********************************
*/

float32_t x;
float32_t y;

float32_t out_coordinate_x(float32_t *_L_location, float32_t *_R_location)
{
	float32_t _cos_arr = arm_cos_f32(pi/4);
	float32_t _sin_arr = arm_sin_f32(pi/4);
	
	float32_t _mult_arr_1;
	float32_t _mult_arr_2;
	
	arm_mult_f32(&_cos_arr, _L_location, &_mult_arr_1, 1);
	arm_mult_f32(&_cos_arr, _R_location, &_mult_arr_2, 1);
	
	_mult_arr_1 = -_mult_arr_1;
	
	arm_add_f32(&_mult_arr_1, &_mult_arr_2, &x, 1);
	return x;
}

float32_t out_coordinate_y(float32_t *_L_location, float32_t *_R_location)
{
	float32_t _cos_arr = arm_cos_f32(pi/4);
	float32_t _sin_arr = arm_sin_f32(pi/4);
	
	float32_t _mult_arr_1;
	float32_t _mult_arr_2;
	
	arm_mult_f32(&_sin_arr, _L_location, &_mult_arr_1, 1);
	arm_mult_f32(&_sin_arr, _R_location, &_mult_arr_2, 1);
		
	arm_add_f32(&_mult_arr_1, &_mult_arr_2, &y, 1);
	return y;
}


/*
	*****************************世界坐标系解算********************************
*/


