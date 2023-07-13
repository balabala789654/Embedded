#ifndef _LOCATION_SENSOR_H
#define _LOCATION_SENSOR_H

#include "arm_math.h"
int read_initial_angle_L(void);
int read_initial_angle_R(void);
float cul_location_L(int _diff_angle, int _initial_angle);
float cul_location_R(int _diff_angle, int _initial_angle);

int compare_angle_set_L(int _diff_angle, int _initial_angle, int _current_angle);
int compare_angle_set_R(int _diff_angle, int _initial_angle, int _current_angle);

float32_t out_coordinate_x(float32_t *_L_location, float32_t *_R_location);
float32_t out_coordinate_y(float32_t *_L_location, float32_t *_R_location);

#endif

