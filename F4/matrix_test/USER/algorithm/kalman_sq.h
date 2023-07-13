#ifndef _KALMAN_SQ_H
#define _KALMAN_SQ_H

#include "arm_math.h"

/*

*/
typedef struct
{
	float state_mat_data[2];//状态
	float cov_mat_data[4];//协方差矩阵
	float state_trans_mat_data[4];//状态转移矩阵
	float cov_state_trans_mat_data[4];//状态转移协方差矩阵
	float observe_mat_data[2];//观测矩阵
	float control_mat_data[2];//控制矩阵
	float Observe_variance[1];
	float kalman_gain[2];//kalman增益
	float sensor_data[1];//传感器数据
	
	
	
}KALMAN_PARAMS;

typedef struct
{
	
	arm_matrix_instance_f32 mat_X;
	arm_matrix_instance_f32 mat_P;
	arm_matrix_instance_f32 mat_F;
	arm_matrix_instance_f32 mat_Q;
	arm_matrix_instance_f32 mat_H;
	arm_matrix_instance_f32 mat_K;
	arm_matrix_instance_f32 mat_Z;
	
	arm_matrix_instance_f32 _mat_X;
	arm_matrix_instance_f32 _mat_P;
	
	arm_matrix_instance_f32 mat_R;
}KALMAN_MAT;

#endif

