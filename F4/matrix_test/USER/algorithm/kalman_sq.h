#ifndef _KALMAN_SQ_H
#define _KALMAN_SQ_H

#include "arm_math.h"

/*

*/
typedef struct
{
	float state_mat_data[2];//״̬
	float cov_mat_data[4];//Э�������
	float state_trans_mat_data[4];//״̬ת�ƾ���
	float cov_state_trans_mat_data[4];//״̬ת��Э�������
	float observe_mat_data[2];//�۲����
	float control_mat_data[2];//���ƾ���
	float Observe_variance[1];
	float kalman_gain[2];//kalman����
	float sensor_data[1];//����������
	
	
	
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

