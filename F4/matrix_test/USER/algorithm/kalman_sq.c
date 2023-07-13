#include "kalman_sq.h"
#include "arm_math.h"

KALMAN_PARAMS kalman_params;
KALMAN_MAT kalman_mat;

/*
X = [0; 0];%×´Ì¬
P = [1 0; 0 1];%Ð­·½²î¾ØÕó
F = [1 1; 0 1];%×´Ì¬×ªÒÆ¾ØÕó
Q = [0.00001 0; 0 0.00001];%×´Ì¬×ªÒÆÐ­·½²î¾ØÕó
H = [1 0];%¹Û²â¾ØÕó
R = 1;%¹Û²âÔëÉù·½²î
*/

void kalman_params_init(KALMAN_PARAMS *_kalman)
{
	_kalman->state_mat_data[0]=0;
	_kalman->state_mat_data[1]=0;
	
	_kalman->cov_mat_data[0]=1;
	_kalman->cov_mat_data[1]=0;
	_kalman->cov_mat_data[2]=0;
	_kalman->cov_mat_data[3]=1;
	
	_kalman->state_trans_mat_data[0]=1;
	_kalman->state_trans_mat_data[1]=1;
	_kalman->state_trans_mat_data[2]=0;
	_kalman->state_trans_mat_data[3]=1;

	_kalman->cov_state_trans_mat_data[0]=0.0001;
	_kalman->cov_state_trans_mat_data[1]=0;
	_kalman->cov_state_trans_mat_data[2]=0;
	_kalman->cov_state_trans_mat_data[3]=0.0001;
	
	_kalman->observe_mat_data[0]=1;
	_kalman->observe_mat_data[1]=0;
	
	_kalman->Observe_variance[0]=1;
	
	_kalman->kalman_gain[0]=0;
	_kalman->kalman_gain[1]=0;
	
	_kalman->sensor_data[0]=0;
}



void kalman_mat_init(KALMAN_MAT *_kalman_mat, KALMAN_PARAMS *_kalman)
{
	arm_mat_init_f32(&_kalman_mat->mat_X, 2, 1, _kalman->state_mat_data);
	arm_mat_init_f32(&_kalman_mat->mat_P, 2, 2, _kalman->cov_mat_data);
	arm_mat_init_f32(&_kalman_mat->mat_F, 2, 2, _kalman->state_trans_mat_data);
	arm_mat_init_f32(&_kalman_mat->mat_Q, 2, 2, _kalman->cov_state_trans_mat_data);
	arm_mat_init_f32(&_kalman_mat->mat_H, 1, 2, _kalman->observe_mat_data);
	arm_mat_init_f32(&_kalman_mat->mat_R, 1, 1, _kalman->Observe_variance);
	arm_mat_init_f32(&_kalman_mat->mat_K, 2, 1, _kalman->kalman_gain);
	arm_mat_init_f32(&_kalman_mat->mat_Z, 1, 1, _kalman->sensor_data);
}


/**
	*@brief 
	*@param 
	*@param	
	*@retval 
**/
void once_loop_predict(KALMAN_MAT *_kalman_mat)
{
	arm_matrix_instance_f32 mat1;
	arm_matrix_instance_f32 mat2;
	arm_matrix_instance_f32 mat3;
	
	arm_mat_mult_f32(&_kalman_mat->mat_F, &_kalman_mat->mat_X, &_kalman_mat->_mat_X);
	
	arm_mat_mult_f32(&_kalman_mat->mat_F, &_kalman_mat->mat_P, &mat1);
	arm_mat_trans_f32(&_kalman_mat->mat_F, &mat2);
	arm_mat_mult_f32(&mat1, &mat2, &mat3);
	arm_mat_add_f32(&mat3, &_kalman_mat->mat_Q, &_kalman_mat->_mat_P);
	
}

/**
	*@brief 
	*@param 
	*@param	
	*@retval 
**/
void _kalman_gain_update(KALMAN_MAT *_kalman_mat)
{
	arm_matrix_instance_f32 mat1;
	arm_matrix_instance_f32 mat2;
	arm_matrix_instance_f32 mat3;
	arm_matrix_instance_f32 mat4;
	arm_matrix_instance_f32 mat5;
	arm_matrix_instance_f32 mat6;
	
	arm_mat_trans_f32(&_kalman_mat->mat_H, &mat1);
	arm_mat_mult_f32(&_kalman_mat->mat_H, &_kalman_mat->_mat_P, &mat2);
	arm_mat_mult_f32(&mat2, &mat1, &mat3);
	arm_mat_add_f32(&mat3, &_kalman_mat->mat_R, &mat4);
	arm_mat_inverse_f32(&mat4, &mat5);
	arm_mat_mult_f32(&_kalman_mat->_mat_P, &mat1, &mat6);
	arm_mat_mult_f32(&mat6, &mat5, &_kalman_mat->mat_K);
	
}

/**
	*@brief 
	*@param 
	*@param	
	*@retval 
**/
void _kalman_state_update(KALMAN_MAT *_kalman_mat)
{
	arm_matrix_instance_f32 mat1;
	arm_matrix_instance_f32 mat2;
	arm_matrix_instance_f32 mat3;
	
	arm_mat_mult_f32(&_kalman_mat->mat_H, &_kalman_mat->_mat_X, &mat1);
	arm_mat_sub_f32(&_kalman_mat->mat_Z, &mat1, &mat2);
	arm_mat_mult_f32(&_kalman_mat->mat_K, &mat2, &mat3);
	arm_mat_add_f32(&_kalman_mat->_mat_X, &mat3, &_kalman_mat->mat_X);
}

/**
	*@brief 
	*@param 
	*@param	
	*@retval 
**/
void _kalman_cov_update(KALMAN_MAT *_kalman_mat)
{
	arm_matrix_instance_f32 mat1;
	arm_matrix_instance_f32 mat2;
	
	arm_mat_mult_f32(&_kalman_mat->mat_K, &_kalman_mat->mat_H, &mat1);
	arm_mat_mult_f32(&mat1, &_kalman_mat->_mat_P, &mat2);
	arm_mat_sub_f32(&_kalman_mat->_mat_P, &mat2, &_kalman_mat->mat_P);
}



void once_loop_update(void)
{
	_kalman_gain_update(&kalman_mat);
	_kalman_state_update(&kalman_mat);
	_kalman_cov_update(&kalman_mat);
	
}


