#include "KalmanFilter.h"
#include "gpio.h"
/*

ÂË²¨¼´¼ÓÈ¨

¿¨¶ûÂü¹«Ê½£»
	Ô¤²â£º
			_X=F*X+B*U
			_P=F*P*F'+Q 
	¸üÐÂ£º
			K=_P*H'/(H*_P*H'+R)
			X=_X+K*(Z-H*_X)
			P=(I-K*H)*_P
*/
KALMAN_FILTER_MAT kalman_filter_mat;
KALMAN_FILTER_DATA kalman_filter_data;

void mat_data_init(KALMAN_FILTER_DATA *data)
{
	data->X_data[0]=0;
	data->X_data[1]=0;
	
	data->P_data[0]=1;
	data->P_data[1]=0;
	data->P_data[2]=0;
	data->P_data[3]=1;
	
	data->F_data[0]=1;
	data->F_data[1]=0;
	data->F_data[2]=1;
	data->F_data[3]=1;
	
	data->H_data[0]=0;
	data->H_data[1]=1;
	
	data->K_data[0]=0;
	data->K_data[1]=0;
	
	data->Z_data[0]=0;
	
	data->Q_data[0]=0.0001;
	data->Q_data[1]=0;
	data->Q_data[2]=0;
	data->Q_data[3]=0.0001;
	
	data->R_data[0]=1;
	
	data->_X_data[0]=0;
	data->_X_data[1]=0;
	
	data->_P_data[0]=0;
	data->_P_data[1]=0;
	data->_P_data[2]=0;
	data->_P_data[3]=0;
	
}


void kalman_mat_init(KALMAN_FILTER_MAT *mat, KALMAN_FILTER_DATA *data)
{
	mat_init(&mat->X, 2, 1, data->X_data);
	mat_init(&mat->P, 2, 2, data->P_data);
	mat_init(&mat->F, 2, 2, data->F_data);
	mat_init(&mat->Q, 2, 2, data->Q_data);
	mat_init(&mat->H, 1, 2, data->H_data);
	mat_init(&mat->R, 1, 1, data->R_data);
	mat_init(&mat->K, 2, 1, data->K_data);
	mat_init(&mat->Z, 1, 1, data->Z_data);
	mat_init(&mat->_X, 2, 1, data->_X_data);
	mat_init(&mat->_P, 2, 2, data->_P_data);
	
}


void once_loop_predict(KALMAN_FILTER_MAT *_mat)
{
	float data1[10];
	float data2[10];
	mat mat1;
	mat mat2;
	
	mat_init(&mat1, 2, 2, data1);
	mat_init(&mat2, 2, 2, data2);
	
	/*		  _X=F*X+B*U						*/
	if(mat_mult(&_mat->F, &_mat->X, &_mat->_X)==ARM_MATH_SIZE_MISMATCH) LED2=0;
	
	/*		  _P=F*P*F'+Q						*/
	if(mat_mult(&_mat->F, &_mat->P, &mat1)==ARM_MATH_SIZE_MISMATCH) LED2=0;
	if(mat_trans(&_mat->F, &mat2)==ARM_MATH_SIZE_MISMATCH) LED2=0;
	if(mat_mult(&mat1, &mat2, &mat1)==ARM_MATH_SIZE_MISMATCH) LED2=0;
	if(mat_add(&mat1, &_mat->Q, &_mat->_P)==ARM_MATH_SIZE_MISMATCH) LED2=0;
} 

void once_loop_update(KALMAN_FILTER_MAT *_mat)
{
	float data1[10];
	float data2[10];
	float data3[10];
	mat mat1;
	mat mat2;
	mat mat3;
	
	/*     ¿¨¶ûÂüÔöÒæ¸üÐÂ ¾ØÕóK     K=_P*H'/(H*_P*H'+R) 	*/
	mat_init(&mat1, 1, 2, data1);
	mat_init(&mat2, 2, 1, data2);
	mat_init(&mat3, 1, 1, data3);
	
	if(mat_mult(&_mat->H, &_mat->_P, &mat1)==ARM_MATH_SIZE_MISMATCH) 
		LED2=0;
	if(mat_trans(&_mat->H, &mat2)==ARM_MATH_SIZE_MISMATCH) 
		LED2=0;
	if(mat_mult(&mat1, &mat2, &mat3)==ARM_MATH_SIZE_MISMATCH) 
		LED2=0;
	
	mat_init(&mat1, 1, 1, data1);
	if(mat_add(&mat3, &_mat->R, &mat1)==ARM_MATH_SIZE_MISMATCH) 
		LED2=0;
	
	mat_init(&mat2, 1, 1, data2);
	if(mat_inv(&mat1, &mat2)==ARM_MATH_SIZE_MISMATCH)
		LED2=0;
	
	mat_init(&mat3, 2, 1, data3);
	mat_init(&mat1, 2, 1, data1);
	if(mat_trans(&_mat->H, &mat3)==ARM_MATH_SIZE_MISMATCH) 
		LED2=0;
	if(mat_mult(&_mat->_P, &mat3, &mat1)==ARM_MATH_SIZE_MISMATCH) 
		LED2=0;
	
	if(mat_mult(&mat1, &mat2, &_mat->K)==ARM_MATH_SIZE_MISMATCH) 
		LED2=0;
	
	
	/*     ¿¨¶ûÂü×´Ì¬¸üÐÂ ¾ØÕóX      X=_X+K*(Z-H*_X)	*/
	mat_init(&mat1, 1, 1, data1);
	if(mat_mult(&_mat->H, &_mat->_X, &mat1)==ARM_MATH_SIZE_MISMATCH) 
		LED2=0;
	
	mat_init(&mat2, 1, 1, data2);
	if(mat_sub(&_mat->Z, &mat1, &mat2)==ARM_MATH_SIZE_MISMATCH) 
		LED2=0;
	
	mat_init(&mat1, 2, 1, data1);
	if(mat_mult(&_mat->K, &mat2, &mat1)==ARM_MATH_SIZE_MISMATCH) 
		LED2=0;
	
	if(mat_add(&_mat->_X, &mat1, &_mat->X)==ARM_MATH_SIZE_MISMATCH)
		LED2=0;
	
	
	/*     ¿¨¶ûÂüÐ­·½²î¾ØÕó¸üÐÂ ¾ØÕóP       P=(I-K*H)*_P	*/
	mat_init(&mat1, 2, 2, data1);
	if(mat_mult(&_mat->K, &_mat->H, &mat1)==ARM_MATH_SIZE_MISMATCH) 
		LED2=0;
	
	mat_init(&mat2, 2, 2, data2);
	if(mat_mult(&mat1, &_mat->_P, &mat2)==ARM_MATH_SIZE_MISMATCH) 
		LED2=0;
	
	if(mat_sub(&_mat->_P, &mat2, &_mat->P)==ARM_MATH_SIZE_MISMATCH) 
		LED2=0;
	
	
}

void kalman_filter_loop(void)
{
	once_loop_predict(&kalman_filter_mat);
	once_loop_update(&kalman_filter_mat);
}

