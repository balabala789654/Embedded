#ifndef GIMBAL_APP_H
#define GIMBAL_APP_H
#include "stm32f4xx.h"
#include "string.h"
#include "stdio.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "arm_math.h"
#include "user_lib.h"
#include "can.h"
#include "CAN_receive.h"
#include "pid.h"

/***********undefine****ͷ�ļ��ظ�********/

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];
    fp32 error[3];
    fp32 out;

    fp32 Dead_Zone; //����
    fp32 gama;
    float I_Separation;
    fp32 angle_max;
    fp32 angle_min;
    fp32 lastdout;
    fp32 lastout;
} Gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;     //�������
    const Encoder_process_t *gimbal_encoder_measure; //����������
    PidTypeDef gimbal_motor_gyro_pid;
    Gimbal_PID_t gimbal_motor_angle_pid;

    uint16_t offset_ecd;
    fp32 relative_angle;
    fp32 absolute_angle;
    fp32 motor_gyro;
    fp32 motor_gyro_set;
    fp32 gimbal_angle_set;
    fp32 gimbal_for_angle_set;
    fp32 gimbal_turn_around;
    fp32 current_set;      //ͨ��pid���㣬����current������give_current
    int16_t given_current; //���ֱֵ�Ӵ������

    extKalman_t Error_Kalman; //����һ����̨�Ƕ�������kalmanָ��
} Gimbal_Motor_t;
void gimbal_pid_reset(Gimbal_PID_t *pid, float p, float i, float d,float Dead_Zone, float gama, float angle_max, float angle_min, float I_Separation);
void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd, float Dead_Zone, float gama, float angle_max, float angle_min, float I_Separation);
extern fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta, extKalman_t Gim); // PID����
int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset);                                      //�õ���ԽǶ�
extern void gimbal_relative_angle_control(Gimbal_Motor_t *gimbal_motor);                               //��̨��ԽǶȿ���
extern void gimbal_absolute_angle_control(Gimbal_Motor_t *gimbal_motor);                               //��̨���ԽǶȿ���

#endif
