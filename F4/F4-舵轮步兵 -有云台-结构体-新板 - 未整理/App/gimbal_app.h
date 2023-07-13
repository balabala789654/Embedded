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

/***********undefine****头文件重复********/

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

    fp32 Dead_Zone; //死区
    fp32 gama;
    float I_Separation;
    fp32 angle_max;
    fp32 angle_min;
    fp32 lastdout;
    fp32 lastout;
} Gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;     //电机反馈
    const Encoder_process_t *gimbal_encoder_measure; //编码器反馈
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
    fp32 current_set;      //通过pid计算，设置current，传给give_current
    int16_t given_current; //这个值直接传给电机

    extKalman_t Error_Kalman; //定义一个云台角度误差卡尔曼kalman指针
} Gimbal_Motor_t;
void gimbal_pid_reset(Gimbal_PID_t *pid, float p, float i, float d,float Dead_Zone, float gama, float angle_max, float angle_min, float I_Separation);
void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd, float Dead_Zone, float gama, float angle_max, float angle_min, float I_Separation);
extern fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta, extKalman_t Gim); // PID解算
int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset);                                      //得到相对角度
extern void gimbal_relative_angle_control(Gimbal_Motor_t *gimbal_motor);                               //云台相对角度控制
extern void gimbal_absolute_angle_control(Gimbal_Motor_t *gimbal_motor);                               //云台绝对角度控制

#endif
