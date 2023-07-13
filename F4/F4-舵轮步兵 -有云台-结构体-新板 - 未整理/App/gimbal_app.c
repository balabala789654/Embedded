#include "gimbal_app.h"

/*********************************************************************************************************************************************************************/
//使用电机相对角度控制
void gimbal_relative_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_angle_pid, gimbal_motor->relative_angle, gimbal_motor->gimbal_angle_set, gimbal_motor->gimbal_motor_measure->speed_rpm, gimbal_motor->Error_Kalman);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->gimbal_motor_measure->speed_rpm, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
//使用电机绝对角度控制电机
void gimbal_absolute_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->gimbal_angle_set, gimbal_motor->motor_gyro, gimbal_motor->Error_Kalman);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/*****************************/

void gimbal_pid_reset(Gimbal_PID_t *pid, float p, float i, float d, float Dead_Zone, float gama, float angle_max, float angle_min, float I_Separation)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = p;
    pid->ki = i;
    pid->kd = d;

    pid->Pout = 0;
    pid->Iout = 0;
    pid->Dout = 0;
    pid->out = 0;
    pid->Dead_Zone = Dead_Zone;
    pid->I_Separation = I_Separation;
    pid->gama = gama;
    pid->angle_max = angle_max;
    pid->angle_min = angle_min;
}

void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd, float Dead_Zone, float gama, float angle_max, float angle_min, float I_Separation)
{
    if (pid == NULL)
    {
        return;
    }
    if (fabs(pid->Dead_Zone) < 1e-5)
    {
        pid->Dead_Zone = 0;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
    pid->Dead_Zone = Dead_Zone;
    pid->I_Separation = I_Separation;
    pid->gama = gama;
    pid->angle_max = angle_max;
    pid->angle_min = angle_min;
}

fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta, extKalman_t Kal_Gim)
{
    fp32 err;
    int de;
    //    fp32 handle_err;

    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->get = get;
    pid->set = set;
    pid->error[0] = set - get;
    if (pid->angle_max != pid->angle_min) //使积分增长变为梯形，如果angle_min和angle_max相等则不启用
    {
        if (pid->error[0] > (pid->angle_max + pid->angle_min) / 2)
            pid->error[0] -= (pid->angle_max + pid->angle_min);
        else if (pid->error[0] < -(pid->angle_max + pid->angle_min) / 2)
            pid->error[0] += (pid->angle_max + pid->angle_min);
    }
    if (fabs(pid->error[0]) < pid->Dead_Zone) //死区，可以使pid系统更快的稳定下来，但是不能给太大，会导致系统滞后，且存在较大静差
    {
        pid->error[0] = pid->error[1] = 0;
    }
    if (fabs(pid->error[0]) > pid->I_Separation) //误差过大，采用积分分离
    {
        de = 0;
    }
    else
    {
        de = 1;
        pid->Iout += pid->ki * pid->error[0];
    }

    pid->error[0] = KalmanFilter(&Kal_Gim, pid->error[0]); //卡尔曼处理角度误差
    pid->Pout = pid->kp * pid->error[0];
    pid->Dout = pid->kd * error_delta * (1 - pid->gama) * (pid->Dbuf[0]) + pid->gama * pid->lastdout;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + de * pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

//得到电机相对角度
int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
    int16_t tmp = 0;

    if (center_offset >= 4096)
    {
        if (raw_ecd > center_offset - 4096)
            tmp = raw_ecd - center_offset;
        else
            tmp = raw_ecd + 8192 - center_offset;
    }
    else
    {
        if (raw_ecd > center_offset + 4096)
            tmp = raw_ecd - 8192 - center_offset;
        else
            tmp = raw_ecd - center_offset;
    }

    return tmp;
}
