/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "pid.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, float max_iout, float Dead_Zone, float gama, float angle_max, float angle_min, float I_Separation)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
    pid->Dead_Zone = Dead_Zone;
    pid->I_Separation = I_Separation;
    pid->gama = gama;
    pid->angle_max = angle_max;
    pid->angle_min = angle_min;
}

fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set)
{
    int de;
    if (pid == 0)
    {
        return 0.0f;
    }
    pid->error[0] = set - ref;
    if (pid->angle_max != pid->angle_min) //使积分增长变为梯形，如果angle_min和angle_max相等则不启用
    {
        if (pid->error[0] > (pid->angle_max + pid->angle_min) / 2)
            pid->error[0] -= (pid->angle_max + pid->angle_min);
        else if (pid->error[0] < -(pid->angle_max + pid->angle_min) / 2)
            pid->error[0] += (pid->angle_max + pid->angle_min);
    }

    pid->set = set;
    pid->fdb = ref;

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
    }
    //只写增量式pid
    pid->Pout = pid->Kp * pid->error[0];

    pid->Iout += pid->Ki * pid->error[0];

    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);

    //计算微分项增量带不完全微分，可以减少高频干扰，gama应是一个0-1的值
    pid->Dout = pid->Kd * (1 - pid->gama) * (pid->Dbuf[0]) + pid->gama * pid->lastdout;
    LimitMax(pid->Iout, pid->max_iout);

    pid->out = pid->Pout + de * pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->lastdout = pid->Dout;
    pid->lastout = pid->out;
    return pid->out;
}

void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
/**
 * @brief     modify pid parameter when code running
 * @param[in] pid: control pid struct
 * @param[in] p/i/d: pid parameter
 * @retval    none
 */
void pid_reset(PidTypeDef *pid, float p, float i, float d, float Dead_Zone, float gama, float angle_max, float angle_min, float I_Separation)
{
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;

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
