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
#ifndef PID_H
#define PID_H
#include "stm32f4xx.h"
#include "string.h"
#include "math.h"
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;
enum PID_MODE
{
  PID_POSITION = 0,
  PID_DELTA
};

typedef struct
{
  uint8_t mode;
  // PID 三参数
  fp32 Kp;
  fp32 Ki;
  fp32 Kd;

  fp32 max_out;  //最大输出
  fp32 max_iout; //最大积分输出

  fp32 set;
  fp32 fdb;

  fp32 out;
  fp32 Pout;
  fp32 Iout;
  fp32 Dout;
  fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
  fp32 error[3]; //误差项 0最新 1上一次 2上上次

  fp32 Dead_Zone; //死区
  fp32 gama;
  float I_Separation;
  fp32 angle_max;
  fp32 angle_min;
  fp32 lastdout;
  fp32 lastout;

} PidTypeDef;
void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, float max_iout, float Dead_Zone, float gama, float angle_max, float angle_min, float I_Separation);
extern fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
extern void PID_clear(PidTypeDef *pid);
void pid_reset(PidTypeDef *pid, float p, float i, float d, float Dead_Zone, float gama, float angle_max, float angle_min, float I_Separation);
#endif
