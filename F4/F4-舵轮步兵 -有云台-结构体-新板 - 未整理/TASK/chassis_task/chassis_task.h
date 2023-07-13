#ifndef __MOTOR_TASK_H
#define __MOTOR_TASK_H
#include "pid.h"
#include "can.h"
#include "remote_control.h"
#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "stdlib.h"
#include "gimbal_task.h"
#include "get_judge_measure.h"
#include "CAN_Receive.h"
#include "Ramp_Control.h"
#include "gimbal_app.h"
//#include "kalman.h"

#define M3505_MOTOR_SPEED_PID_MAX_OUT 10000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 1000.0f

// 6020归中值的设置,每一次电机重新安装都需要重新校准居中值，最好由机械准确确定归中值，以防止其跑偏，
#define M6020_m1 -1 / ECD
#define M6020_m2 6699 / ECD
#define M6020_m3 -1 / ECD
#define M6020_m4 640 / ECD


//==========3508电机pid数据初始化===========
#define M3508_SPEED_PID_KP 10
#define M3508_SPEED_PID_KI 0
#define M3508_SPEED_PID_KD 0
#define M3508_SPEED_OUT_MAX 16000
#define M3508_SPEED_MAX_IOUT 2000.0
//===========6020电机数据初始化（双环）=============
#define M6020_SPEED_PID_KP 80
#define M6020_SPEED_PID_KI 0
#define M6020_SPEED_PID_KD 0
#define M6020_SPEED_OUT_MAX 30000.0
#define M6020_SPEED_MAX_IOUT 2000.0
#define ECD 8192.0f * 360.0f
#define M6020_ECD_PID_KP 15
#define M6020_ECD_PID_KI 0
#define M6020_ECD_PID_KD 0

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 			5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 				0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 			1.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT  		8000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 		100.0f


#define AGV 1 //是否使用舵轮步兵

#define ANGLE_TO_RAD 0.01745f

extern void chassis_task(void *pvParameters);
extern void Steering_wheel(int vx, int vy, int vz);
typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    const Encoder_process_t *chassis_encoder_measure;
	  fp32 relative_angle;
} Chassis_Motor_t;
//舵轮相关数据的结构体
typedef struct
{
    const RC_ctrl_t *AGV_rc;
    const ext_game_robot_status_t *gimbal_status_measure; //底盘裁判系统功率读取

    fp32 set_6020[4]; //设置3508和6020电机期望值
    fp32 set_3508[4];

    Chassis_Motor_t motor_chassis_3508[4]; //电机数据
    Chassis_Motor_t motor_chassis_6020[4];

    fp32 real_vz, real_vx, real_vy; //设置进入小陀螺模式时各个参数

    float RC_X_ChassisSpeedRef; //左右动态输入，用于键盘输入时的小陀螺标志
    float RC_Y_ChassisSpeedRef; //前后动态输入

    PidTypeDef PID_6020_speed[4]; // pid各个结构体的定义
    PidTypeDef PID_6020_angle[4];
    PidTypeDef Righting; //尝试：利用角度差作为反馈，从而使用pid对扭腰进行更快的回正处理
    PidTypeDef PID_3508[4];

    fp32 change1, change2, change3, change4;   //对舵轮当前状态进行保存，防止进行状态切换的时候舵轮疯转
    const Gimbal_Motor_t *chassis_yaw_motor;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
    const Gimbal_Motor_t *chassis_pitch_motor; //底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角

    const ext_power_heat_data_t *chassis_power_measure;    //获取裁判系统相关数据的结构体
    const ext_game_robot_status_t *chassis_status_measure; //底盘裁判系统功率读取
    const ext_robot_hurt_t *chassis_hurt_type;
    const monitor_t *chassis_monitor_point; //检测任务

    int rc_mode;
} AGV_chassis_t;

typedef enum
{
    AGV_CHASSIS_RELAX = 0,         //底盘无力模式
    AGV_CHASSIS_FOLLOW_GIMBAL = 1, //舵轮自动跟随云台
    AGV_CHASSIS_DODGE_MODE = 2,    //小陀螺
    AGV_CHASSIS_KEY_MODE = 3,      //按键控制模式
    CHASSIS_FOLLOW_GIMBAL = 4,      //底盘整体跟随模式
} chassis_mode_e;

#endif
