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

// 6020����ֵ������,ÿһ�ε�����°�װ����Ҫ����У׼����ֵ������ɻ�е׼ȷȷ������ֵ���Է�ֹ����ƫ��
#define M6020_m1 -1 / ECD
#define M6020_m2 6699 / ECD
#define M6020_m3 -1 / ECD
#define M6020_m4 640 / ECD


//==========3508���pid���ݳ�ʼ��===========
#define M3508_SPEED_PID_KP 10
#define M3508_SPEED_PID_KI 0
#define M3508_SPEED_PID_KD 0
#define M3508_SPEED_OUT_MAX 16000
#define M3508_SPEED_MAX_IOUT 2000.0
//===========6020������ݳ�ʼ����˫����=============
#define M6020_SPEED_PID_KP 80
#define M6020_SPEED_PID_KI 0
#define M6020_SPEED_PID_KD 0
#define M6020_SPEED_OUT_MAX 30000.0
#define M6020_SPEED_MAX_IOUT 2000.0
#define ECD 8192.0f * 360.0f
#define M6020_ECD_PID_KP 15
#define M6020_ECD_PID_KI 0
#define M6020_ECD_PID_KD 0

//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 			5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 				0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 			1.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT  		8000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 		100.0f


#define AGV 1 //�Ƿ�ʹ�ö��ֲ���

#define ANGLE_TO_RAD 0.01745f

extern void chassis_task(void *pvParameters);
extern void Steering_wheel(int vx, int vy, int vz);
typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    const Encoder_process_t *chassis_encoder_measure;
	  fp32 relative_angle;
} Chassis_Motor_t;
//����������ݵĽṹ��
typedef struct
{
    const RC_ctrl_t *AGV_rc;
    const ext_game_robot_status_t *gimbal_status_measure; //���̲���ϵͳ���ʶ�ȡ

    fp32 set_6020[4]; //����3508��6020�������ֵ
    fp32 set_3508[4];

    Chassis_Motor_t motor_chassis_3508[4]; //�������
    Chassis_Motor_t motor_chassis_6020[4];

    fp32 real_vz, real_vx, real_vy; //���ý���С����ģʽʱ��������

    float RC_X_ChassisSpeedRef; //���Ҷ�̬���룬���ڼ�������ʱ��С���ݱ�־
    float RC_Y_ChassisSpeedRef; //ǰ��̬����

    PidTypeDef PID_6020_speed[4]; // pid�����ṹ��Ķ���
    PidTypeDef PID_6020_angle[4];
    PidTypeDef Righting; //���ԣ����ýǶȲ���Ϊ�������Ӷ�ʹ��pid��Ť�����и���Ļ�������
    PidTypeDef PID_3508[4];

    fp32 change1, change2, change3, change4;   //�Զ��ֵ�ǰ״̬���б��棬��ֹ����״̬�л���ʱ����ַ�ת
    const Gimbal_Motor_t *chassis_yaw_motor;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����
    const Gimbal_Motor_t *chassis_pitch_motor; //����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����

    const ext_power_heat_data_t *chassis_power_measure;    //��ȡ����ϵͳ������ݵĽṹ��
    const ext_game_robot_status_t *chassis_status_measure; //���̲���ϵͳ���ʶ�ȡ
    const ext_robot_hurt_t *chassis_hurt_type;
    const monitor_t *chassis_monitor_point; //�������

    int rc_mode;
} AGV_chassis_t;

typedef enum
{
    AGV_CHASSIS_RELAX = 0,         //��������ģʽ
    AGV_CHASSIS_FOLLOW_GIMBAL = 1, //�����Զ�������̨
    AGV_CHASSIS_DODGE_MODE = 2,    //С����
    AGV_CHASSIS_KEY_MODE = 3,      //��������ģʽ
    CHASSIS_FOLLOW_GIMBAL = 4,      //�����������ģʽ
} chassis_mode_e;

#endif
