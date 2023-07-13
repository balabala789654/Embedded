///*
//舵轮步兵底盘 lin
//主要采用全局变量的写法，没有采用结构体去简化变量
//实现，舵轮步兵的舵轮跟随云台，还有舵轮底盘的小陀螺算法解算
//以及初步完成舵轮步兵的键盘控制（WASD）12.25
//代码实现小陀螺解算，完成舵轮步兵的摆正操作，同时可以通过摆正操作进行第二种舵轮的控制---2022.2.23（待回校测试）
//下一步，将舵轮步兵设置为普通步兵的其中一种模式，从而实现步兵代码的控制。。。。2022.2.23
//舵轮步兵全局变量变为结构体变量 -----2022.3.10
//舵轮步兵基本功能测试完成，出现小陀螺旋转的过程中出现舵向电机不正常的反转，初步为解算出角度有正负的原因。 ------2022.5.5
//*/
#include "chassis_task.h"
#define number 12.12 // 3508直接获取通道值，number为其成比例的倍数
float ch0, ch1, res;
uint8_t FFlag_state = 0;
uint8_t CTRLFlag_state = 0;
float change_yaw;
AGV_chassis_t agv_chassis; //舵轮底盘相关结构体
extern Gimbal_Control_t gimbal_control;
extern float yaw_relative;
extern int yaw_count;
extern Super_power_t Super_power;
int count11 = 0, count22 = 0, count33 = 0, count44 = 0;
float  max_speed = number;
float change_mode1 = 0, change_mode2 = 0, change_mode3 = 0, change_mode4 = 0;
float DODGE_NUM = 1;
//因为底盘4个6020电机的电机性能可能会存在差别，所以使用4套pid精确控制
const static fp32 M3508_speed_pid[3] = {M3508_SPEED_PID_KP, M3508_SPEED_PID_KI, M3508_SPEED_PID_KD};
const static fp32 M6020_angle_pid1[3] = {M6020_ECD_PID_KP, M6020_ECD_PID_KI, M6020_ECD_PID_KD};
const static fp32 M6020_speed_pid1[3] = {M6020_SPEED_PID_KP, M6020_SPEED_PID_KI, M6020_SPEED_PID_KD};
const static fp32 M6020_angle_pid2[3] = {M6020_ECD_PID_KP, M6020_ECD_PID_KI, M6020_ECD_PID_KD};
const static fp32 M6020_speed_pid2[3] = {M6020_SPEED_PID_KP, M6020_SPEED_PID_KI, M6020_SPEED_PID_KD};
const static fp32 M6020_angle_pid3[3] = {M6020_ECD_PID_KP, M6020_ECD_PID_KI, M6020_ECD_PID_KD};
const static fp32 M6020_speed_pid3[3] = {M6020_SPEED_PID_KP, M6020_SPEED_PID_KI, M6020_SPEED_PID_KD};
const static fp32 M6020_angle_pid4[3] = {M6020_ECD_PID_KP, M6020_ECD_PID_KI, M6020_ECD_PID_KD};
const static fp32 M6020_speed_pid4[3] = {M6020_SPEED_PID_KP, M6020_SPEED_PID_KI, M6020_SPEED_PID_KD};
const static fp32 chassis_follow_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
void Steering_wheel(int vx, int vy, int vz);		  //舵轮的小陀螺解算
void Chassis_Power_Limit(AGV_chassis_t *agv_chassis); //舵轮的功率控制
void chassis_Init(AGV_chassis_t *agv_chassis);		  //舵轮初始化
void KEY_UPDATED(void);
void Super_power_ctrl(AGV_chassis_t *power_ctrl);
RampGen_t chassis_WRamp, chassis_ARamp, chassis_SRamp, chassis_DRamp;
void change_mode(void);
void chang(void);
void motor_speed_change(void); //用于速度预设
float sin_yaw = 0, cos_yaw = 0;
float all_relative_angle = 0;
float re_yaw = 0, ab_yaw;
int count_chassis, cha, F_flag = 0;
float change_Chassis_tan;
void chassis_task(void *pvParameters)
{
	float ch0, ch1, res;
	chassis_Init(&agv_chassis); //对底盘相关数据的初始化
	while (1)
	{
		motor_speed_change();
		all_relative_angle = (agv_chassis.chassis_yaw_motor->gimbal_motor_measure->all_ecd + 1405) / ECD;
		change_mode();
		Super_power_ctrl(&agv_chassis);
		// 3508电机pid控制
		PID_Calc(&agv_chassis.PID_3508[0], agv_chassis.motor_chassis_3508[0].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[0]);
		PID_Calc(&agv_chassis.PID_3508[1], agv_chassis.motor_chassis_3508[1].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[1]);
		PID_Calc(&agv_chassis.PID_3508[2], agv_chassis.motor_chassis_3508[2].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[2]);
		PID_Calc(&agv_chassis.PID_3508[3], agv_chassis.motor_chassis_3508[3].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[3]);
		// 6020电机pid双环控制
		PID_Calc(&agv_chassis.PID_6020_angle[0], agv_chassis.motor_chassis_6020[0].chassis_motor_measure->all_ecd / ECD, agv_chassis.set_6020[0]);
		PID_Calc(&agv_chassis.PID_6020_angle[1], agv_chassis.motor_chassis_6020[1].chassis_motor_measure->all_ecd / ECD, agv_chassis.set_6020[1]);
		PID_Calc(&agv_chassis.PID_6020_angle[2], agv_chassis.motor_chassis_6020[2].chassis_motor_measure->all_ecd / ECD, agv_chassis.set_6020[2]);
		PID_Calc(&agv_chassis.PID_6020_angle[3], agv_chassis.motor_chassis_6020[3].chassis_motor_measure->all_ecd / ECD, agv_chassis.set_6020[3]);
		PID_Calc(&agv_chassis.PID_6020_speed[0], agv_chassis.motor_chassis_6020[0].chassis_motor_measure->speed_rpm, agv_chassis.PID_6020_angle[0].out);
		PID_Calc(&agv_chassis.PID_6020_speed[1], agv_chassis.motor_chassis_6020[1].chassis_motor_measure->speed_rpm, agv_chassis.PID_6020_angle[1].out);
		PID_Calc(&agv_chassis.PID_6020_speed[2], agv_chassis.motor_chassis_6020[2].chassis_motor_measure->speed_rpm, agv_chassis.PID_6020_angle[2].out);
		PID_Calc(&agv_chassis.PID_6020_speed[3], agv_chassis.motor_chassis_6020[3].chassis_motor_measure->speed_rpm, agv_chassis.PID_6020_angle[3].out);
		if (agv_chassis.rc_mode == AGV_CHASSIS_FOLLOW_GIMBAL) //正常前进，舵轮跟随云台模式
		{

			agv_chassis.real_vz = 0; //小陀螺是否开启的标志
			ch0 = agv_chassis.AGV_rc->rc.ch[0];
			ch1 = agv_chassis.AGV_rc->rc.ch[1];
			res = atan2(ch0, ch1) * 180 / 3.14159; //对两个通道进行角度转换
												   //对现有的数据进行保存，以确保小陀螺开启的时候不会归回原位
			agv_chassis.change1 = yaw_count * 360;
			agv_chassis.change2 = yaw_count * 360;
			agv_chassis.change3 = yaw_count * 360;
			agv_chassis.change4 = yaw_count * 360;
			change_yaw = abs(agv_chassis.chassis_yaw_motor->gimbal_motor_measure->count + 1) * 360;

			//基础运动的代码实现---有bug无法进行左后方的转向
			//（因为对舵轮转向的原理是从0-90和0--90，当对-90做数据处理的时候会出现转向动的方向不对的问题）
			//通过云台的相对角度对底盘电机做相同角度的转向（未测试）
			if (res >= 0)
			{
				agv_chassis.set_6020[0] = M6020_m1 - yaw_relative + res - count11 * 360; // chassis_yaw_motor->relative_angle是从云台获取的相对角度
				agv_chassis.set_6020[1] = M6020_m2 - yaw_relative + res - count22 * 360;
				agv_chassis.set_6020[2] = M6020_m3 - yaw_relative + res - count33 * 360;
				agv_chassis.set_6020[3] = M6020_m4 - yaw_relative + res - count44 * 360;
				agv_chassis.set_3508[0] = (abs(agv_chassis.AGV_rc->rc.ch[1]) + abs(agv_chassis.AGV_rc->rc.ch[0])) * max_speed;
				agv_chassis.set_3508[1] = (abs(agv_chassis.AGV_rc->rc.ch[1]) + abs(agv_chassis.AGV_rc->rc.ch[0])) * max_speed;
				agv_chassis.set_3508[2] = (abs(agv_chassis.AGV_rc->rc.ch[1]) + abs(agv_chassis.AGV_rc->rc.ch[0])) * max_speed;
				agv_chassis.set_3508[3] = (abs(agv_chassis.AGV_rc->rc.ch[1]) + abs(agv_chassis.AGV_rc->rc.ch[0])) * max_speed;
				if (agv_chassis.AGV_rc->rc.ch[1] < 0)
				{
					agv_chassis.set_6020[0] = M6020_m1 - yaw_relative + res - 180 - count11 * 360;
					agv_chassis.set_6020[1] = M6020_m2 - yaw_relative + res - 180 - count22 * 360;
					agv_chassis.set_6020[2] = M6020_m3 - yaw_relative + res - 180 - count33 * 360;
					agv_chassis.set_6020[3] = M6020_m4 - yaw_relative + res - 180 - count44 * 360;
					agv_chassis.set_3508[0] = -(abs(agv_chassis.AGV_rc->rc.ch[1]) + abs(agv_chassis.AGV_rc->rc.ch[0])) * max_speed;
					agv_chassis.set_3508[1] = -(abs(agv_chassis.AGV_rc->rc.ch[1]) + abs(agv_chassis.AGV_rc->rc.ch[0])) * max_speed;
					agv_chassis.set_3508[2] = -(abs(agv_chassis.AGV_rc->rc.ch[1]) + abs(agv_chassis.AGV_rc->rc.ch[0])) * max_speed;
					agv_chassis.set_3508[3] = -(abs(agv_chassis.AGV_rc->rc.ch[1]) + abs(agv_chassis.AGV_rc->rc.ch[0])) * max_speed;
				}
			}
			else if (res < 0)
			{
				agv_chassis.set_6020[0] = M6020_m1 - yaw_relative + res - count11 * 360;
				agv_chassis.set_6020[1] = M6020_m2 - yaw_relative + res - count22 * 360;
				agv_chassis.set_6020[2] = M6020_m3 - yaw_relative + res - count33 * 360;
				agv_chassis.set_6020[3] = M6020_m4 - yaw_relative + res - count44 * 360;
				agv_chassis.set_3508[0] = (abs(agv_chassis.AGV_rc->rc.ch[1]) + abs(agv_chassis.AGV_rc->rc.ch[0])) * max_speed;
				agv_chassis.set_3508[1] = (abs(agv_chassis.AGV_rc->rc.ch[1]) + abs(agv_chassis.AGV_rc->rc.ch[0])) * max_speed;
				agv_chassis.set_3508[2] = (abs(agv_chassis.AGV_rc->rc.ch[1]) + abs(agv_chassis.AGV_rc->rc.ch[0])) * max_speed;
				agv_chassis.set_3508[3] = (abs(agv_chassis.AGV_rc->rc.ch[1]) + abs(agv_chassis.AGV_rc->rc.ch[0])) * max_speed;
				if (res <= -90 && res >= -180)
				{
					agv_chassis.set_6020[0] = M6020_m1 - yaw_relative + res + 180 - count11 * 360; // chassis_yaw_motor->relative_angle是从云台获取的相对角度
					agv_chassis.set_6020[1] = M6020_m2 - yaw_relative + res + 180 - count22 * 360;
					agv_chassis.set_6020[2] = M6020_m3 - yaw_relative + res + 180 - count33 * 360;
					agv_chassis.set_6020[3] = M6020_m4 - yaw_relative + res + 180 - count44 * 360;
					agv_chassis.set_3508[0] = -(abs(agv_chassis.AGV_rc->rc.ch[1]) + abs(agv_chassis.AGV_rc->rc.ch[0])) * max_speed;
					agv_chassis.set_3508[1] = -(abs(agv_chassis.AGV_rc->rc.ch[1]) + abs(agv_chassis.AGV_rc->rc.ch[0])) * max_speed;
					agv_chassis.set_3508[2] = -(abs(agv_chassis.AGV_rc->rc.ch[1]) + abs(agv_chassis.AGV_rc->rc.ch[0])) * max_speed;
					agv_chassis.set_3508[3] = -(abs(agv_chassis.AGV_rc->rc.ch[1]) + abs(agv_chassis.AGV_rc->rc.ch[0])) * max_speed;
				}
			}
			Chassis_Power_Limit(&agv_chassis);
//			CAN_CMD_CHASSIS(agv_chassis.PID_3508[0].out, agv_chassis.PID_3508[1].out, agv_chassis.PID_3508[2].out, agv_chassis.PID_3508[3].out);
			Motor_Set_Current(agv_chassis.PID_6020_speed[0].out, agv_chassis.PID_6020_speed[1].out, agv_chassis.PID_6020_speed[2].out, agv_chassis.PID_6020_speed[3].out);
			count_chassis = yaw_count;
			F_flag = 0;
		}
		else if (agv_chassis.rc_mode == AGV_CHASSIS_RELAX) //无力模式
		{
			agv_chassis.real_vz = 0;
			Chassis_Power_Limit(&agv_chassis);
			CAN_CMD_CHASSIS(0, 0, 0, 0);
			Motor_Set_Current(0, 0, 0, 0);
		}
		//小陀螺运动
		//未完全完成，小陀螺相关移动为实现（工程以移植陀螺仪，调用即可）
		//已完成全部小陀螺运动，未测试
		else if (agv_chassis.rc_mode == AGV_CHASSIS_DODGE_MODE)
		{
			F_flag = 1;
			//小陀螺相关解算的实现（未测试）
			agv_chassis.real_vz = -660;
			agv_chassis.RC_X_ChassisSpeedRef = agv_chassis.AGV_rc->rc.ch[1];
			agv_chassis.RC_Y_ChassisSpeedRef = agv_chassis.AGV_rc->rc.ch[0];
			sin_yaw = arm_sin_f32((agv_chassis.chassis_yaw_motor->relative_angle) * ANGLE_TO_RAD); // arm_sin_f32不能直接用角度值，要将其转化为弧度值
			cos_yaw = arm_cos_f32((agv_chassis.chassis_yaw_motor->relative_angle) * ANGLE_TO_RAD);
			agv_chassis.real_vx = (cos_yaw * agv_chassis.RC_X_ChassisSpeedRef + sin_yaw * agv_chassis.RC_Y_ChassisSpeedRef);
			agv_chassis.real_vy = (-sin_yaw * agv_chassis.RC_X_ChassisSpeedRef + cos_yaw * agv_chassis.RC_Y_ChassisSpeedRef);
			Steering_wheel(agv_chassis.real_vx, agv_chassis.real_vy, agv_chassis.real_vz);
			Chassis_Power_Limit(&agv_chassis);
//			CAN_CMD_CHASSIS(agv_chassis.PID_3508[0].out, agv_chassis.PID_3508[1].out, agv_chassis.PID_3508[2].out, agv_chassis.PID_3508[3].out);
			Motor_Set_Current(agv_chassis.PID_6020_speed[0].out, agv_chassis.PID_6020_speed[1].out, agv_chassis.PID_6020_speed[2].out, agv_chassis.PID_6020_speed[3].out);
		}
		else if (agv_chassis.rc_mode == CHASSIS_FOLLOW_GIMBAL) //传统底盘跟随模式
		{
			float FOLLOW_GIMBAL_Z; //左右旋转
			fp32 Rotation_rate = 0.0f;
			int chassis_relative_angle_set = 0.0f;
			agv_chassis.RC_X_ChassisSpeedRef = agv_chassis.AGV_rc->rc.ch[1];
			agv_chassis.RC_Y_ChassisSpeedRef = agv_chassis.AGV_rc->rc.ch[0];
			sin_yaw = arm_sin_f32(agv_chassis.chassis_yaw_motor->relative_angle * ANGLE_TO_RAD); // arm_sin_f32不能直接用角度值，要将其转化为弧度值
			cos_yaw = arm_cos_f32(agv_chassis.chassis_yaw_motor->relative_angle * ANGLE_TO_RAD);
			agv_chassis.real_vx = (cos_yaw * agv_chassis.RC_X_ChassisSpeedRef + sin_yaw * agv_chassis.RC_Y_ChassisSpeedRef);
			agv_chassis.real_vy = (-sin_yaw * agv_chassis.RC_X_ChassisSpeedRef + cos_yaw * agv_chassis.RC_Y_ChassisSpeedRef);
			FOLLOW_GIMBAL_Z = PID_Calc(&agv_chassis.Righting, agv_chassis.chassis_yaw_motor->relative_angle, chassis_relative_angle_set);
			if (fabs(FOLLOW_GIMBAL_Z) > 160.0f) // 210
			{
				Rotation_rate = ((8000.0f - fabs(FOLLOW_GIMBAL_Z) - 4500.0f) / 8000.0f) * ((8000.0f - fabs(FOLLOW_GIMBAL_Z) - 4500.0f) / 8000.0f);
			}
			else
			{
				Rotation_rate = 1.0f;
			}

			agv_chassis.real_vx = /*Rotation_rate */ fp32_constrain(agv_chassis.real_vx, -8000, 8000);
			agv_chassis.real_vy = /*Rotation_rate */ fp32_constrain(agv_chassis.real_vy, -8000, 8000);
			agv_chassis.real_vz = -fp32_constrain(FOLLOW_GIMBAL_Z, -8000, 8000);

			Steering_wheel(agv_chassis.real_vx, agv_chassis.real_vy, agv_chassis.real_vz);
			Chassis_Power_Limit(&agv_chassis);
//			CAN_CMD_CHASSIS(agv_chassis.PID_3508[0].out, agv_chassis.PID_3508[1].out, agv_chassis.PID_3508[2].out, agv_chassis.PID_3508[3].out);
			Motor_Set_Current(agv_chassis.PID_6020_speed[0].out, agv_chassis.PID_6020_speed[1].out, agv_chassis.PID_6020_speed[2].out, agv_chassis.PID_6020_speed[3].out);
		}
		//键盘控制.主要是WASD控制，控制底盘的全向移动
		else if (agv_chassis.rc_mode == AGV_CHASSIS_KEY_MODE)
		{
			KEY_UPDATED();
			if (FFlag_state == 0)
			{
				agv_chassis.change1 = yaw_count * 360;
				agv_chassis.change2 = yaw_count * 360;
				agv_chassis.change3 = yaw_count * 360;
				agv_chassis.change4 = yaw_count * 360;
				if (W_Flag == 1 && (A_Flag + D_Flag) == 0)
				{
					agv_chassis.set_6020[0] = M6020_m1 - yaw_relative - count11 * 360;
					agv_chassis.set_6020[1] = M6020_m2 - yaw_relative - count22 * 360;
					agv_chassis.set_6020[2] = M6020_m3 - yaw_relative - count33 * 360;
					agv_chassis.set_6020[3] = M6020_m4 - yaw_relative - count44 * 360;
					agv_chassis.set_3508[0] = W_Flag * RampCalc(&chassis_WRamp)* 660 * max_speed;
					agv_chassis.set_3508[1] = W_Flag * RampCalc(&chassis_WRamp)* 660 * max_speed;
					agv_chassis.set_3508[2] = W_Flag * RampCalc(&chassis_WRamp)* 660 * max_speed;
					agv_chassis.set_3508[3] = W_Flag * RampCalc(&chassis_WRamp)* 660 * max_speed;
					change_Chassis_tan = 0;
					F_flag = 0;
				}
				else if (S_Flag == 1 && (A_Flag + D_Flag) == 0)
				{
					agv_chassis.set_6020[0] = M6020_m1 - yaw_relative - count11 * 360;
					agv_chassis.set_6020[1] = M6020_m2 - yaw_relative - count22 * 360;
					agv_chassis.set_6020[2] = M6020_m3 - yaw_relative - count33 * 360;
					agv_chassis.set_6020[3] = M6020_m4 - yaw_relative - count44 * 360;
					agv_chassis.set_3508[0] = -S_Flag * RampCalc(&chassis_SRamp) *660 * max_speed;
					agv_chassis.set_3508[1] = -S_Flag * RampCalc(&chassis_SRamp) *660 * max_speed;
					agv_chassis.set_3508[2] = -S_Flag * RampCalc(&chassis_SRamp) *660 * max_speed;
					agv_chassis.set_3508[3] = -S_Flag * RampCalc(&chassis_SRamp) *660 * max_speed;
					change_Chassis_tan = 0;
					F_flag = 0;
				}
				else if (A_Flag == 1 && (W_Flag + S_Flag) == 0)
				{
					agv_chassis.set_6020[0] = M6020_m1 + atan2(1, 0) * 180.f / 3.14159 - yaw_relative - count11 * 360;
					agv_chassis.set_6020[1] = M6020_m2 + atan2(1, 0) * 180.f / 3.14159 - yaw_relative - count22 * 360;
					agv_chassis.set_6020[2] = M6020_m3 + atan2(1, 0) * 180.f / 3.14159 - yaw_relative - count33 * 360;
					agv_chassis.set_6020[3] = M6020_m4 + atan2(1, 0) * 180.f / 3.14159 - yaw_relative - count44 * 360;
					agv_chassis.set_3508[0] = -A_Flag * RampCalc(&chassis_ARamp)*660 * max_speed;
					agv_chassis.set_3508[1] = -A_Flag * RampCalc(&chassis_ARamp)*660 * max_speed;
					agv_chassis.set_3508[2] = -A_Flag * RampCalc(&chassis_ARamp)*660 * max_speed;
					agv_chassis.set_3508[3] = -A_Flag * RampCalc(&chassis_ARamp)*660 * max_speed;
					change_Chassis_tan = atan2(1, 0) * 180.f / 3.14159;
					F_flag = 0;
				}
				else if (D_Flag == 1 && (W_Flag + S_Flag) == 0)
				{
					agv_chassis.set_6020[0] = M6020_m1 + atan2(1, 0) * 180.f / 3.14159 - yaw_relative - count11 * 360;
					agv_chassis.set_6020[1] = M6020_m2 + atan2(1, 0) * 180.f / 3.14159 - yaw_relative - count22 * 360;
					agv_chassis.set_6020[2] = M6020_m3 + atan2(1, 0) * 180.f / 3.14159 - yaw_relative - count33 * 360;
					agv_chassis.set_6020[3] = M6020_m4 + atan2(1, 0) * 180.f / 3.14159 - yaw_relative - count44 * 360;
					agv_chassis.set_3508[0] = D_Flag * RampCalc(&chassis_DRamp) * 660 * max_speed;
					agv_chassis.set_3508[1] = D_Flag * RampCalc(&chassis_DRamp) * 660 * max_speed;
					agv_chassis.set_3508[2] = D_Flag * RampCalc(&chassis_DRamp) * 660 * max_speed;
					agv_chassis.set_3508[3] = D_Flag * RampCalc(&chassis_DRamp) * 660 * max_speed;
					change_Chassis_tan = atan2(1, 0) * 180.f / 3.14159;
					F_flag = 0;
				}
				else if (W_Flag == 1 && A_Flag == 1)
				{
					agv_chassis.set_6020[0] = M6020_m1 - atan2(1, 1) * 180.f / 3.14159 - yaw_relative - count11 * 360;
					agv_chassis.set_6020[1] = M6020_m2 - atan2(1, 1) * 180.f / 3.14159 - yaw_relative - count22 * 360;
					agv_chassis.set_6020[2] = M6020_m3 - atan2(1, 1) * 180.f / 3.14159 - yaw_relative - count33 * 360;
					agv_chassis.set_6020[3] = M6020_m4 - atan2(1, 1) * 180.f / 3.14159 - yaw_relative - count44 * 360;
					agv_chassis.set_3508[0] = W_Flag * RampCalc(&chassis_WRamp) * 660 * max_speed;
					agv_chassis.set_3508[1] = W_Flag * RampCalc(&chassis_WRamp) * 660 * max_speed;
					agv_chassis.set_3508[2] = W_Flag * RampCalc(&chassis_WRamp) * 660 * max_speed;
					agv_chassis.set_3508[3] = W_Flag * RampCalc(&chassis_WRamp) * 660 * max_speed;
					change_Chassis_tan = atan2(1, 1) * 180.f / 3.14159;
					F_flag = 0;
				}
				else if (W_Flag == 1 && D_Flag == 1)
				{
					agv_chassis.set_6020[0] = M6020_m1 + atan2(1, 1) * 180.f / 3.14159 - yaw_relative - count11 * 360;
					agv_chassis.set_6020[1] = M6020_m2 + atan2(1, 1) * 180.f / 3.14159 - yaw_relative - count22 * 360;
					agv_chassis.set_6020[2] = M6020_m3 + atan2(1, 1) * 180.f / 3.14159 - yaw_relative - count33 * 360;
					agv_chassis.set_6020[3] = M6020_m4 + atan2(1, 1) * 180.f / 3.14159 - yaw_relative - count44 * 360;
					agv_chassis.set_3508[0] = W_Flag * RampCalc(&chassis_WRamp) * 660 * max_speed;
					agv_chassis.set_3508[1] = W_Flag * RampCalc(&chassis_WRamp) * 660 * max_speed;
					agv_chassis.set_3508[2] = W_Flag * RampCalc(&chassis_WRamp) * 660 * max_speed;
					agv_chassis.set_3508[3] = W_Flag * RampCalc(&chassis_WRamp) * 660 * max_speed;
					change_Chassis_tan = atan2(1, 1) * 180.f / 3.14159;
					F_flag = 0;
				}
				else if (S_Flag == 1 && A_Flag == 1)
				{
					agv_chassis.set_6020[0] = M6020_m1 + atan2(1, 1) * 180.f / 3.14159 - yaw_relative - count11 * 360;
					agv_chassis.set_6020[1] = M6020_m2 + atan2(1, 1) * 180.f / 3.14159 - yaw_relative - count22 * 360;
					agv_chassis.set_6020[2] = M6020_m3 + atan2(1, 1) * 180.f / 3.14159 - yaw_relative - count33 * 360;
					agv_chassis.set_6020[3] = M6020_m4 + atan2(1, 1) * 180.f / 3.14159 - yaw_relative - count44 * 360;
					agv_chassis.set_3508[0] = -S_Flag * RampCalc(&chassis_SRamp) * 660 * max_speed;
					agv_chassis.set_3508[1] = -S_Flag * RampCalc(&chassis_SRamp) * 660 * max_speed;
					agv_chassis.set_3508[2] = -S_Flag * RampCalc(&chassis_SRamp) * 660 * max_speed;
					agv_chassis.set_3508[3] = -S_Flag * RampCalc(&chassis_SRamp) * 660 * max_speed;
					change_Chassis_tan = atan2(1, 1) * 180.f / 3.14159;
					F_flag = 0;
				}
				else if (S_Flag == 1 && D_Flag == 1)
				{
					agv_chassis.set_6020[0] = M6020_m1 - atan2(1, 1) * 180.f / 3.14159 - yaw_relative - count11 * 360;
					agv_chassis.set_6020[1] = M6020_m2 - atan2(1, 1) * 180.f / 3.14159 - yaw_relative - count22 * 360;
					agv_chassis.set_6020[2] = M6020_m3 - atan2(1, 1) * 180.f / 3.14159 - yaw_relative - count33 * 360;
					agv_chassis.set_6020[3] = M6020_m4 - atan2(1, 1) * 180.f / 3.14159 - yaw_relative - count44 * 360;
					agv_chassis.set_3508[0] = -S_Flag * RampCalc(&chassis_SRamp) * 660 * max_speed;
					agv_chassis.set_3508[1] = -S_Flag * RampCalc(&chassis_SRamp) * 660 * max_speed;
					agv_chassis.set_3508[2] = -S_Flag * RampCalc(&chassis_SRamp) * 660 * max_speed;
					agv_chassis.set_3508[3] = -S_Flag * RampCalc(&chassis_SRamp) * 660 * max_speed;
					change_Chassis_tan = atan2(1, 1) * 180.f / 3.14159;
					F_flag = 0;
				}
				// if (G_Flag == 1) //射符模式下底盘不动
				// {
				// 	agv_chassis.set_3508[0] = 0;
				// 	agv_chassis.set_3508[1] = 0;
				// 	agv_chassis.set_3508[2] = 0;
				// 	agv_chassis.set_3508[3] = 0;
				// }
				else
				{
					agv_chassis.set_6020[0] = M6020_m1 - yaw_relative - count11 * 360+change_Chassis_tan;
					agv_chassis.set_6020[1] = M6020_m2 - yaw_relative - count22 * 360+change_Chassis_tan;
					agv_chassis.set_6020[2] = M6020_m3 - yaw_relative - count33 * 360+change_Chassis_tan;
					agv_chassis.set_6020[3] = M6020_m4 - yaw_relative - count44 * 360+change_Chassis_tan;
					agv_chassis.set_3508[0] = 0;
					agv_chassis.set_3508[1] = 0;
					agv_chassis.set_3508[2] = 0;
					agv_chassis.set_3508[3] = 0;
					F_flag = 0;
				}
			}
			//因为基础运动与小陀螺所用解算方式不同，所以在此做出判断
			while (FFlag_state == 0x01) //按一次f开启小陀螺，再按下一次小陀螺关闭
			{
				KEY_UPDATED();
				motor_speed_change();
				Super_power_ctrl(&agv_chassis);
				agv_chassis.rc_mode = AGV_CHASSIS_DODGE_MODE;
				if (W_Flag == 1 && (A_Flag + D_Flag) == 0)
				{
					agv_chassis.RC_X_ChassisSpeedRef = 660*DODGE_NUM;
					agv_chassis.RC_Y_ChassisSpeedRef = 0;
				}
				else if (S_Flag == 1 && (A_Flag + D_Flag) == 0)
				{
					agv_chassis.RC_X_ChassisSpeedRef = -660*DODGE_NUM;
					agv_chassis.RC_Y_ChassisSpeedRef = 0;
				}
				else if (A_Flag == 1 && (W_Flag + S_Flag) == 0)
				{
					agv_chassis.RC_Y_ChassisSpeedRef = -660*DODGE_NUM;
					agv_chassis.RC_X_ChassisSpeedRef = 0;
				}
				else if (D_Flag == 1 && (W_Flag + S_Flag) == 0)
				{
					agv_chassis.RC_Y_ChassisSpeedRef = 660*DODGE_NUM;
					agv_chassis.RC_X_ChassisSpeedRef = 0;
				}
				else if (W_Flag == 1 && A_Flag == 1)
				{
					agv_chassis.RC_Y_ChassisSpeedRef = -660;
					agv_chassis.RC_X_ChassisSpeedRef = 660;
				}
				else if (W_Flag == 1 && D_Flag == 1)
				{
					agv_chassis.RC_Y_ChassisSpeedRef = 660;
					agv_chassis.RC_X_ChassisSpeedRef = 660;
				}
				else if (S_Flag == 1 && A_Flag == 1)
				{
					agv_chassis.RC_Y_ChassisSpeedRef = -660;
					agv_chassis.RC_X_ChassisSpeedRef = -660;
				}
				else if (S_Flag == 1 && D_Flag == 1)
				{
					agv_chassis.RC_Y_ChassisSpeedRef = 660;
					agv_chassis.RC_X_ChassisSpeedRef = -660;
				}
				else
				{
					agv_chassis.RC_Y_ChassisSpeedRef = 0;
					agv_chassis.RC_X_ChassisSpeedRef = 0;
				}
				agv_chassis.real_vz = 660;
				sin_yaw = arm_sin_f32(agv_chassis.chassis_yaw_motor->relative_angle * ANGLE_TO_RAD); // arm_sin_f32不能直接用角度值，要将其转化为弧度值
				cos_yaw = arm_cos_f32(agv_chassis.chassis_yaw_motor->relative_angle * ANGLE_TO_RAD);
				agv_chassis.real_vx = (cos_yaw * agv_chassis.RC_X_ChassisSpeedRef + sin_yaw * agv_chassis.RC_Y_ChassisSpeedRef);
				agv_chassis.real_vy = (-sin_yaw * agv_chassis.RC_X_ChassisSpeedRef + cos_yaw * agv_chassis.RC_Y_ChassisSpeedRef);

				Steering_wheel(agv_chassis.real_vx, agv_chassis.real_vy, agv_chassis.real_vz);

				// 3508电机pid控制
				PID_Calc(&agv_chassis.PID_3508[0], agv_chassis.motor_chassis_3508[0].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[0]);
				PID_Calc(&agv_chassis.PID_3508[1], agv_chassis.motor_chassis_3508[1].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[1]);
				PID_Calc(&agv_chassis.PID_3508[2], agv_chassis.motor_chassis_3508[2].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[2]);
				PID_Calc(&agv_chassis.PID_3508[3], agv_chassis.motor_chassis_3508[3].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[3]);
				// 6020电机pid双环控制
				PID_Calc(&agv_chassis.PID_6020_angle[0], agv_chassis.motor_chassis_6020[0].chassis_motor_measure->all_ecd / ECD, agv_chassis.set_6020[0]);
				PID_Calc(&agv_chassis.PID_6020_angle[1], agv_chassis.motor_chassis_6020[1].chassis_motor_measure->all_ecd / ECD, agv_chassis.set_6020[1]);
				PID_Calc(&agv_chassis.PID_6020_angle[2], agv_chassis.motor_chassis_6020[2].chassis_motor_measure->all_ecd / ECD, agv_chassis.set_6020[2]);
				PID_Calc(&agv_chassis.PID_6020_angle[3], agv_chassis.motor_chassis_6020[3].chassis_motor_measure->all_ecd / ECD, agv_chassis.set_6020[3]);
				PID_Calc(&agv_chassis.PID_6020_speed[0], agv_chassis.motor_chassis_6020[0].chassis_motor_measure->speed_rpm, agv_chassis.PID_6020_angle[0].out);
				PID_Calc(&agv_chassis.PID_6020_speed[1], agv_chassis.motor_chassis_6020[1].chassis_motor_measure->speed_rpm, agv_chassis.PID_6020_angle[1].out);
				PID_Calc(&agv_chassis.PID_6020_speed[2], agv_chassis.motor_chassis_6020[2].chassis_motor_measure->speed_rpm, agv_chassis.PID_6020_angle[2].out);
				PID_Calc(&agv_chassis.PID_6020_speed[3], agv_chassis.motor_chassis_6020[3].chassis_motor_measure->speed_rpm, agv_chassis.PID_6020_angle[3].out);

				Chassis_Power_Limit(&agv_chassis);
				CAN_CMD_CHASSIS(agv_chassis.PID_3508[0].out, agv_chassis.PID_3508[1].out, agv_chassis.PID_3508[2].out, agv_chassis.PID_3508[3].out);
				Motor_Set_Current(agv_chassis.PID_6020_speed[0].out, agv_chassis.PID_6020_speed[1].out, agv_chassis.PID_6020_speed[2].out, agv_chassis.PID_6020_speed[3].out);
				F_flag = 1;
				vTaskDelay(1);
			}
			while (CTRLFlag_state == 1)
			{

				KEY_UPDATED();
				motor_speed_change();
				agv_chassis.rc_mode = CHASSIS_FOLLOW_GIMBAL;
				//		Super_power_ctrl(&agv_chassis);
				//获取键盘数据
				if (W_Flag == 1 && (A_Flag + D_Flag) == 0)
				{
					agv_chassis.RC_X_ChassisSpeedRef = 660;
					agv_chassis.RC_Y_ChassisSpeedRef = 0;
				}
				else if (S_Flag == 1 && (A_Flag + D_Flag) == 0)
				{
					agv_chassis.RC_X_ChassisSpeedRef = -660;
					agv_chassis.RC_Y_ChassisSpeedRef = 0;
				}
				else if (A_Flag == 1 && (W_Flag + S_Flag) == 0)
				{
					agv_chassis.RC_Y_ChassisSpeedRef = -660;
					agv_chassis.RC_X_ChassisSpeedRef = 0;
				}
				else if (D_Flag == 1 && (W_Flag + S_Flag) == 0)
				{
					agv_chassis.RC_Y_ChassisSpeedRef = 660;
					agv_chassis.RC_X_ChassisSpeedRef = 0;
				}
				else if (W_Flag == 1 && A_Flag == 1)
				{
					agv_chassis.RC_Y_ChassisSpeedRef = -660;
					agv_chassis.RC_X_ChassisSpeedRef = 660;
				}
				else if (W_Flag == 1 && D_Flag == 1)
				{
					agv_chassis.RC_Y_ChassisSpeedRef = 660;
					agv_chassis.RC_X_ChassisSpeedRef = 660;
				}
				else if (S_Flag == 1 && A_Flag == 1)
				{
					agv_chassis.RC_Y_ChassisSpeedRef = -660;
					agv_chassis.RC_X_ChassisSpeedRef = -660;
				}
				else if (S_Flag == 1 && D_Flag == 1)
				{
					agv_chassis.RC_Y_ChassisSpeedRef = 660;
					agv_chassis.RC_X_ChassisSpeedRef = -660;
				}
				else
				{
					agv_chassis.RC_Y_ChassisSpeedRef = 0;
					agv_chassis.RC_X_ChassisSpeedRef = 0;
				}
				float FOLLOW_GIMBAL_Z; //计算得出的旋转速度
				int chassis_relative_angle_set = 0.0f;
				sin_yaw = arm_sin_f32(agv_chassis.chassis_yaw_motor->relative_angle * ANGLE_TO_RAD);
				cos_yaw = arm_cos_f32(agv_chassis.chassis_yaw_motor->relative_angle * ANGLE_TO_RAD);
				agv_chassis.real_vx = (cos_yaw * agv_chassis.RC_X_ChassisSpeedRef + sin_yaw * agv_chassis.RC_Y_ChassisSpeedRef);
				agv_chassis.real_vy = (-sin_yaw * agv_chassis.RC_X_ChassisSpeedRef + cos_yaw * agv_chassis.RC_Y_ChassisSpeedRef);
				FOLLOW_GIMBAL_Z = PID_Calc(&agv_chassis.Righting, agv_chassis.chassis_yaw_motor->relative_angle, chassis_relative_angle_set);
				agv_chassis.real_vz = -fp32_constrain(FOLLOW_GIMBAL_Z, -8000, 8000);
				agv_chassis.real_vx = fp32_constrain(agv_chassis.real_vx, -8900, 8900);
				agv_chassis.real_vy = fp32_constrain(agv_chassis.real_vy, -8900, 8900);
				Steering_wheel(agv_chassis.real_vx, agv_chassis.real_vy, agv_chassis.real_vz);

				// 3508电机pid控制
				PID_Calc(&agv_chassis.PID_3508[0], agv_chassis.motor_chassis_3508[0].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[0]);
				PID_Calc(&agv_chassis.PID_3508[1], agv_chassis.motor_chassis_3508[1].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[1]);
				PID_Calc(&agv_chassis.PID_3508[2], agv_chassis.motor_chassis_3508[2].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[2]);
				PID_Calc(&agv_chassis.PID_3508[3], agv_chassis.motor_chassis_3508[3].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[3]);
				// 6020电机pid双环控制
				PID_Calc(&agv_chassis.PID_6020_angle[0], agv_chassis.motor_chassis_6020[0].chassis_motor_measure->all_ecd / ECD, agv_chassis.set_6020[0]);
				PID_Calc(&agv_chassis.PID_6020_angle[1], agv_chassis.motor_chassis_6020[1].chassis_motor_measure->all_ecd / ECD, agv_chassis.set_6020[1]);
				PID_Calc(&agv_chassis.PID_6020_angle[2], agv_chassis.motor_chassis_6020[2].chassis_motor_measure->all_ecd / ECD, agv_chassis.set_6020[2]);
				PID_Calc(&agv_chassis.PID_6020_angle[3], agv_chassis.motor_chassis_6020[3].chassis_motor_measure->all_ecd / ECD, agv_chassis.set_6020[3]);
				PID_Calc(&agv_chassis.PID_6020_speed[0], agv_chassis.motor_chassis_6020[0].chassis_motor_measure->speed_rpm, agv_chassis.PID_6020_angle[0].out);
				PID_Calc(&agv_chassis.PID_6020_speed[1], agv_chassis.motor_chassis_6020[1].chassis_motor_measure->speed_rpm, agv_chassis.PID_6020_angle[1].out);
				PID_Calc(&agv_chassis.PID_6020_speed[2], agv_chassis.motor_chassis_6020[2].chassis_motor_measure->speed_rpm, agv_chassis.PID_6020_angle[2].out);
				PID_Calc(&agv_chassis.PID_6020_speed[3], agv_chassis.motor_chassis_6020[3].chassis_motor_measure->speed_rpm, agv_chassis.PID_6020_angle[3].out);

				Chassis_Power_Limit(&agv_chassis);
				CAN_CMD_CHASSIS(agv_chassis.PID_3508[0].out, agv_chassis.PID_3508[1].out, agv_chassis.PID_3508[2].out, agv_chassis.PID_3508[3].out);
				Motor_Set_Current(agv_chassis.PID_6020_speed[0].out, agv_chassis.PID_6020_speed[1].out, agv_chassis.PID_6020_speed[2].out, agv_chassis.PID_6020_speed[3].out);
				vTaskDelay(1);
			}
			Chassis_Power_Limit(&agv_chassis);
			CAN_CMD_CHASSIS(agv_chassis.PID_3508[0].out, agv_chassis.PID_3508[1].out, agv_chassis.PID_3508[2].out, agv_chassis.PID_3508[3].out);
			Motor_Set_Current(agv_chassis.PID_6020_speed[0].out, agv_chassis.PID_6020_speed[1].out, agv_chassis.PID_6020_speed[2].out, agv_chassis.PID_6020_speed[3].out);
		}
		vTaskDelay(1);
	}
}
/*键盘数据更新
  WASD前后左右
*/
void KEY_UPDATED(void)
{
	if (agv_chassis.AGV_rc->key.v & KEY_PRESSED_OFFSET_W)
	{
		W_Flag = 1;
	}
	if (!(agv_chassis.AGV_rc->key.v & KEY_PRESSED_OFFSET_W) != 0)
	{
		W_Flag = 0;
		RampResetCounter(&chassis_WRamp);
	}
	if ((!(agv_chassis.AGV_rc->key.v & KEY_PRESSED_OFFSET_SHIFT)) != 0)
	{
		Shift_Flag = 2;
	}

	//按下 Shift 使用超级电容
	if ((agv_chassis.AGV_rc->key.v & KEY_PRESSED_OFFSET_SHIFT) && (Super_power.volt > 12000))
	{
		Shift_Flag = 1;
	}
	else
	{
		Shift_Flag = 2;
	}
	if (agv_chassis.AGV_rc->key.v & KEY_PRESSED_OFFSET_S)
	{
		S_Flag = 1;
	}
	if (!(agv_chassis.AGV_rc->key.v & KEY_PRESSED_OFFSET_S) != 0)
	{
		S_Flag = 0;
		RampResetCounter(&chassis_SRamp);
	}
	if (agv_chassis.AGV_rc->key.v & KEY_PRESSED_OFFSET_A)
	{
		A_Flag = 1;
	}
	if (!(agv_chassis.AGV_rc->key.v & KEY_PRESSED_OFFSET_A) != 0)
	{
		A_Flag = 0;
		RampResetCounter(&chassis_ARamp);
	}
	if (agv_chassis.AGV_rc->key.v & KEY_PRESSED_OFFSET_D)
	{
		D_Flag = 1;
	}
	if (!(agv_chassis.AGV_rc->key.v & KEY_PRESSED_OFFSET_D) != 0)
	{
		D_Flag = 0;
		RampResetCounter(&chassis_DRamp);
	}
	if (!(agv_chassis.AGV_rc->key.v & KEY_PRESSED_OFFSET_F) != 0)
	{
		F_Flag = 1;
	}
	if (agv_chassis.AGV_rc->key.v & KEY_PRESSED_OFFSET_F && F_Flag == 1)
	{
		F_Flag = 0;
		FFlag_state++;
		FFlag_state %= 2;
	}
	if (!(agv_chassis.AGV_rc->key.v & KEY_PRESSED_OFFSET_CTRL) != 0)
	{
		Ctrl_Flag = 1;
	}
	if (agv_chassis.AGV_rc->key.v & KEY_PRESSED_OFFSET_CTRL && Ctrl_Flag == 1)
	{
		Ctrl_Flag = 0;
		CTRLFlag_state++;
		CTRLFlag_state %= 2;
	}
	RampSetScale(&chassis_WRamp, 600);
	RampSetScale(&chassis_ARamp, 600);
	RampSetScale(&chassis_SRamp, 2000);
	RampSetScale(&chassis_DRamp, 2000);
}

void chassis_Init(AGV_chassis_t *agv_chassis) //底盘初始化函数
{
	RampInit(&chassis_WRamp, 0);
	RampInit(&chassis_ARamp, 0);
	RampInit(&chassis_SRamp, 0);
	RampInit(&chassis_DRamp, 0);

	agv_chassis->chassis_yaw_motor = get_yaw_motor_point();		  //获取云台yaw电机数据
	agv_chassis->chassis_pitch_motor = get_pitch_motor_point();	  //获取云台pitch电机
	agv_chassis->chassis_power_measure = get_power_heat_data_t(); //获取底盘功率
	//获取检测系统数据指针
	agv_chassis->chassis_monitor_point = getErrorListPoint();
	agv_chassis->chassis_status_measure = get_game_robot_state_t();
	agv_chassis->chassis_hurt_type = get_robot_hurt_t();
	for (int i = 0; i < 4; i++) //获取3508
	{
		agv_chassis->motor_chassis_3508[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
		agv_chassis->motor_chassis_3508[i].chassis_encoder_measure = get_Chassis_Encoder_Measure_Point(i);
	}

	for (int i = 0; i < 4; i++) //获取6020电机数据
	{
		agv_chassis->motor_chassis_6020[i].chassis_motor_measure = get_Chassis_6020_Motor_Measure_Point(i);
		agv_chassis->motor_chassis_6020[i].chassis_encoder_measure = get_Chassis_6020_Encoder_Measure_Point(i);
	}
	//获取遥控器指针
	agv_chassis->AGV_rc = get_remote_control_point();
	//进行电机期望的初始化
	agv_chassis->set_3508[0] = 0;
	agv_chassis->set_3508[1] = 0;
	agv_chassis->set_3508[2] = 0;
	agv_chassis->set_3508[3] = 0;
	agv_chassis->set_6020[0] = M6020_m1;
	agv_chassis->set_6020[1] = M6020_m2;
	agv_chassis->set_6020[2] = M6020_m3;
	agv_chassis->set_6020[3] = M6020_m4;
	//========================3508pid初始化=========================================================
	PID_Init(&agv_chassis->PID_3508[0], PID_POSITION, M3508_speed_pid, M3508_SPEED_OUT_MAX, M3508_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_3508[1], PID_POSITION, M3508_speed_pid, M3508_SPEED_OUT_MAX, M3508_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_3508[2], PID_POSITION, M3508_speed_pid, M3508_SPEED_OUT_MAX, M3508_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_3508[3], PID_POSITION, M3508_speed_pid, M3508_SPEED_OUT_MAX, M3508_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	//========================6020pid初始化================================================================
	PID_Init(&agv_chassis->PID_6020_speed[0], PID_POSITION, M6020_speed_pid1, M6020_SPEED_OUT_MAX, M6020_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_6020_speed[1], PID_POSITION, M6020_speed_pid2, M6020_SPEED_OUT_MAX, M6020_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_6020_speed[2], PID_POSITION, M6020_speed_pid3, M6020_SPEED_OUT_MAX, M6020_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_6020_speed[3], PID_POSITION, M6020_speed_pid4, M6020_SPEED_OUT_MAX, M6020_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	//========================6020串级pid==================================================================
	PID_Init(&agv_chassis->PID_6020_angle[0], PID_POSITION, M6020_angle_pid1, M6020_SPEED_OUT_MAX, M6020_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_6020_angle[1], PID_POSITION, M6020_angle_pid2, M6020_SPEED_OUT_MAX, M6020_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_6020_angle[2], PID_POSITION, M6020_angle_pid3, M6020_SPEED_OUT_MAX, M6020_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_6020_angle[3], PID_POSITION, M6020_angle_pid4, M6020_SPEED_OUT_MAX, M6020_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);

	//底盘整体跟随云台使用
	PID_Init(&agv_chassis->Righting, PID_POSITION, chassis_follow_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, 0, 0, 0, 0, 0);
}
//舵轮运动解算
//主要应用与小陀螺的时的姿态解算，其余的时候不调用此函数
int vx1, vx2, vx3, vx4, vy1, vy2, vy3, vy4, w1, w2, w3, w4; //小陀螺相关解算
int v1, v2, v3, v4;
float tan1 = 0, tan2 = 0, tan3 = 0, tan4 = 0;
float tan11 = 0, tan22 = 0, tan33 = 0, tan44 = 0;
void Steering_wheel(int vx, int vy, int vz)
{
	vx1 = vx * max_speed; //获取通道值，或通过其他方式对x方向进行速度赋值（此处建议用通道值）
	vx2 = vx * max_speed;
	vx3 = vx * max_speed;
	vx4 = vx * max_speed;

	vy1 = vy * max_speed; //获取y轴速度
	vy2 = vy * max_speed;
	vy3 = vy * max_speed;
	vy4 = vy * max_speed;

	w1 = vz * max_speed; //获取z轴速度
	w2 = vz * max_speed;
	w3 = vz * max_speed;
	w4 = vz * max_speed;

	//对3508的速度进项解算--因为解算的值全都是正值，从而对角度进行判断
	//将cos和sin转换成0。707，防止因为数值过大导致数据溢出（在数据转换后没有测试）。
	v1 = sqrt((vy2 - w2 * 0.707) * (vy2 - w2 * 0.707) + (vx2 + w2 * 0.707) * (vx2 + w2 * 0.707));
	v2 = sqrt((vy1 - w1 * 0.707) * (vy1 - w1 * 0.707) + (vx1 - w1 * 0.707) * (vx1 - w1 * 0.707));
	v3 = sqrt((vy4 + w4 * 0.707) * (vy4 + w4 * 0.707) + (vx4 - w4 * 0.707) * (vx4 - w4 * 0.707));
	v4 = sqrt((vy3 + w3 * 0.707) * (vy3 + w3 * 0.707) + (vx3 + w3 * 0.707) * (vx3 + w3 * 0.707));

	agv_chassis.set_3508[0] = v1;
	agv_chassis.set_3508[1] = v2;
	agv_chassis.set_3508[2] = v3;
	agv_chassis.set_3508[3] = v4;

	tan1 = atan2((vy2 - w2 * 0.707), (vx2 + w2 * 0.707)) * 57.2957805;
	tan2 = atan2((vy1 - w1 * 0.707), (vx1 - w1 * 0.707)) * 57.2957805;
	tan3 = atan2((vy4 + w4 * 0.707), (vx4 - w4 * 0.707)) * 57.2957805;
	tan4 = atan2((vy3 + w3 * 0.707), (vx3 + w3 * 0.707)) * 57.2957805;
	if (tan1 - tan11 > 110)
	{
		count11++;
	}
	else if (tan1 - tan11 < -110)
	{
		count11--;
	}
	if (tan2 - tan22 > 110)
	{
		count22++;
	}
	else if (tan2 - tan22 < -110)
	{
		count22--;
	}
	if (tan3 - tan33 > 110)
	{
		count33++;
	}
	else if (tan3 - tan33 < -110)
	{
		count33--;
	}
	if (tan4 - tan44 > 110)
	{
		count44++;
	}
	else if (tan4 - tan44 < -110)
	{
		count44--;
	}
	agv_chassis.set_6020[0] = M6020_m1 + tan1 - count11 * 360 - agv_chassis.change1;
	agv_chassis.set_6020[1] = M6020_m2 + tan2 - count22 * 360 - agv_chassis.change2;
	agv_chassis.set_6020[2] = M6020_m3 + tan3 - count33 * 360 - agv_chassis.change3;
	agv_chassis.set_6020[3] = M6020_m4 + tan4 - count44 * 360 - agv_chassis.change4;
	tan11 = tan1;
	tan22 = tan2;
	tan33 = tan3;
	tan44 = tan4;
}

//底盘功率限制（接裁判系统，电源管理）祖传算法----已经实现
//该代码只控制3508功率，6020消耗功率越大3508停转的可能就越大
//考虑到6020电机转向的特性，为了保持方向的前进的正常，所以采用3508限速的方法进行
#define WARNING_ENERGY 60
float toatl_speed_err, out_speed_err[4], finaal_out[4];
float toatl_speed_err, out_speed_err_6020[4], finaal_out[4], out_speed_err_3508[4];
void Chassis_Power_Limit(AGV_chassis_t *agv_chassis)
{
	/*********************祖传算法*************************/
	float kLimit = 0.0f; //功率限制系数
	float fTotalCurrentLimit;
	float chassis_totaloutput = 0.0f; //统计总输出电流
	float Joule_Residue = 0.0f;		  //剩余焦耳缓冲能量
									  //	static int16_t judgDataError_Time = 0;

	Joule_Residue = agv_chassis->chassis_power_measure->chassis_power_buffer;
	for (int i = 0; i < 4; i++)
	{
		out_speed_err_6020[i] = agv_chassis->PID_6020_speed[i].set - agv_chassis->PID_6020_speed[i].fdb;
		out_speed_err_3508[i] = agv_chassis->PID_3508[i].set - agv_chassis->PID_3508[i].fdb;
	}
	chassis_totaloutput = abs((int16_t)agv_chassis->PID_6020_speed[0].out) + abs((int16_t)agv_chassis->PID_6020_speed[1].out) + abs((int16_t)agv_chassis->PID_6020_speed[2].out) + abs((int16_t)agv_chassis->PID_6020_speed[3].out) + abs((int16_t)agv_chassis->PID_3508[0].out) + abs((int16_t)agv_chassis->PID_3508[1].out) + abs((int16_t)agv_chassis->PID_3508[2].out) + abs((int16_t)agv_chassis->PID_3508[3].out);
	toatl_speed_err = abs((int16_t)out_speed_err_6020[0]) + abs((int16_t)out_speed_err_6020[1]) + abs((int16_t)out_speed_err_6020[2]) + abs((int16_t)out_speed_err_6020[3]) + abs((int16_t)out_speed_err_3508[0]) + abs((int16_t)out_speed_err_3508[1]) + abs((int16_t)out_speed_err_3508[2]) + abs((int16_t)out_speed_err_3508[3]);

	//	judgDataError_Time = 0;

	//剩余焦耳量过小,开始限制输出,限制系数为平方关系
	if (Joule_Residue < WARNING_ENERGY)
	{
		kLimit = (float)(Joule_Residue / WARNING_ENERGY) * (float)(Joule_Residue / WARNING_ENERGY);
		fTotalCurrentLimit = (kLimit * M3505_MOTOR_SPEED_PID_MAX_OUT * 4);
	}
	else //焦耳能量恢复到一定数值
	{
		fTotalCurrentLimit = (M3505_MOTOR_SPEED_PID_MAX_OUT * 4);
	}

	//底盘各电机电流重新分配
	if (chassis_totaloutput > fTotalCurrentLimit)
	{
		//赋值电流值
		//            power_ctrl->motor_speed_pid[0].out = ((power_ctrl->motor_speed_pid[0].out) / chassis_totaloutput * fTotalCurrentLimit);
		//            power_ctrl->motor_speed_pid[1].out = ((power_ctrl->motor_speed_pid[1].out) / chassis_totaloutput * fTotalCurrentLimit);
		//            power_ctrl->motor_speed_pid[2].out = ((power_ctrl->motor_speed_pid[2].out) / chassis_totaloutput * fTotalCurrentLimit);
		//            power_ctrl->motor_speed_pid[3].out = ((power_ctrl->motor_speed_pid[3].out) / chassis_totaloutput * fTotalCurrentLimit);
		agv_chassis->PID_3508[0].out = out_speed_err_3508[0] / toatl_speed_err * fTotalCurrentLimit;
		agv_chassis->PID_3508[1].out = out_speed_err_3508[1] / toatl_speed_err * fTotalCurrentLimit;
		agv_chassis->PID_3508[2].out = out_speed_err_3508[2] / toatl_speed_err * fTotalCurrentLimit;
		agv_chassis->PID_3508[3].out = out_speed_err_3508[3] / toatl_speed_err * fTotalCurrentLimit;
	}
	for (int i = 0; i < 4; i++)
	{
		finaal_out[i] = agv_chassis->PID_3508[0].out;
	}
}
//模式切换函数，美观代码用，不用也行
void change_mode(void)
{
	if (agv_chassis.AGV_rc->rc.s[0] == 3 && agv_chassis.AGV_rc->rc.s[1] == 3) //正常前进，舵轮跟随云台模式
	{
		agv_chassis.rc_mode = AGV_CHASSIS_FOLLOW_GIMBAL;
	}
	else if (agv_chassis.AGV_rc->rc.s[0] == 2 && agv_chassis.AGV_rc->rc.s[1] == 2)
	{
		agv_chassis.rc_mode = AGV_CHASSIS_RELAX;
	}
	else if (agv_chassis.AGV_rc->rc.s[0] == 3 && agv_chassis.AGV_rc->rc.s[1] == 1)
	{
		agv_chassis.rc_mode = AGV_CHASSIS_DODGE_MODE;
	}
	else if (agv_chassis.AGV_rc->rc.s[0] == 1 && agv_chassis.AGV_rc->rc.s[1] == 3)
	{
		agv_chassis.rc_mode = AGV_CHASSIS_KEY_MODE;
	}
	else if (agv_chassis.AGV_rc->rc.s[0] == 1 && agv_chassis.AGV_rc->rc.s[1] == 1)
	{
		agv_chassis.rc_mode = CHASSIS_FOLLOW_GIMBAL;
	}
}

/*********************************************************************超级电容**************************************************************************************/
void Super_power_ctrl(AGV_chassis_t *power_ctrl)
{
	uint16_t fTotalpowerLimit;

	if (power_ctrl->chassis_monitor_point[JudgementWDG].errorExist == 0)
	{
		fTotalpowerLimit = power_ctrl->chassis_status_measure->chassis_power_limit;
		CAN_CMD_SUPERPOWER(fTotalpowerLimit, Shift_Flag, power_ctrl->chassis_power_measure->chassis_power_buffer); //切换电源
	}
	else if (power_ctrl->chassis_monitor_point[JudgementWDG].errorExist == 1)
	{
		CAN_CMD_SUPERPOWER(20, Shift_Flag, 0); // 20W充电使用
	}
}

void chang(void)
{
	change_mode1 = agv_chassis.PID_6020_angle[0].set;
	change_mode2 = agv_chassis.PID_6020_angle[1].set;
	change_mode3 = agv_chassis.PID_6020_angle[2].set;
	change_mode4 = agv_chassis.PID_6020_angle[3].set;
}

void motor_speed_change(void) //用于速度预设
{
	if (Shift_Flag == 2) //未开启超级电容
	{
		if (agv_chassis.chassis_status_measure->chassis_power_limit == 50)
		{
			max_speed = 6.0606;
			DODGE_NUM = 1;
		}
		else if (agv_chassis.chassis_status_measure->chassis_power_limit == 60)
		{
			max_speed = 7.575;
			DODGE_NUM = 1;
		}
		else if (agv_chassis.chassis_status_measure->chassis_power_limit == 80)
		{
			max_speed = 9.090;
			DODGE_NUM = 1.2;
		}
		else if (agv_chassis.chassis_status_measure->chassis_power_limit == 100)
		{
			max_speed = 12;
			DODGE_NUM = 1.4;
		}
	}
	else if (Shift_Flag == 1) //开启超级电容
	{
		max_speed = 15;
		DODGE_NUM = 1.6;
	}
}

