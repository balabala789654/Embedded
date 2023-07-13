///*
//���ֲ������� lin
//��Ҫ����ȫ�ֱ�����д����û�в��ýṹ��ȥ�򻯱���
//ʵ�֣����ֲ����Ķ��ָ�����̨�����ж��ֵ��̵�С�����㷨����
//�Լ�������ɶ��ֲ����ļ��̿��ƣ�WASD��12.25
//����ʵ��С���ݽ��㣬��ɶ��ֲ����İ���������ͬʱ����ͨ�������������еڶ��ֶ��ֵĿ���---2022.2.23������У���ԣ�
//��һ���������ֲ�������Ϊ��ͨ����������һ��ģʽ���Ӷ�ʵ�ֲ�������Ŀ��ơ�������2022.2.23
//���ֲ���ȫ�ֱ�����Ϊ�ṹ����� -----2022.3.10
//���ֲ����������ܲ�����ɣ�����С������ת�Ĺ����г��ֶ������������ķ�ת������Ϊ������Ƕ���������ԭ�� ------2022.5.5
//*/
#include "chassis_task.h"
#define number 12.12 // 3508ֱ�ӻ�ȡͨ��ֵ��numberΪ��ɱ����ı���
float ch0, ch1, res;
uint8_t FFlag_state = 0;
uint8_t CTRLFlag_state = 0;
float change_yaw;
AGV_chassis_t agv_chassis; //���ֵ�����ؽṹ��
extern Gimbal_Control_t gimbal_control;
extern float yaw_relative;
extern int yaw_count;
extern Super_power_t Super_power;
int count11 = 0, count22 = 0, count33 = 0, count44 = 0;
float  max_speed = number;
float change_mode1 = 0, change_mode2 = 0, change_mode3 = 0, change_mode4 = 0;
float DODGE_NUM = 1;
//��Ϊ����4��6020����ĵ�����ܿ��ܻ���ڲ������ʹ��4��pid��ȷ����
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
void Steering_wheel(int vx, int vy, int vz);		  //���ֵ�С���ݽ���
void Chassis_Power_Limit(AGV_chassis_t *agv_chassis); //���ֵĹ��ʿ���
void chassis_Init(AGV_chassis_t *agv_chassis);		  //���ֳ�ʼ��
void KEY_UPDATED(void);
void Super_power_ctrl(AGV_chassis_t *power_ctrl);
RampGen_t chassis_WRamp, chassis_ARamp, chassis_SRamp, chassis_DRamp;
void change_mode(void);
void chang(void);
void motor_speed_change(void); //�����ٶ�Ԥ��
float sin_yaw = 0, cos_yaw = 0;
float all_relative_angle = 0;
float re_yaw = 0, ab_yaw;
int count_chassis, cha, F_flag = 0;
float change_Chassis_tan;
void chassis_task(void *pvParameters)
{
	float ch0, ch1, res;
	chassis_Init(&agv_chassis); //�Ե���������ݵĳ�ʼ��
	while (1)
	{
		motor_speed_change();
		all_relative_angle = (agv_chassis.chassis_yaw_motor->gimbal_motor_measure->all_ecd + 1405) / ECD;
		change_mode();
		Super_power_ctrl(&agv_chassis);
		// 3508���pid����
		PID_Calc(&agv_chassis.PID_3508[0], agv_chassis.motor_chassis_3508[0].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[0]);
		PID_Calc(&agv_chassis.PID_3508[1], agv_chassis.motor_chassis_3508[1].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[1]);
		PID_Calc(&agv_chassis.PID_3508[2], agv_chassis.motor_chassis_3508[2].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[2]);
		PID_Calc(&agv_chassis.PID_3508[3], agv_chassis.motor_chassis_3508[3].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[3]);
		// 6020���pid˫������
		PID_Calc(&agv_chassis.PID_6020_angle[0], agv_chassis.motor_chassis_6020[0].chassis_motor_measure->all_ecd / ECD, agv_chassis.set_6020[0]);
		PID_Calc(&agv_chassis.PID_6020_angle[1], agv_chassis.motor_chassis_6020[1].chassis_motor_measure->all_ecd / ECD, agv_chassis.set_6020[1]);
		PID_Calc(&agv_chassis.PID_6020_angle[2], agv_chassis.motor_chassis_6020[2].chassis_motor_measure->all_ecd / ECD, agv_chassis.set_6020[2]);
		PID_Calc(&agv_chassis.PID_6020_angle[3], agv_chassis.motor_chassis_6020[3].chassis_motor_measure->all_ecd / ECD, agv_chassis.set_6020[3]);
		PID_Calc(&agv_chassis.PID_6020_speed[0], agv_chassis.motor_chassis_6020[0].chassis_motor_measure->speed_rpm, agv_chassis.PID_6020_angle[0].out);
		PID_Calc(&agv_chassis.PID_6020_speed[1], agv_chassis.motor_chassis_6020[1].chassis_motor_measure->speed_rpm, agv_chassis.PID_6020_angle[1].out);
		PID_Calc(&agv_chassis.PID_6020_speed[2], agv_chassis.motor_chassis_6020[2].chassis_motor_measure->speed_rpm, agv_chassis.PID_6020_angle[2].out);
		PID_Calc(&agv_chassis.PID_6020_speed[3], agv_chassis.motor_chassis_6020[3].chassis_motor_measure->speed_rpm, agv_chassis.PID_6020_angle[3].out);
		if (agv_chassis.rc_mode == AGV_CHASSIS_FOLLOW_GIMBAL) //����ǰ�������ָ�����̨ģʽ
		{

			agv_chassis.real_vz = 0; //С�����Ƿ����ı�־
			ch0 = agv_chassis.AGV_rc->rc.ch[0];
			ch1 = agv_chassis.AGV_rc->rc.ch[1];
			res = atan2(ch0, ch1) * 180 / 3.14159; //������ͨ�����нǶ�ת��
												   //�����е����ݽ��б��棬��ȷ��С���ݿ�����ʱ�򲻻���ԭλ
			agv_chassis.change1 = yaw_count * 360;
			agv_chassis.change2 = yaw_count * 360;
			agv_chassis.change3 = yaw_count * 360;
			agv_chassis.change4 = yaw_count * 360;
			change_yaw = abs(agv_chassis.chassis_yaw_motor->gimbal_motor_measure->count + 1) * 360;

			//�����˶��Ĵ���ʵ��---��bug�޷�������󷽵�ת��
			//����Ϊ�Զ���ת���ԭ���Ǵ�0-90��0--90������-90�����ݴ����ʱ������ת�򶯵ķ��򲻶Ե����⣩
			//ͨ����̨����ԽǶȶԵ��̵������ͬ�Ƕȵ�ת��δ���ԣ�
			if (res >= 0)
			{
				agv_chassis.set_6020[0] = M6020_m1 - yaw_relative + res - count11 * 360; // chassis_yaw_motor->relative_angle�Ǵ���̨��ȡ����ԽǶ�
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
					agv_chassis.set_6020[0] = M6020_m1 - yaw_relative + res + 180 - count11 * 360; // chassis_yaw_motor->relative_angle�Ǵ���̨��ȡ����ԽǶ�
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
		else if (agv_chassis.rc_mode == AGV_CHASSIS_RELAX) //����ģʽ
		{
			agv_chassis.real_vz = 0;
			Chassis_Power_Limit(&agv_chassis);
			CAN_CMD_CHASSIS(0, 0, 0, 0);
			Motor_Set_Current(0, 0, 0, 0);
		}
		//С�����˶�
		//δ��ȫ��ɣ�С��������ƶ�Ϊʵ�֣���������ֲ�����ǣ����ü��ɣ�
		//�����ȫ��С�����˶���δ����
		else if (agv_chassis.rc_mode == AGV_CHASSIS_DODGE_MODE)
		{
			F_flag = 1;
			//С������ؽ����ʵ�֣�δ���ԣ�
			agv_chassis.real_vz = -660;
			agv_chassis.RC_X_ChassisSpeedRef = agv_chassis.AGV_rc->rc.ch[1];
			agv_chassis.RC_Y_ChassisSpeedRef = agv_chassis.AGV_rc->rc.ch[0];
			sin_yaw = arm_sin_f32((agv_chassis.chassis_yaw_motor->relative_angle) * ANGLE_TO_RAD); // arm_sin_f32����ֱ���ýǶ�ֵ��Ҫ����ת��Ϊ����ֵ
			cos_yaw = arm_cos_f32((agv_chassis.chassis_yaw_motor->relative_angle) * ANGLE_TO_RAD);
			agv_chassis.real_vx = (cos_yaw * agv_chassis.RC_X_ChassisSpeedRef + sin_yaw * agv_chassis.RC_Y_ChassisSpeedRef);
			agv_chassis.real_vy = (-sin_yaw * agv_chassis.RC_X_ChassisSpeedRef + cos_yaw * agv_chassis.RC_Y_ChassisSpeedRef);
			Steering_wheel(agv_chassis.real_vx, agv_chassis.real_vy, agv_chassis.real_vz);
			Chassis_Power_Limit(&agv_chassis);
//			CAN_CMD_CHASSIS(agv_chassis.PID_3508[0].out, agv_chassis.PID_3508[1].out, agv_chassis.PID_3508[2].out, agv_chassis.PID_3508[3].out);
			Motor_Set_Current(agv_chassis.PID_6020_speed[0].out, agv_chassis.PID_6020_speed[1].out, agv_chassis.PID_6020_speed[2].out, agv_chassis.PID_6020_speed[3].out);
		}
		else if (agv_chassis.rc_mode == CHASSIS_FOLLOW_GIMBAL) //��ͳ���̸���ģʽ
		{
			float FOLLOW_GIMBAL_Z; //������ת
			fp32 Rotation_rate = 0.0f;
			int chassis_relative_angle_set = 0.0f;
			agv_chassis.RC_X_ChassisSpeedRef = agv_chassis.AGV_rc->rc.ch[1];
			agv_chassis.RC_Y_ChassisSpeedRef = agv_chassis.AGV_rc->rc.ch[0];
			sin_yaw = arm_sin_f32(agv_chassis.chassis_yaw_motor->relative_angle * ANGLE_TO_RAD); // arm_sin_f32����ֱ���ýǶ�ֵ��Ҫ����ת��Ϊ����ֵ
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
		//���̿���.��Ҫ��WASD���ƣ����Ƶ��̵�ȫ���ƶ�
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
				// if (G_Flag == 1) //���ģʽ�µ��̲���
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
			//��Ϊ�����˶���С�������ý��㷽ʽ��ͬ�������ڴ������ж�
			while (FFlag_state == 0x01) //��һ��f����С���ݣ��ٰ���һ��С���ݹر�
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
				sin_yaw = arm_sin_f32(agv_chassis.chassis_yaw_motor->relative_angle * ANGLE_TO_RAD); // arm_sin_f32����ֱ���ýǶ�ֵ��Ҫ����ת��Ϊ����ֵ
				cos_yaw = arm_cos_f32(agv_chassis.chassis_yaw_motor->relative_angle * ANGLE_TO_RAD);
				agv_chassis.real_vx = (cos_yaw * agv_chassis.RC_X_ChassisSpeedRef + sin_yaw * agv_chassis.RC_Y_ChassisSpeedRef);
				agv_chassis.real_vy = (-sin_yaw * agv_chassis.RC_X_ChassisSpeedRef + cos_yaw * agv_chassis.RC_Y_ChassisSpeedRef);

				Steering_wheel(agv_chassis.real_vx, agv_chassis.real_vy, agv_chassis.real_vz);

				// 3508���pid����
				PID_Calc(&agv_chassis.PID_3508[0], agv_chassis.motor_chassis_3508[0].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[0]);
				PID_Calc(&agv_chassis.PID_3508[1], agv_chassis.motor_chassis_3508[1].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[1]);
				PID_Calc(&agv_chassis.PID_3508[2], agv_chassis.motor_chassis_3508[2].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[2]);
				PID_Calc(&agv_chassis.PID_3508[3], agv_chassis.motor_chassis_3508[3].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[3]);
				// 6020���pid˫������
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
				//��ȡ��������
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
				float FOLLOW_GIMBAL_Z; //����ó�����ת�ٶ�
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

				// 3508���pid����
				PID_Calc(&agv_chassis.PID_3508[0], agv_chassis.motor_chassis_3508[0].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[0]);
				PID_Calc(&agv_chassis.PID_3508[1], agv_chassis.motor_chassis_3508[1].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[1]);
				PID_Calc(&agv_chassis.PID_3508[2], agv_chassis.motor_chassis_3508[2].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[2]);
				PID_Calc(&agv_chassis.PID_3508[3], agv_chassis.motor_chassis_3508[3].chassis_motor_measure->speed_rpm, agv_chassis.set_3508[3]);
				// 6020���pid˫������
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
/*�������ݸ���
  WASDǰ������
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

	//���� Shift ʹ�ó�������
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

void chassis_Init(AGV_chassis_t *agv_chassis) //���̳�ʼ������
{
	RampInit(&chassis_WRamp, 0);
	RampInit(&chassis_ARamp, 0);
	RampInit(&chassis_SRamp, 0);
	RampInit(&chassis_DRamp, 0);

	agv_chassis->chassis_yaw_motor = get_yaw_motor_point();		  //��ȡ��̨yaw�������
	agv_chassis->chassis_pitch_motor = get_pitch_motor_point();	  //��ȡ��̨pitch���
	agv_chassis->chassis_power_measure = get_power_heat_data_t(); //��ȡ���̹���
	//��ȡ���ϵͳ����ָ��
	agv_chassis->chassis_monitor_point = getErrorListPoint();
	agv_chassis->chassis_status_measure = get_game_robot_state_t();
	agv_chassis->chassis_hurt_type = get_robot_hurt_t();
	for (int i = 0; i < 4; i++) //��ȡ3508
	{
		agv_chassis->motor_chassis_3508[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
		agv_chassis->motor_chassis_3508[i].chassis_encoder_measure = get_Chassis_Encoder_Measure_Point(i);
	}

	for (int i = 0; i < 4; i++) //��ȡ6020�������
	{
		agv_chassis->motor_chassis_6020[i].chassis_motor_measure = get_Chassis_6020_Motor_Measure_Point(i);
		agv_chassis->motor_chassis_6020[i].chassis_encoder_measure = get_Chassis_6020_Encoder_Measure_Point(i);
	}
	//��ȡң����ָ��
	agv_chassis->AGV_rc = get_remote_control_point();
	//���е�������ĳ�ʼ��
	agv_chassis->set_3508[0] = 0;
	agv_chassis->set_3508[1] = 0;
	agv_chassis->set_3508[2] = 0;
	agv_chassis->set_3508[3] = 0;
	agv_chassis->set_6020[0] = M6020_m1;
	agv_chassis->set_6020[1] = M6020_m2;
	agv_chassis->set_6020[2] = M6020_m3;
	agv_chassis->set_6020[3] = M6020_m4;
	//========================3508pid��ʼ��=========================================================
	PID_Init(&agv_chassis->PID_3508[0], PID_POSITION, M3508_speed_pid, M3508_SPEED_OUT_MAX, M3508_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_3508[1], PID_POSITION, M3508_speed_pid, M3508_SPEED_OUT_MAX, M3508_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_3508[2], PID_POSITION, M3508_speed_pid, M3508_SPEED_OUT_MAX, M3508_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_3508[3], PID_POSITION, M3508_speed_pid, M3508_SPEED_OUT_MAX, M3508_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	//========================6020pid��ʼ��================================================================
	PID_Init(&agv_chassis->PID_6020_speed[0], PID_POSITION, M6020_speed_pid1, M6020_SPEED_OUT_MAX, M6020_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_6020_speed[1], PID_POSITION, M6020_speed_pid2, M6020_SPEED_OUT_MAX, M6020_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_6020_speed[2], PID_POSITION, M6020_speed_pid3, M6020_SPEED_OUT_MAX, M6020_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_6020_speed[3], PID_POSITION, M6020_speed_pid4, M6020_SPEED_OUT_MAX, M6020_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	//========================6020����pid==================================================================
	PID_Init(&agv_chassis->PID_6020_angle[0], PID_POSITION, M6020_angle_pid1, M6020_SPEED_OUT_MAX, M6020_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_6020_angle[1], PID_POSITION, M6020_angle_pid2, M6020_SPEED_OUT_MAX, M6020_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_6020_angle[2], PID_POSITION, M6020_angle_pid3, M6020_SPEED_OUT_MAX, M6020_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);
	PID_Init(&agv_chassis->PID_6020_angle[3], PID_POSITION, M6020_angle_pid4, M6020_SPEED_OUT_MAX, M6020_SPEED_MAX_IOUT, 0, 0, 0, 0, 0);

	//�������������̨ʹ��
	PID_Init(&agv_chassis->Righting, PID_POSITION, chassis_follow_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, 0, 0, 0, 0, 0);
}
//�����˶�����
//��ҪӦ����С���ݵ�ʱ����̬���㣬�����ʱ�򲻵��ô˺���
int vx1, vx2, vx3, vx4, vy1, vy2, vy3, vy4, w1, w2, w3, w4; //С������ؽ���
int v1, v2, v3, v4;
float tan1 = 0, tan2 = 0, tan3 = 0, tan4 = 0;
float tan11 = 0, tan22 = 0, tan33 = 0, tan44 = 0;
void Steering_wheel(int vx, int vy, int vz)
{
	vx1 = vx * max_speed; //��ȡͨ��ֵ����ͨ��������ʽ��x��������ٶȸ�ֵ���˴�������ͨ��ֵ��
	vx2 = vx * max_speed;
	vx3 = vx * max_speed;
	vx4 = vx * max_speed;

	vy1 = vy * max_speed; //��ȡy���ٶ�
	vy2 = vy * max_speed;
	vy3 = vy * max_speed;
	vy4 = vy * max_speed;

	w1 = vz * max_speed; //��ȡz���ٶ�
	w2 = vz * max_speed;
	w3 = vz * max_speed;
	w4 = vz * max_speed;

	//��3508���ٶȽ������--��Ϊ�����ֵȫ������ֵ���Ӷ��ԽǶȽ����ж�
	//��cos��sinת����0��707����ֹ��Ϊ��ֵ���������������������ת����û�в��ԣ���
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

//���̹������ƣ��Ӳ���ϵͳ����Դ�����洫�㷨----�Ѿ�ʵ��
//�ô���ֻ����3508���ʣ�6020���Ĺ���Խ��3508ͣת�Ŀ��ܾ�Խ��
//���ǵ�6020���ת������ԣ�Ϊ�˱��ַ����ǰ�������������Բ���3508���ٵķ�������
#define WARNING_ENERGY 60
float toatl_speed_err, out_speed_err[4], finaal_out[4];
float toatl_speed_err, out_speed_err_6020[4], finaal_out[4], out_speed_err_3508[4];
void Chassis_Power_Limit(AGV_chassis_t *agv_chassis)
{
	/*********************�洫�㷨*************************/
	float kLimit = 0.0f; //��������ϵ��
	float fTotalCurrentLimit;
	float chassis_totaloutput = 0.0f; //ͳ�����������
	float Joule_Residue = 0.0f;		  //ʣ�ཹ����������
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

	//ʣ�ཹ������С,��ʼ�������,����ϵ��Ϊƽ����ϵ
	if (Joule_Residue < WARNING_ENERGY)
	{
		kLimit = (float)(Joule_Residue / WARNING_ENERGY) * (float)(Joule_Residue / WARNING_ENERGY);
		fTotalCurrentLimit = (kLimit * M3505_MOTOR_SPEED_PID_MAX_OUT * 4);
	}
	else //���������ָ���һ����ֵ
	{
		fTotalCurrentLimit = (M3505_MOTOR_SPEED_PID_MAX_OUT * 4);
	}

	//���̸�����������·���
	if (chassis_totaloutput > fTotalCurrentLimit)
	{
		//��ֵ����ֵ
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
//ģʽ�л����������۴����ã�����Ҳ��
void change_mode(void)
{
	if (agv_chassis.AGV_rc->rc.s[0] == 3 && agv_chassis.AGV_rc->rc.s[1] == 3) //����ǰ�������ָ�����̨ģʽ
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

/*********************************************************************��������**************************************************************************************/
void Super_power_ctrl(AGV_chassis_t *power_ctrl)
{
	uint16_t fTotalpowerLimit;

	if (power_ctrl->chassis_monitor_point[JudgementWDG].errorExist == 0)
	{
		fTotalpowerLimit = power_ctrl->chassis_status_measure->chassis_power_limit;
		CAN_CMD_SUPERPOWER(fTotalpowerLimit, Shift_Flag, power_ctrl->chassis_power_measure->chassis_power_buffer); //�л���Դ
	}
	else if (power_ctrl->chassis_monitor_point[JudgementWDG].errorExist == 1)
	{
		CAN_CMD_SUPERPOWER(20, Shift_Flag, 0); // 20W���ʹ��
	}
}

void chang(void)
{
	change_mode1 = agv_chassis.PID_6020_angle[0].set;
	change_mode2 = agv_chassis.PID_6020_angle[1].set;
	change_mode3 = agv_chassis.PID_6020_angle[2].set;
	change_mode4 = agv_chassis.PID_6020_angle[3].set;
}

void motor_speed_change(void) //�����ٶ�Ԥ��
{
	if (Shift_Flag == 2) //δ������������
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
	else if (Shift_Flag == 1) //������������
	{
		max_speed = 15;
		DODGE_NUM = 1.6;
	}
}

