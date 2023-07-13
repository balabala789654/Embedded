#ifndef __CAN_RECEIVE_H
#define __CAN_RECEIVE_H
#include "sys.h"
#include "can.h"

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x206,
    CAN_PIT_MOTOR_ID = 0x205,
    CAN_GIMBAL_ALL_ID = 0x1FF,

    CAN_TRIGGER_MOTOR_ID = 0x201,
    CAN_FRICTION_right_ID = 0x202,
    CAN_FRICTION_left_ID = 0x203,

    CAN_6020_M1_ID = 0x205,
    CAN_6020_M2_ID = 0x206,
    CAN_6020_M3_ID = 0x207,
    CAN_6020_M4_ID = 0x208,

} can_msg_id_e;
//���ͳһ�Ľṹ��
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    int32_t all_ecd; //��������ֵ(��ֵ)
    int32_t count;

    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;
typedef struct 
{
  uint16_t power;
	uint16_t flag;
	uint16_t buffer_power;
}Send_Data;

typedef union 
{
  Send_Data TX_data;
  uint8_t Array_Tx_data[sizeof(Send_Data)];
}Tx_Union_data;

typedef struct
{
    uint16_t 	volt;
    uint16_t	power;
    uint16_t	current;
} Super_power_t;

#define RATE_BUF_SIZE 5
typedef struct
{
    int32_t diff;
    int32_t round_cnt;
    int32_t ecd_raw_rate;
    int32_t rate_buf[RATE_BUF_SIZE]; // buf��for filter
    uint8_t buf_count;               //�˲�����buf��
    int32_t filter_rate;             //�ٶ�
} Encoder_process_t;

//���̵�����ݶ�ȡ
#define get_motor_measure(ptr, rx_message)                                                   \
    {                                                                                        \
        if ((ptr)->ecd - (ptr)->last_ecd > 4096)                                             \
            (ptr)->count--;                                                                  \
        else if ((ptr)->ecd - (ptr)->last_ecd < -4096)                                       \
            (ptr)->count++;                                                                  \
        (ptr)->last_ecd = (ptr)->ecd;                                                        \
        (ptr)->ecd = (uint16_t)((rx_message).Data[0] << 8 | (rx_message).Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message).Data[2] << 8 | (rx_message).Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message).Data[4] << 8 | (rx_message).Data[5]); \
        (ptr)->temperate = (rx_message).Data[6];                                             \
        (ptr)->all_ecd = (ptr)->count * 8191 + (ptr)->ecd;                                   \
    }

//��̨������ݶ�ȡ
#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        if ((ptr)->ecd - (ptr)->last_ecd > 4096)                                             \
            (ptr)->count--;                                                                  \
        else if ((ptr)->ecd - (ptr)->last_ecd < -4096)                                       \
            (ptr)->count++;                                                                  \
        (ptr)->last_ecd = (ptr)->ecd;                                                        \
        (ptr)->ecd = (uint16_t)((rx_message).Data[0] << 8 | (rx_message).Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message).Data[2] << 8 | (rx_message).Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message).Data[4] << 8 | (rx_message).Data[5]); \
        (ptr)->temperate = (rx_message).Data[6];                                             \
        (ptr)->all_ecd = (ptr)->count * 8191 + (ptr)->ecd;                                        \
    }

//������̨�����������revΪ�����ֽ�
extern void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
//���͵��̵����������
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//�������ģ���������
extern void CAN_CMD_Shoot(int16_t trigger, int16_t fri_right, int16_t fri_left);
//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����,i�ķ�Χ��0-3����Ӧ0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
//����trigger,friction���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
extern const motor_measure_t *get_Friction_Motor_Measure_Point(uint8_t i);

//���ص��̵��������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const Encoder_process_t *get_Chassis_Encoder_Measure_Point(uint8_t i);
//����yaw������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const Encoder_process_t *get_Yaw_Gimbal_Encoder_Measure_Point(void);
//����pit������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const Encoder_process_t *get_Pitch_Gimbal_Encoder_Measure_Point(void);
//����Ħ�����������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const Encoder_process_t *get_Friction_Encoder_Measure_Point(uint8_t i);
//����trigger������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const Encoder_process_t *get_Trigger_Encoder_Measure_Point(void);


const motor_measure_t *get_Chassis_6020_Motor_Measure_Point(uint8_t i); //����6020���
const Encoder_process_t *get_Chassis_6020_Encoder_Measure_Point(uint8_t i); //������

void Motor_Set_Current(signed short int i1, signed short int i2, signed short int i3, signed short int i4);
void Motor_Set_Current_3508(signed short int i1, signed short int i2, signed short int i3, signed short int i4);
extern void EncoderProcess3508(Encoder_process_t *v, motor_measure_t *motor);
extern void EncoderProcess6020(Encoder_process_t *v, motor_measure_t *motor);
void CAN_CMD_SUPERPOWER(int16_t power, int16_t i,uint16_t buffer_power);
void Motor_Set_Current(signed short int i1, signed short int i2, signed short int i3, signed short int i4);
#endif
