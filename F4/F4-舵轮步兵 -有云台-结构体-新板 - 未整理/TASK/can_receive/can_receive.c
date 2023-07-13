#include "can_receive.h"
#include "detect_task.h"
#define CHASSIS_CAN CAN1
#define GIMBAL_CAN CAN1
#define SHOOT_CAN CAN2

motor_measure_t M3508[4];
motor_measure_t M6020[4];
motor_measure_t YAW;
motor_measure_t PITCH;
//声明超级电容模块
Super_power_t Super_power;
static void CAN1_hook(CanRxMsg *rx_message);
static void CAN2_hook(CanRxMsg *rx_message);

motor_measure_t motor_yaw, motor_pit, motor_trigger, motor_friction[2], motor_chassis[4], motor_6020_chassis[4];
Encoder_process_t Encoder_friction[2], Encoder_chassis[4], Encoder_6020_chassis[4], Encoder_pit, Encoder_yaw, Encoder_trigger;
static CanTxMsg GIMBAL_TxMessage;
CanRxMsg RxMessage;
void CAN1_RX0_IRQHandler(void)
{
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
    CAN1_hook(&RxMessage);
}
CanRxMsg Rx2Message;
void CAN2_RX1_IRQHandler(void)
{
    CAN_Receive(CAN2, CAN_FIFO1, &Rx2Message);
    CAN2_hook(&Rx2Message);
}
static void CAN1_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
        static uint8_t i = 0;
        i = rx_message->StdId - CAN_3508_M1_ID;
        get_motor_measure(&motor_chassis[i], *rx_message);
        EncoderProcess3508(&Encoder_chassis[i], &motor_chassis[i]);
        //记录时间
        DetectHook(ChassisMotor1TOE + i);
        break;
    }

    case CAN_YAW_MOTOR_ID:
    {
        get_motor_measure(&motor_yaw, *rx_message);
        EncoderProcess6020(&Encoder_yaw, &motor_yaw);
        //记录时间
        DetectHook(YawGimbalMotorTOE);
        break;
    }
    case CAN_PIT_MOTOR_ID:
    {
        get_motor_measure(&motor_pit, *rx_message);
        EncoderProcess6020(&Encoder_pit, &motor_yaw);
        //记录时间
        DetectHook(PitchGimbalMotorTOE);
        break;
    }
    case 0x300:
    {
        Super_power.volt = (uint16_t)((rx_message)->Data[1] << 8 | (rx_message)->Data[0]);
        Super_power.power = (uint16_t)((rx_message)->Data[3] << 8 | (rx_message)->Data[2]);
        Super_power.current = (uint16_t)((rx_message)->Data[5] << 8 | (rx_message)->Data[4]);
    }

    default:
    {
        break;
    }
    }
}
static void CAN2_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
    case CAN_TRIGGER_MOTOR_ID:
    {
        get_motor_measure(&motor_trigger, *rx_message);
        EncoderProcess3508(&Encoder_trigger, &motor_trigger);
        //记录时间
        DetectHook(TriggerMotorTOE);
        break;
    }

    case CAN_FRICTION_right_ID:
    case CAN_FRICTION_left_ID:
    {
        static uint8_t i = 0;
        i = rx_message->StdId - CAN_FRICTION_right_ID;
        get_motor_measure(&motor_friction[i], *rx_message);
        EncoderProcess3508(&Encoder_friction[i], &motor_friction[i]);
        //记录时间
        DetectHook(frictionmotorRTOE + i);
        break;
    }
    case CAN_6020_M1_ID:
    case CAN_6020_M2_ID:
    case CAN_6020_M3_ID:
    case CAN_6020_M4_ID:
    {
        static uint8_t i = 0;
        i = rx_message->StdId - CAN_6020_M1_ID;
        get_motor_measure(&motor_6020_chassis[i], *rx_message);
        EncoderProcess6020(&Encoder_6020_chassis[i], &motor_6020_chassis[i]);
        //记录时间
        DetectHook(ChassisMotor1TOE + i);
        break;
    }
    default:
    {
        break;
    }
    }
}

//各个电机的指针获取
const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void) //云台yaw轴电机
{
    return &motor_yaw;
}
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void) //云台pitch轴电机
{
    return &motor_pit;
}
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i) //底盘电机
{
    return &motor_chassis[(i & 0x03)];
}
//返回trigger电机变量地址，通过指针方式获取原始数据（拨盘）
const motor_measure_t *get_Trigger_Motor_Measure_Point(void)
{
    return &motor_trigger;
}
//返回Friction电机变量地址，通过指针方式获取原始数据（摩擦轮）
const motor_measure_t *get_Friction_Motor_Measure_Point(uint8_t i)
{
    return &motor_friction[(i & 0x03)];
}
//返回底盘6020电机变量地址，通过指针的方式获取原始数据
const motor_measure_t *get_Chassis_6020_Motor_Measure_Point(uint8_t i) //底盘6020电机
{
    return &motor_6020_chassis[(i & 0x03)];
}
const Encoder_process_t *get_Chassis_Encoder_Measure_Point(uint8_t i) //编码器
{
    return &Encoder_chassis[(i & 0x03)];
}
const Encoder_process_t *get_Chassis_6020_Encoder_Measure_Point(uint8_t i) //编码器
{
    return &Encoder_6020_chassis[(i & 0x03)];
}
const Encoder_process_t *get_Pitch_Gimbal_Encoder_Measure_Point(void) //编码器
{
    return &Encoder_pit;
}
const Encoder_process_t *get_Yaw_Gimbal_Encoder_Measure_Point(void) //编码器
{
    return &Encoder_yaw;
}
//返回摩擦电机编码器变量地址，通过指针方式获取原始数据
const Encoder_process_t *get_Friction_Encoder_Measure_Point(uint8_t i)
{
    return &Encoder_friction[(i & 0x03)];
}
//返回trigger编码器变量地址，通过指针方式获取原始数据
const Encoder_process_t *get_Trigger_Encoder_Measure_Point(void)
{
    return &Encoder_trigger;
}
void EncoderProcess3508(Encoder_process_t *v, motor_measure_t *motor)
{
    int32_t temp_sum = 0;
    v->diff = motor->ecd - motor->last_ecd;

    if (v->diff < -6500)
    {
        v->round_cnt++;
        v->ecd_raw_rate = v->diff + 8192;
    }
    else if (v->diff > 6500)
    {
        v->round_cnt--;
        v->ecd_raw_rate = v->diff - 8192;
    }
    else
    {
        v->ecd_raw_rate = v->diff;
    }

    v->rate_buf[v->buf_count++] = v->ecd_raw_rate;

    if (v->buf_count == RATE_BUF_SIZE)
    {
        v->buf_count = 0;
    }

    for (uint8_t i = 0; i < RATE_BUF_SIZE; i++)
    {
        temp_sum += v->rate_buf[i];
    }

    v->filter_rate = (int32_t)(temp_sum / RATE_BUF_SIZE);
}

void EncoderProcess6020(Encoder_process_t *v, motor_measure_t *motor)
{
    int32_t temp_sum = 0;
    v->diff = motor->ecd - motor->last_ecd;

    if (v->diff < -4096)
    {
        v->round_cnt++;
        v->ecd_raw_rate = v->diff + 8192;
    }
    else if (v->diff > 4096)
    {
        v->round_cnt--;
        v->ecd_raw_rate = v->diff - 8192;
    }
    else
    {
        v->ecd_raw_rate = v->diff;
    }

    v->rate_buf[v->buf_count++] = v->ecd_raw_rate;

    if (v->buf_count == RATE_BUF_SIZE)
    {
        v->buf_count = 0;
    }

    for (uint8_t i = 0; i < RATE_BUF_SIZE; i++)
    {
        temp_sum += v->rate_buf[i];
    }

    v->filter_rate = (int32_t)(temp_sum / RATE_BUF_SIZE);
}

void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

void CAN_CMD_GIMBAL(int16_t pitch, int16_t yaw, int16_t shoot, int16_t rev)
{
    GIMBAL_TxMessage.StdId = CAN_GIMBAL_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (pitch >> 8);
    GIMBAL_TxMessage.Data[1] = pitch;
    GIMBAL_TxMessage.Data[2] = (yaw >> 8);
    GIMBAL_TxMessage.Data[3] = yaw;
    GIMBAL_TxMessage.Data[4] = (shoot >> 8);
    GIMBAL_TxMessage.Data[5] = shoot;
    GIMBAL_TxMessage.Data[6] = (rev >> 8);
    GIMBAL_TxMessage.Data[7] = rev;

    CAN_Transmit(GIMBAL_CAN, &GIMBAL_TxMessage);
}

void CAN_CMD_Shoot(int16_t trigger, int16_t fri_right, int16_t fri_left)
{
    CanTxMsg SendCanTxMsg;
    SendCanTxMsg.StdId = 0x200;
    SendCanTxMsg.IDE = CAN_ID_STD;
    SendCanTxMsg.RTR = CAN_RTR_DATA;
    SendCanTxMsg.DLC = 0x08;
    SendCanTxMsg.Data[0] = trigger >> 8;
    SendCanTxMsg.Data[1] = trigger;
    SendCanTxMsg.Data[2] = fri_right >> 8;
    SendCanTxMsg.Data[3] = fri_right;
    SendCanTxMsg.Data[4] = fri_left >> 8;
    SendCanTxMsg.Data[5] = fri_left;
    SendCanTxMsg.Data[6] = 0;
    SendCanTxMsg.Data[7] = 0;

    CAN_Transmit(SHOOT_CAN, &SendCanTxMsg);
}

//============================6020发送函数===============================
void Motor_Set_Current(signed short int i1, signed short int i2, signed short int i3, signed short int i4)
{
    CanTxMsg tx_message;

    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08; // 0x02对应一个电机  0x08 对应4个电机
    tx_message.StdId = 0x1ff;

    tx_message.Data[0] = i1 >> 8;
    tx_message.Data[1] = i1;
    tx_message.Data[2] = i2 >> 8;
    tx_message.Data[3] = i2;
    tx_message.Data[4] = i3 >> 8;
    tx_message.Data[5] = i3;
    tx_message.Data[6] = i4 >> 8;
    tx_message.Data[7] = i4;

    CAN_Transmit(CAN2, &tx_message);
}

Tx_Union_data Can_Tx_Data;
void CAN_CMD_SUPERPOWER(int16_t power, int16_t i, uint16_t buffer_power)
{
    CanTxMsg SendCanTxMsg;
    Can_Tx_Data.TX_data.power = power;
    Can_Tx_Data.TX_data.flag = i;
    Can_Tx_Data.TX_data.buffer_power = buffer_power;
    SendCanTxMsg.StdId = 0x222;
    SendCanTxMsg.IDE = CAN_ID_STD;
    SendCanTxMsg.RTR = CAN_RTR_DATA;
    SendCanTxMsg.DLC = 0x08;
    for (int i = 0; i < sizeof(Send_Data); i++)
    {
        SendCanTxMsg.Data[i] = Can_Tx_Data.Array_Tx_data[i];
    }
    CAN_Transmit(CHASSIS_CAN, &SendCanTxMsg);
}
