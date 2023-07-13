//#include "canrecive.h"
///*
////ÐèÖØÐ´ °¥
//#define get_motor_measure(ptr, rx_message)                                              \
//{                                                                                       \
//    if((ptr)->ecd - (ptr)->last_ecd > 4096) (ptr)->count-- ;                            \
//		else if((ptr)->ecd - (ptr)->last_ecd < -4096 ) (ptr)->count ++ ;											\
//    (ptr)->last_ecd = (ptr)->ecd;                                                       \
//    (ptr)->ecd = (uint16_t)((rx_message).Data[0] << 8 | (rx_message).Data[1]);          \
//    (ptr)->speed_rpm = (uint16_t)((rx_message).Data[2] << 8 |(rx_message).Data[3]);     \
//    (ptr)->given_current = (uint16_t)((rx_message).Data[4] << 8 | (rx_message).Data[5]); \
//    (ptr)->temperate = (rx_message).Data[6];                                             \
//    (ptr)->all_ecd=(ptr)->count*8191+(ptr)->ecd;                                     \
//}
//*/



////M3508 M3508_control;
////GM6020 GM6020_control;
//int8_t send_data[8]={0};
//int8_t rec_data[8]={0};
//uint32_t pTxMailbox = 0;

//void moter_send_3508(int i1,int i2,int i3,int i4)
//{
//	CAN_TxHeaderTypeDef TxMessage;
//	
//	TxMessage.DLC=8;
//	TxMessage.IDE=CAN_ID_STD;
//	TxMessage.RTR=CAN_RTR_DATA;
//	TxMessage.StdId=0x200;
//	TxMessage.TransmitGlobalTime=DISABLE;
//	HAL_CAN_AddTxMessage(&hcan1, &TxMessage, (uint8_t*)send_data, &pTxMailbox);
//}


//CAN_RxHeaderTypeDef Rx1Message;

////void CAN1_RX0_IRQHandler(void)
////{
////	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Rx1Message, (uint8_t*)rec_data);
////	switch(Rx1Message.StdId)
////	{
////		case 0x201: get_motor_measure(chassis_contorl.M3508[0], rec_data);break;
////		case 0x202: get_motor_measure(&M3508_control.M3508[1], rec_data);break;
////		case 0x203: get_motor_measure(&M3508_control.M3508[2], rec_data);break;
////		case 0x204: get_motor_measure(&M3508_control.M3508[3], rec_data);break;
////	}

////}

////CanRxMsg Rx2Message;
////void CAN2_RX1_IRQHandler(void)
////{
////	CAN_Receive(CAN2,CAN_FIFO1,&Rx2Message);
////	switch(Rx2Message.StdId)
////	{
////		case 0x201: get_motor_measure(&M3508_control.M3508[0], Rx2Message);break;
////		case 0x202: get_motor_measure(&M3508_control.M3508[1], Rx2Message);break;
////		case 0x203: get_motor_measure(&M3508_control.M3508[2], Rx2Message);break;
////		case 0x204: get_motor_measure(&M3508_control.M3508[3], Rx2Message);break;
////		case 0x205: get_motor_measure(&GM6020_control.GM6020, Rx2Message);break;
////	}

////}


