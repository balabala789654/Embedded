#include "canrecive.h"

M3508 M3508_control[4];
M2006 M2006_control[4];

void moter_send_3508(int i1,int i2,int i3,int i4)
{
	CanTxMsg TxMessage;
	
	TxMessage.DLC=8;
	TxMessage.IDE=CAN_Id_Standard;
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.StdId=0x200;
	
	TxMessage.Data[0]=i1 >> 8;
	TxMessage.Data[1]=i1;
	TxMessage.Data[2]=i2 >> 8;
	TxMessage.Data[3]=i2;
	TxMessage.Data[4]=i3 >> 8;
	TxMessage.Data[5]=i3;
	TxMessage.Data[6]=i4 >> 8;
	TxMessage.Data[7]=i4; 
	
	CAN_Transmit(CAN1,&TxMessage);
}

void motor_send_2006(int i1,int i2,int i3,int i4)
{
	
	CanTxMsg TxMessage;	
	TxMessage.DLC=8;
	TxMessage.IDE=CAN_Id_Standard;
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.StdId=0x200;
	
	TxMessage.Data[0]=i1 >> 8;
	TxMessage.Data[1]=i1;
	TxMessage.Data[2]=i2 >> 8;
	TxMessage.Data[3]=i2;
	TxMessage.Data[4]=i3 >> 8;
	TxMessage.Data[5]=i3;
	TxMessage.Data[6]=i4 >> 8;
	TxMessage.Data[7]=i4; 
	
	CAN_Transmit(CAN2,&TxMessage);

}

CanRxMsg Rx1Message;
void CAN1_RX0_IRQHandler(void)
{
	CAN_Receive(CAN1,CAN_FIFO0,&Rx1Message);
	switch(Rx1Message.StdId)
	{
		case 0x201: get_motor_measure(&M3508_control[0].M3508, Rx1Message);break;
		case 0x202: get_motor_measure(&M3508_control[1].M3508, Rx1Message);break;
		case 0x203: get_motor_measure(&M3508_control[2].M3508, Rx1Message);break;
		case 0x204: get_motor_measure(&M3508_control[3].M3508, Rx1Message);break;
	}

}

CanRxMsg Rx2Message;
void CAN2_RX1_IRQHandler(void)
{
	CAN_Receive(CAN2,CAN_FIFO1,&Rx2Message);
	switch(Rx2Message.StdId)
	{
		case 0x201: get_motor_measure(&M2006_control[0].M2006, Rx2Message);break;
		case 0x202: get_motor_measure(&M2006_control[1].M2006, Rx2Message);break;
		case 0x203: get_motor_measure(&M2006_control[2].M2006, Rx2Message);break;
		case 0x204: get_motor_measure(&M2006_control[3].M2006, Rx2Message);break;
	}

}
