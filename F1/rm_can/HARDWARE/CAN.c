#include "can.h"
#include "delay.h"
#include "usart.h"
//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024;  tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M,�������CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
//������Ϊ:36M/((8+9+1)*4)=500Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��;

motor_measure_t M6020[4];
motor_measure_t M3508[4];
motor_measure_t YAW;
motor_measure_t PITCH;

u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
 	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);		//��ʼ��IO
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��IO
	  
 	//CAN��Ԫ����
 	  CAN_InitStructure.CAN_TTCM=DISABLE;						 //��ʱ�䴥��ͨ��ģʽ  //
 	  CAN_InitStructure.CAN_ABOM=DISABLE;						 //����Զ����߹���	 //
  	CAN_InitStructure.CAN_AWUM=DISABLE;						 //˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)//
  	CAN_InitStructure.CAN_NART=ENABLE;						 	//��ֹ�����Զ����� //
  	CAN_InitStructure.CAN_RFLM=DISABLE;						 //���Ĳ�����,�µĸ��Ǿɵ� // 
  	CAN_InitStructure.CAN_TXFP=DISABLE;						 //���ȼ��ɱ��ı�ʶ������ //
  	CAN_InitStructure.CAN_Mode= mode;	         //ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; //
  	//���ò�����
  	CAN_InitStructure.CAN_SJW=tsjw;				//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;            //��Ƶϵ��(Fdiv)Ϊbrp+1	//
  	CAN_Init(CAN1, &CAN_InitStructure);            // ��ʼ��CAN1 

 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
   	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
 	  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0

  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

		return 0;
	}

CanRxMsg RxMessage;
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CAN_Receive(CAN1,CAN_FIFO0,&RxMessage);
	switch(RxMessage.StdId)
	{
		case 0x201: get_motor_measure(&M3508[0], RxMessage);break;
		case 0x202: get_motor_measure(&M3508[1], RxMessage);break;
		case 0x203: get_motor_measure(&M3508[2], RxMessage);break;
		case 0x204: get_motor_measure(&M3508[3], RxMessage);break;
		case 0x205: get_motor_measure(&YAW, RxMessage);break;
		case 0x206: get_motor_measure(&PITCH, RxMessage);break;
	}
	
}

void moter_send_3508(short int i1,short int i2,short int i3,short int i4)
{
	CanTxMsg TxMessage;
	
	TxMessage.DLC=0x08;
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


	
	

	













