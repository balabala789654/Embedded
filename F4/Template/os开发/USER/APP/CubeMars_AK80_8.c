#include "CubeMars_AK80_8.h"

_ak80_8 AK80_8;

void AK80_8_set(void);

void AK80_8_init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode){
	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
   	NVIC_InitTypeDef  NVIC_InitStructure;
	
    //ʹ�����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB_GPIOx, ENABLE);//ʹ��PORTAʱ��	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB_CANx, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = CAN_GPIO_Pin_1| CAN_GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(CAN_GPIOx, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	//���Ÿ���ӳ������
	GPIO_PinAFConfig(CAN_GPIOx,CAN_GPIO_PinSource_1,GPIO_AF_CANx); //GPIOA11����ΪCAN1
	GPIO_PinAFConfig(CAN_GPIOx,CAN_GPIO_PinSource_2,GPIO_AF_CANx); //GPIOA12����ΪCAN1
	
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN_num, &CAN_InitStructure);   // ��ʼ��CAN1 
    
	//���ù�����
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
		
	
	CAN_ITConfig(CAN_num,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    

  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
	
	AK80_8_set();
	return;
}

void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
	uint8_t i=0;
	if (len > 8) len = 8;
	
	CanTxMsg TxMessage;
	TxMessage.StdId = 0;
	TxMessage.IDE = CAN_Id_Extended;
	TxMessage.ExtId = id;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = len;
	for(i=0;i<len;i++)
		TxMessage.Data[i]=data[i];
	CAN_Transmit(CAN_num, &TxMessage); //CAN �ڷ��� TxMessage ����
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
} 

void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) {
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

//ռ�ձ�ģʽ
void comm_can_set_duty(uint8_t controller_id, float duty) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
	comm_can_transmit_eid(controller_id |
							((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

/*
������ģʽ
������ֵΪ int32 ���ͣ� ��ֵ-60000-60000 ����-60-60A
*/
void comm_can_set_current(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

/*
����ɲ��ģʽ
ɲ��������ֵΪ int32 ���ͣ� ��ֵ 0-60000 ���� 0-60A
*/
void comm_can_set_cb(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

/*
�ٶȻ�ģʽ
�ٶ���ֵΪ int32 �ͣ� ��Χ-100000-100000 ����-100000-100000 ����ת��
*/
void comm_can_set_rpm(uint8_t controller_id, float rpm) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}


/*
λ�û�ģʽ
λ��Ϊ int32 �ͣ� ��Χ-360000000-360000000 ����λ��-36000�� ~36000��
*/
void comm_can_set_pos(uint8_t controller_id, float pos) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

//����ԭ��ģʽ
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode) {
	int32_t send_index = 0;
	uint8_t buffer;
	buffer=set_origin_mode;	//����ָ��Ϊ uint8_t �ͣ� 0 ����������ʱԭ��(�ϵ�����)�� 1 ���������������(�����Զ�����);
	comm_can_transmit_eid(controller_id |
						((uint32_t) CAN_PACKET_SET_ORIGIN_HERE << 8), &buffer, send_index);
}

/*
λ���ٶȻ�ģʽ
���У� λ��Ϊ int32 �ͣ� ��Χ-360000000~360000000 ��Ӧ-λ��-36000�� ~36000�� 
���У� �ٶ�Ϊ int16 �ͣ� ��Χ-32768~32767 ��Ӧ-327680~-327680 ����ת��
���У� ���ٶ�Ϊ int16 �ͣ� ��Χ 0~32767�� ��Ӧ 0~327670�� 1 ��λ���� 10 ����ת��/s2
*/
void comm_can_set_pos_spd(uint8_t controller_id, float pos,int16_t spd, int16_t RPA ) {
	int32_t send_index = 0;
	int16_t send_index1 = 4;
	uint8_t buffer[8];
	buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
	buffer_append_int16(buffer,spd/10.0, & send_index1);
	buffer_append_int16(buffer,RPA/10.0, & send_index1);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index1);
}


void CAN1_RX0_IRQHandler(void){
	CanRxMsg Rx1Message;
	CAN_Receive(CAN_num,CAN_FIFO0,&Rx1Message);
	int16_t pos_int = (Rx1Message).Data[0] << 8 | (Rx1Message).Data[1];
	int16_t spd_int = (Rx1Message).Data[2] << 8 | (Rx1Message).Data[3];
	int16_t cur_int = (Rx1Message).Data[4] << 8 | (Rx1Message).Data[5];
	AK80_8.cur_position = (float)( pos_int * 0.1f); //���λ��
	AK80_8.cur_speed = (float)( spd_int * 10.0f);//����ٶ�
	AK80_8.cur_curenty = (float) ( cur_int * 0.01f);//�������
	AK80_8.cur_temperature = Rx1Message.Data[6] ;//����¶�
	AK80_8.error_code = Rx1Message.Data[7] ;//���������
}


//void CAN2_TX_IRQHandler(void){
//	int i;
//	i++;
//}

void AK80_8_set(void){
	comm_can_set_origin(motor_id, 0);
}


int speed=-100;
float position = 0;

void AK80_8_control(void){
	comm_can_set_rpm(motor_id, speed);
	//comm_can_set_pos(motor_id, position);
}


