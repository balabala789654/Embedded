#include "CubeMars_AK80_8.h"

_ak80_8 AK80_8;

void AK80_8_set(void);

void AK80_8_init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode){
	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
   	NVIC_InitTypeDef  NVIC_InitStructure;
	
    //使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB_GPIOx, ENABLE);//使能PORTA时钟	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB_CANx, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	GPIO_InitStructure.GPIO_Pin = CAN_GPIO_Pin_1| CAN_GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(CAN_GPIOx, &GPIO_InitStructure);//初始化PA11,PA12
	
	//引脚复用映射配置
	GPIO_PinAFConfig(CAN_GPIOx,CAN_GPIO_PinSource_1,GPIO_AF_CANx); //GPIOA11复用为CAN1
	GPIO_PinAFConfig(CAN_GPIOx,CAN_GPIO_PinSource_2,GPIO_AF_CANx); //GPIOA12复用为CAN1
	
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN_num, &CAN_InitStructure);   // 初始化CAN1 
    
	//配置过滤器
 	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		
	
	CAN_ITConfig(CAN_num,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    

  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
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
	CAN_Transmit(CAN_num, &TxMessage); //CAN 口发送 TxMessage 数据
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

//占空比模式
void comm_can_set_duty(uint8_t controller_id, float duty) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
	comm_can_transmit_eid(controller_id |
							((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

/*
电流环模式
电流数值为 int32 类型， 数值-60000-60000 代表-60-60A
*/
void comm_can_set_current(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

/*
电流刹车模式
刹车电流数值为 int32 类型， 数值 0-60000 代表 0-60A
*/
void comm_can_set_cb(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

/*
速度环模式
速度数值为 int32 型， 范围-100000-100000 代表-100000-100000 电气转速
*/
void comm_can_set_rpm(uint8_t controller_id, float rpm) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}


/*
位置环模式
位置为 int32 型， 范围-360000000-360000000 代表位置-36000° ~36000°
*/
void comm_can_set_pos(uint8_t controller_id, float pos) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
	comm_can_transmit_eid(controller_id |
						((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

//设置原点模式
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode) {
	int32_t send_index = 0;
	uint8_t buffer;
	buffer=set_origin_mode;	//设置指令为 uint8_t 型， 0 代表设置临时原点(断电消除)， 1 代表设置永久零点(参数自动保存);
	comm_can_transmit_eid(controller_id |
						((uint32_t) CAN_PACKET_SET_ORIGIN_HERE << 8), &buffer, send_index);
}

/*
位置速度环模式
其中， 位置为 int32 型， 范围-360000000~360000000 对应-位置-36000° ~36000° 
其中， 速度为 int16 型， 范围-32768~32767 对应-327680~-327680 电气转速
其中， 加速度为 int16 型， 范围 0~32767， 对应 0~327670， 1 单位等于 10 电气转速/s2
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
	AK80_8.cur_position = (float)( pos_int * 0.1f); //电机位置
	AK80_8.cur_speed = (float)( spd_int * 10.0f);//电机速度
	AK80_8.cur_curenty = (float) ( cur_int * 0.01f);//电机电流
	AK80_8.cur_temperature = Rx1Message.Data[6] ;//电机温度
	AK80_8.error_code = Rx1Message.Data[7] ;//电机故障码
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


