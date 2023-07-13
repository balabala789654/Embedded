#include "p3.h"
#include "stm32f4xx.h"                  // Device header

static uint8_t head_1;
static uint8_t head_2;

extern uint8_t rec[13];

uint8_t verify_data[13];


p3 p3_remote_output(uint8_t _data[13])
{
	p3 _p3;
	_p3.ch[3]=127-_data[6];
	_p3.ch[2]=127-_data[5];
	_p3.ch[1]=127-_data[8];
	_p3.ch[0]=127-_data[7];
	
	_p3.L2 = _data[9]>>7;
	_p3.L1 = (_data[9]&0x40)>>6;
	_p3.LU = (_data[9]&0x20)>>5;
	_p3.LL = (_data[9]&0x10)>>4;
	
	_p3.LD = (_data[9]&0x8)>>3;
	_p3.LR = (_data[9]&0x4)>>2;
	_p3.SE = (_data[9]&0x2)>>1;
	_p3.ST = (_data[9]&0x00)>>0;
	
	_p3.RL = 	_data[10]>>7;
	_p3.RD = 	(_data[10]&0x40)>>6;
	_p3.RR = 	(_data[10]&0x20)>>5;
	_p3.RU = 	(_data[10]&0x10)>>4;
	            
	_p3.R1 = 	(_data[10]&0x8)>>3;
	_p3.R2 = 	(_data[10]&0x4)>>2;
	_p3.R_key = (_data[10]&0x2)>>1;
	_p3.L_key = (_data[10]&0x01)>>0;

	
	return _p3;
}

void verify_data_func(uint8_t _data[13])
{	
	head_verify(_data);
	for(int i=head_1, j=0; j<13; j++)
	{
		verify_data[j]=_data[i];
		i++;
		if(i==13) i=0;
		
	}
		
}

void head_verify(uint8_t _data[13])
{
	for(int i=0; i<13; i++)
	{
		if(_data[i]==0xAA)
		{
			head_1=i;
			if(_data[i+1]==0xAA)
			{				
				head_2=i+1;
				break;
			}
			else if(_data[12-i]==0xAA)
			{
				head_2=12-i;
			}
			else 
			{
				
			}
		}
		
	}	
}


void p3_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOA10复用为USART1
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART6, &USART_InitStructure); //初始化串口1
	
	USART_Cmd(USART6, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
	
        DMA_DeInit(DMA2_Stream1);

        DMA_InitStructure.DMA_Channel = DMA_Channel_5;
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rec;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure.DMA_BufferSize = 13;//
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA2_Stream1, &DMA_InitStructure);
        DMA_Cmd(DMA2_Stream1, DISABLE); //Add a disable  DMA_Cmd(DMA1_Stream1, DISABLE)
        DMA_Cmd(DMA2_Stream1, ENABLE);//   DMA_Cmd(DMA1_Stream1, ENABLE)
	
}

static int i;
void USART6_IRQHandler(void)
{
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		
		rec[i] = USART_ReceiveData(USART6);
		i++;
		if(i==13) 
		{
			verify_data_func(rec);
			i=0;
		}
		USART_ClearFlag(USART6, USART_FLAG_RXNE);
	}
	
}


