#ifndef __SPI_H
#define __SPI_H
#include "sys.h"


#define CMD_ANGLE          0x3FFF  
#define CMD_READ_MAG       0x3FFE 
#define CMD_READ_DIAG      0x3FFD  
#define CMD_NOP            0x0000  
#define CMD_CLEAR_ERROR    0x0001  
#define CMD_ProgramControl 0x0003
#define CMD_OTPHigh 0x0016
#define CMD_OTPLow 0x0017



//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//SPI ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
 	    					
void SPI2_Init(void);							
void SPI3_Init(void);			 //��ʼ��SPI1��

void SPI1_SetSpeed(u8 SpeedSet); //����SPI1�ٶ�   

uint16_t SPI3_ReadWriteByte(uint16_t TxData);
uint16_t SPI2_ReadWriteByte(uint16_t TxData);

uint8_t parity_even(uint16_t v);
uint16_t Read_As5048A_Reg(uint16_t cmd);

unsigned int parity(unsigned x);


#endif

