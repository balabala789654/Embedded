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
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//SPI 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
 	    					
void SPI2_Init(void);							
void SPI3_Init(void);			 //初始化SPI1口

void SPI1_SetSpeed(u8 SpeedSet); //设置SPI1速度   

uint16_t SPI3_ReadWriteByte(uint16_t TxData);
uint16_t SPI2_ReadWriteByte(uint16_t TxData);

uint8_t parity_even(uint16_t v);
uint16_t Read_As5048A_Reg(uint16_t cmd);

unsigned int parity(unsigned x);


#endif

