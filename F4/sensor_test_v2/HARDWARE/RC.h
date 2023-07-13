#ifndef __RC_H
#define __RC_H

#include "sys.h"
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)

#define RC_FRAME_LENGTH 18u
#define SBUS_RX_BUF_NUM 36u

#define rc_DMAx_Streamx							DMA1_Stream1
#define rc_DMA_Channel_x						DMA_Channel_4
#define rc_USARTx										USART3
#define rc_USARTx_IRQn							USART3_IRQn
#define rc_GPIO_AF_USARTx  					GPIO_AF_USART3


extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);

#endif



