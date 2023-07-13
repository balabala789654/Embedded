#include "as5047.h"


int read_as5047_L(void)
{
	int angle;
	int send_data=0x3fff;
	int data;
	
	send_data |= 0x4000;
	send_data |= (parity_even(send_data)<<15);
	data = SPI2_ReadWriteByte(send_data);
	angle = data & 0x3fff;	
	
	return angle;	
}

int read_as5047_R(void)
{
	int angle;
	int send_data=0x3fff;
	int data;
	
	send_data |= 0x4000;
	send_data |= (parity_even(send_data)<<15);
	data = SPI3_ReadWriteByte(send_data);
	angle = data & 0x3fff;	
	
	return angle;	
}

uint8_t parity_even(uint16_t v)
{
      if(v == 0) return 0;


      v ^= v >> 8;
      v ^= v >> 4;
      v ^= v >> 2;
      v ^= v >> 1;
      return v & 1;
}

unsigned int parity(unsigned x)
{
	unsigned int parity =0;
	while(x != 0)
	{
		parity ^=x;
		x >>= 1;
	}
	return (parity & 0x1);
}
