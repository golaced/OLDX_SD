#ifndef __SPI_RC_H
#define __SPI_RC_H
#include "head.h"

#define SPI_CE_H1   GPIO_SetBits(GPIOB, GPIO_Pin_10) 
#define SPI_CE_L1   GPIO_ResetBits(GPIOB, GPIO_Pin_10)
#define SPI_CE_H2   GPIO_SetBits(GPIOB, GPIO_Pin_7) 
#define SPI_CE_L2   GPIO_ResetBits(GPIOB, GPIO_Pin_7)


#define SPI_CSN_H1  GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define SPI_CSN_L1  GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define SPI_CSN_H2  GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define SPI_CSN_L2  GPIO_ResetBits(GPIOB, GPIO_Pin_8)
void Spi1_Init(void);
u8 Spi_RW(u8 dat);
		 
#endif


