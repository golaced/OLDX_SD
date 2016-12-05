/**
  ******************************************************************************
  * @file     SPI1.h 
  * @author   xukai
  * @version  V0.1
  * @date     2012-3-29
  * @brief    ��ʼ��SPI1��SPI1��д��������       
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI1_H
#define __SPI1_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//��������
void SPI1_Config(void);
u8 SPI1_SendByte(uint8_t byte);
u8 SPI1_ReceiveByte(void);
#endif
/***************************************************************END OF FILE****/
