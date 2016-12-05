/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * ???  :spi.c
 * ??    :spi??         
 * ????:Air Nano?????
 * ???  :ST3.5.0
 * ??    :Air Nano Team 
 * ??    :http://byd2.taobao.com
**********************************************************************************/
#include "head.h"


void Spi1_Init(void)
{ 
		GPIO_InitTypeDef GPIO_InitStructure;
		SPI_InitTypeDef  SPI_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOFʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOFʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOFʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//ʹ��GPIOFʱ��

	 //
	/*?? SPI_NRF_SPI? SCK,MISO,MOSI??,GPIOA^5,GPIOA^6,GPIOA^7 */ //pa5  pa6  pa7
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13|GPIO_Pin_15|GPIO_Pin_14; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	/*??SPI_NRF_SPI?CE??,?SPI_NRF_SPI? CSN ??:*/   //ce1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_7; //ce
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_8; //csn  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	/*??SPI_NRF_SPI?IRQ??,*/  //pc5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_9; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	
	SPI_CSN_H(1);
	SPI_CSN_H(2);
	SPI_CE_H(1);
	SPI_CE_H(2);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2); //PB3����Ϊ SPI1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2); //PB4����Ϊ SPI1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2); //PB5����Ϊ SPI1
 
	//����ֻ���SPI�ڳ�ʼ��
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);//��λSPI1
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//ֹͣ��λSPI1 
       
  	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //????? 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //??? 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //????8? 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //????,????? 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //?1?????,???????? 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //NSS??????? 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //8??,9MHz 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //???? 
	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	SPI_Init(SPI2, &SPI_InitStructure); 
	/* Enable SPI1 */ 
	SPI_Cmd(SPI2, ENABLE);
}
u8 Spi_RW(u8 dat) 
{ 
	/* ? SPI?????????? */ 
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); 
	/* ?? SPI2??????? */ 
	SPI_I2S_SendData(SPI2, dat); 
	/* ?SPI?????????? */ 
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET); 
	/* Return the byte read from the SPI bus */ 
	return SPI_I2S_ReceiveData(SPI2); 
}

