/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * ???  :nrf24l01.c
 * ??    :nrf24l01??         
 * ????:Air Nano?????
 * ???  :ST3.5.0
 * ??    :Air Nano Team 
 * ??    :http://byd2.taobao.com
**********************************************************************************/

#include "head.h"

uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];//nrf24l01??????
uint8_t NRF24L01_TXDATA[RX_PLOAD_WIDTH];//nrf24l01???????
u8  TX_ADDRESS[TX_ADR_WIDTH]= {0xE1,0xE2,0xE3,0xE4,0xE5};	//????
u8  RX_ADDRESS[RX_ADR_WIDTH]= {0xE1,0xE2,0xE3,0xE4,0xE5};	//????
/*
*****************************************************************
* ????
*****************************************************************
*/
void SPI_CE_H(u8 sel)  
{ if(sel==1)
	{GPIO_SetBits(GPIOB, GPIO_Pin_10) ;}
	else if(sel==2)GPIO_SetBits(GPIOB, GPIO_Pin_7) ;
}
void SPI_CE_L(u8 sel)  
{ if(sel==1)
	GPIO_ResetBits(GPIOB, GPIO_Pin_10) ;
	else if(sel==2)GPIO_ResetBits(GPIOB, GPIO_Pin_7) ;
}
void SPI_CSN_H(u8 sel)  
{ if(sel==1)
	GPIO_SetBits(GPIOB, GPIO_Pin_12) ;
	else if(sel==2)GPIO_SetBits(GPIOB, GPIO_Pin_8) ;
}
void SPI_CSN_L(u8 sel)  
{ if(sel==1)
	GPIO_ResetBits(GPIOB, GPIO_Pin_12) ;
	else if(sel==2)GPIO_ResetBits(GPIOB, GPIO_Pin_8) ;
}
//---
void SPI_CE_Ht(u8 sel)  
{ if(sel==2)
	GPIO_SetBits(GPIOB, GPIO_Pin_10) ;
	else if(sel==1)GPIO_SetBits(GPIOB, GPIO_Pin_7) ;
}
void SPI_CE_Lt(u8 sel)  
{ if(sel==2)
	GPIO_ResetBits(GPIOB, GPIO_Pin_10) ;
	else if(sel==1)GPIO_ResetBits(GPIOB, GPIO_Pin_7) ;
}
void SPI_CSN_Ht(u8 sel)  
{ if(sel==2)
	GPIO_SetBits(GPIOB, GPIO_Pin_12) ;
	else if(sel==1)GPIO_SetBits(GPIOB, GPIO_Pin_8) ;
}
void SPI_CSN_Lt(u8 sel)  
{ if(sel==2)
	GPIO_ResetBits(GPIOB, GPIO_Pin_12) ;
	else if(sel==1)GPIO_ResetBits(GPIOB, GPIO_Pin_8) ;
}	

uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value,u8 sel)
{
	uint8_t status;
	SPI_CSN_Ht(sel);		
	SPI_CSN_L(sel);					  /* ???? */
	status = Spi_RW(reg);  /* ?????? */
	Spi_RW(value);		  /* ??? */
	SPI_CSN_H(sel);					  /* ????? */
  return 	status;
}
/*
*****************************************************************
* ????
*****************************************************************
*/
uint8_t NRF_Read_Reg(uint8_t reg,u8 sel)
{
	uint8_t reg_val;
	SPI_CSN_Ht(sel);	
	SPI_CSN_L(sel);					  /* ???? */
	Spi_RW(reg);			  /* ?????? */
	reg_val = Spi_RW(0);	  /* ?????????? */
	SPI_CSN_H(sel);					  /* ????? */
    return 	reg_val;
}
/*
*****************************************************************
*
* ????
*
*****************************************************************
*/
uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars,u8 sel)
{
	uint8_t i;
	uint8_t status;
	SPI_CSN_Ht(sel);	
	SPI_CSN_L(sel);				        /* ???? */
	status = Spi_RW(reg);	/* ?????? */
	for(i=0; i<uchars; i++)
	{
		Spi_RW(pBuf[i]);		/* ??? */
	}
	SPI_CSN_H(sel);						/* ????? */
    return 	status;	
}
/*
*****************************************************************
* ????
*****************************************************************
*/
uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars,u8 sel)
{
	uint8_t i;
	uint8_t status;
	SPI_CSN_Ht(sel);	
	SPI_CSN_L(sel);						/* ???? */
	status = Spi_RW(reg);	/* ?????? */
	for(i=0; i<uchars; i++)
	{
		pBuf[i] = Spi_RW(0); /* ?????? */ 	
	}
	SPI_CSN_H(sel);						/* ????? */
    return 	status;
}
/*
*****************************************************************
* ????
*****************************************************************
*/
void NRF_TxPacket(uint8_t * tx_buf, uint8_t len,u8 sel)
{	SPI_CE_Ht(sel);	
	SPI_CE_L(sel);		 //StandBy I??	
	
	NRF_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH, sel); // ???????
	NRF_Write_Buf(WR_TX_PLOAD, tx_buf, len, sel); 			 // ????	
	SPI_CE_H(sel);		 //??CE,??????
}
void NRF_TxPacket_AP(uint8_t * tx_buf, uint8_t len,u8 sel)
{	SPI_CE_Ht(sel);	
	SPI_CE_L(sel);		 //StandBy I??	
	NRF_Write_Buf(0xa8, tx_buf, len, sel); 			 // ????
	SPI_CE_H(sel);		 //??CE
}
u8 Nrf24l01_Check(u8 sel)
{ 
	u8 buf1[5]; 
	u8 i; 
	/*??5??????. */ 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,5, sel); 
	/*??????? */ 
	NRF_Read_Buf(TX_ADDR,buf1,5, sel); 
	/*??*/ 
	for(i=0;i<5;i++) 
	{ 
		if(buf1[i]!=TX_ADDRESS[i]) 
			break; 
	} 
	if(i==5)
		return SUCCESS ; //MCU?NRF???? 
	else
		return ERROR ; //MCU?NRF????? 
}


/*====================================================================================================*/
/*====================================================================================================*
**?? : NRF24L01_RxPacket
**?? : ??NRF24L01??????
**?? : txbuf:????????
**?? : 0,????;??,????
**?? : None
**====================================================================================================*/
/*====================================================================================================*/
#define STATUS          0x07  //?????;bit0:TX FIFO???;bit3:1,???????(??:6);bit4,???????
#define WRITE_REG_NRF       0x20  //??????,?5???????
#define RX_OK   		0x40  //???????
u8 NRF24L01_RxPacket(u8 *rxbuf,u8 sel)
{
	u8 sta;		    							   

	sta=NRF_Read_Reg(STATUS,sel);  //?????????    	 
	NRF_Write_Reg(WRITE_REG_NRF+STATUS,sta, sel); //??TX_DS?MAX_RT????
	if(sta&RX_OK)//?????
	{
		NRF_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH, sel);//????
		NRF_Write_Reg(FLUSH_RX,0xff, sel);//??RX FIFO??? 
		return 0; 
	}	   
	return 1;//???????
}		


void Nrf24l01_Init(u8 model, u8 ch,u8 sel)
{SPI_CE_Ht(sel);	
	SPI_CE_L(sel);
	NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH, sel);	//?RX???? 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH, sel); 		//?TX????  
	NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01, sel); 													//????0????? 
	NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01, sel);											//????0????? 
	NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a, sel);											//??????????:500us;????????:10? 
	NRF_Write_Reg(NRF_WRITE_REG+RF_CH,ch, sel);														//??RF???CHANAL
	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f, sel); 											//??TX????,0db??,2Mbps,???????
//NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07); 										  //??TX????,0db??,1Mbps,???????
/////////////////////////////////////////////////////////
	if(model==1)				//RX
	{
		NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH, sel);								//????0??????? 
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f, sel);   		 // IRQ????????,16?CRC,???
	}
	else if(model==2)		//TX
	{
		NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH, sel);								//????0??????? 
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e, sel);   		 // IRQ????????,16?CRC,???
	}
	else if(model==3)		//RX2	???
	{
		NRF_Write_Reg(FLUSH_TX,0xff, sel);
		NRF_Write_Reg(FLUSH_RX,0xff, sel);
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f, sel);   		 // IRQ????????,16?CRC,???
		
		Spi_RW(0x50);
		Spi_RW(0x73);
		NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01, sel);
		NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x07, sel);
	}
	else								//TX2	???
	{
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e, sel);   		 // IRQ????????,16?CRC,???
		NRF_Write_Reg(FLUSH_TX,0xff, sel);
		NRF_Write_Reg(FLUSH_RX,0xff, sel);
		
		Spi_RW(0x50);
		Spi_RW(0x73);
		NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01, sel);
		NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x07, sel);
	}
	SPI_CE_H(sel);
}
////////////////////////////////////////////////////////////////////////////////
