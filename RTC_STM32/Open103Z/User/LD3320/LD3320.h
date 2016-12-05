#ifndef __LD3320_H
#define	__LD3320_H

#include "stm32f10x.h"
#include <stdio.h>
#include "usart_config.h"
#include "LD3320_config.h"
#include "stm32f10x_it.h"

#define uint8 unsigned char
#define uint16 unsigned int
#define uint32 unsigned long

///��������״̬����������¼������������ASRʶ����������MP3����
#define LD_MODE_IDLE			0x00
#define LD_MODE_ASR_RUN		0x08
#define LD_MODE_MP3		 		0x40

///�������״̬����������¼������������ASRʶ������е��ĸ�״̬
#define LD_ASR_NONE					0x00	//��ʾû������ASRʶ��
#define LD_ASR_RUNING				0x01	//��ʾLD3320������ASRʶ����
#define LD_ASR_FOUNDOK			0x10	//��ʾһ��ʶ�����̽�������һ��ʶ����
#define LD_ASR_FOUNDZERO 		0x11	//��ʾһ��ʶ�����̽�����û��ʶ����
#define LD_ASR_ERROR	 			0x31	//	��ʾһ��ʶ��������LD3320оƬ�ڲ����ֲ���ȷ��״̬

#define CLK_IN   					22/* user need modify this value according to clock in */
#define LD_PLL_11					(uint8)((CLK_IN/2.0)-1)
#define LD_PLL_MP3_19			0x0f
#define LD_PLL_MP3_1B			0x18
#define LD_PLL_MP3_1D   	(uint8)(((90.0*((LD_PLL_11)+1))/(CLK_IN))-1)

#define LD_PLL_ASR_19 		(uint8)(CLK_IN*32.0/(LD_PLL_11+1) - 0.51)
#define LD_PLL_ASR_1B 		0x48
#define LD_PLL_ASR_1D 		0x1f

#define MIC_VOL 0x4c		//��˷����� //origin 43
#define MIC_SEN 0xf		//��˷����� //origin 43

///�û��޸ĺ���
void  LD3320_main(void);
static uint8 LD_AsrAddFixed(void);
static void Board_text(uint8 Code_Val);
static void Delayms(uint16 i);
static void Glide_LED(void);
static void Flicker_LED(void);
static void Key_LED(void);
static void Off_LED(void);
static void Jt_LED(void);

///��س�ʼ��
extern void LD3320_init(void);
static void LD3320_GPIO_Cfg(void);
static void LD3320_EXTI_Cfg(void);
static void LD3320_SPI_cfg(void);


///�м��
static void LD3320_delay(unsigned long uldata);
static uint8 RunASR(void);
static void LD_reset(void);
static void LD_AsrStart(void);
static uint8 LD_Check_ASRBusyFlag_b2(void);

///�Ĵ�������
static uint8 spi_send_byte(uint8 byte);
static void LD_WriteReg(uint8 data1,uint8 data2);
static uint8 LD_ReadReg(uint8 reg_add);
static uint8 LD_GetResult(void);
static uint8 LD_AsrRun(void);
static void ProcessInt(void);
static void LD_Init_Common(void);
static void LD_Init_ASR(void);

#endif /*__LD3320_H */
