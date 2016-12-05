#ifndef __DS1302_H
#define __DS1302_H
#include "stm32f4xx.h"


//sbit SCK=P1^4;		
//sbit SDA=P1^5;		
//sbit RST=P1^6;
//��λ��
#define P_RST GPIO_Pin_11
#define P_DAT GPIO_Pin_13
#define P_CLK GPIO_Pin_14
#define RST_CLR	GPIO_ResetBits(GPIOE,P_RST)//RST=0//��ƽ�õ�
#define RST_SET	GPIO_SetBits(GPIOE,P_RST)//��ƽ�ø�

//˫������
#define IO_CLR	GPIO_ResetBits(GPIOE,P_DAT)//SDA=0//��ƽ�õ�
#define IO_SET	GPIO_SetBits(GPIOE,P_DAT)//SDA=1//��ƽ�ø�
#define IO_R	GPIO_ReadInputDataBit(GPIOE,P_DAT)//SDA  //��ƽ��ȡ


//ʱ���ź�
#define SCK_CLR	GPIO_ResetBits(GPIOE,P_CLK)//SCK=0//ʱ���ź�
#define SCK_SET	GPIO_SetBits(GPIOE,P_CLK)//SCK=1//��ƽ�ø�


#define ds1302_sec_add			0x80		//�����ݵ�ַ
#define ds1302_min_add			0x82		//�����ݵ�ַ
#define ds1302_hr_add			0x84		//ʱ���ݵ�ַ
#define ds1302_date_add			0x86		//�����ݵ�ַ
#define ds1302_month_add		0x88		//�����ݵ�ַ
#define ds1302_day_add			0x8a		//�������ݵ�ַ
#define ds1302_year_add			0x8c		//�����ݵ�ַ
#define ds1302_control_add		0x8e		//�������ݵ�ַ
#define ds1302_charger_add		0x90 					 
#define ds1302_clkburst_add		0xbe

//extern unsigned char time_buf1[8];//��������ʱ������
//extern unsigned char time_buf[8] ;//��������ʱ������

typedef struct
{
	unsigned char year;
	unsigned char month;
	unsigned char day;
	unsigned char week;
	unsigned char hour;
	unsigned char min;
	unsigned char sec;
}TimeStruct;

//extern Timestruct Writetime;
extern TimeStruct Readtime;
/*------------------------------------------------
           ��DS1302д��һ�ֽ�����
------------------------------------------------*/
void Ds1302_Write_Byte(unsigned char addr, unsigned char d);
/*------------------------------------------------
           ��DS1302����һ�ֽ�����
------------------------------------------------*/
unsigned char Ds1302_Read_Byte(unsigned char addr) ;
/*------------------------------------------------
           ��DS1302д��ʱ������
------------------------------------------------*/
void Ds1302_Write_Time(void) ;
/*------------------------------------------------
           ��DS1302����ʱ������
------------------------------------------------*/
void Ds1302_Read_Time(void)  ;
/*------------------------------------------------
                DS1302��ʼ��
------------------------------------------------*/
void Ds1302_Init(void);
extern void Time_Conver_char(u8 sel,u16 loc,u8 * s);

#endif