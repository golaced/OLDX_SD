#ifndef __BEEP_H
#define __BEEP_H	 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//��������������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
//�������˿ڶ���
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#define GPIO_BEEP GPIOA
#define PIN_BEEP GPIO_Pin_8 
#define RCC_BEEP RCC_APB2Periph_GPIOA

void BEEP_Init(void);	//��ʼ��

void LED_Init(void);	//��ʼ��
void RED_Init(void);	//��ʼ�� 	

#define GPIO_RED GPIOB 
#define PIN_RED1 GPIO_Pin_6
#define PIN_RED2 GPIO_Pin_7 
#define PIN_RED3 GPIO_Pin_8 
#define PIN_RED4 GPIO_Pin_9 

extern u8 RED_State,RED_Stater;

extern void RED_TASK(void);
#endif

