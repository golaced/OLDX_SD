/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��time.c
 * ����    ��TIM3����         
 * ʵ��ƽ̨��Air Nano���������
 * ��汾  ��ST3.5.0
 * ����    ��Air Nano Team 
 * �Ա�    ��http://byd2.taobao.com
**********************************************************************************/
#include "head.h"



/**************************ʵ�ֺ���********************************************
*����ԭ��:		
*��������:1ms�ж�һ��,������Ϊ1000		
*******************************************************************************/
void Nvic_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* NVIC_PriorityGroup */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;            //TIM2�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2 ;  //��ռ���ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //�����ȼ�3��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
	
	
	/* Enable the USARTy Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	//nav 
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	//fly 
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
}

void TIM2_Config(void)
 {
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  /*ʹ��TIM1ʱ��*/
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	
	 /*�Զ���װ��ֵ5000*/
	  TIM_TimeBaseStructure.TIM_Period = 2000; 
	  /*Ԥ��Ƶֵ��+1Ϊ��Ƶϵ��*/
	  TIM_TimeBaseStructure.TIM_Prescaler =167;  
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	  /*TIM���ϼ���ģʽ*/
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  /*����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ*/  
	  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
	   /*ʹ��TIM2�ж�Դ*/
	  TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
	  /*ʹ��TIMx����*/
	  TIM_Cmd(TIM2, ENABLE);           
 }


void TIM2_Int_Init(u16 arr,u16 psc)
{ GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///ʹ��TIM3ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��3
	
	/*
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOFʱ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);*/
}

