/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：time.c
 * 描述    ：TIM3配置         
 * 实验平台：Air Nano四轴飞行器
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team 
 * 淘宝    ：http://byd2.taobao.com
**********************************************************************************/
#include "head.h"



/**************************实现函数********************************************
*函数原型:		
*功　　能:1ms中断一次,计数器为1000		
*******************************************************************************/
void Nvic_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* NVIC_PriorityGroup */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;            //TIM2中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2 ;  //先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	
	
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
	  /*使能TIM1时钟*/
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	
	 /*自动重装载值5000*/
	  TIM_TimeBaseStructure.TIM_Period = 2000; 
	  /*预分频值，+1为分频系数*/
	  TIM_TimeBaseStructure.TIM_Prescaler =167;  
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	  /*TIM向上计数模式*/
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  /*根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位*/  
	  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
	   /*使能TIM2中断源*/
	  TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
	  /*使能TIMx外设*/
	  TIM_Cmd(TIM2, ENABLE);           
 }


void TIM2_Int_Init(u16 arr,u16 psc)
{ GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM2,ENABLE); //使能定时器3
	
	/*
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOF时钟
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);*/
}

