#ifndef __BEEP_H
#define __BEEP_H	 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//蜂鸣器驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
//蜂鸣器端口定义
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#define GPIO_BEEP GPIOA
#define PIN_BEEP GPIO_Pin_8 
#define RCC_BEEP RCC_APB2Periph_GPIOA

void BEEP_Init(void);	//初始化

void LED_Init(void);	//初始化
void RED_Init(void);	//初始化 	

#define GPIO_RED GPIOB 
#define PIN_RED1 GPIO_Pin_6
#define PIN_RED2 GPIO_Pin_7 
#define PIN_RED3 GPIO_Pin_8 
#define PIN_RED4 GPIO_Pin_9 

extern u8 RED_State,RED_Stater;

extern void RED_TASK(void);
#endif

