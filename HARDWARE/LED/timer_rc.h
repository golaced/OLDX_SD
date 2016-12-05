#ifndef __TIME_H__
#define __TIME_H__
#include "stm32f4xx.h" 

#define EnTIM2()  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE)
void Nvic_Init(void);
void TIM2_Config(void);
void TIM2_Int_Init(u16 arr,u16 psc);
#endif 
