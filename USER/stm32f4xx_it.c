/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "head.h"
u16 fame=0;
float int_2ms;
u8 save_num=6;//10;
volatile uint32_t lastUpdate2ms, now2ms; // 采样周期计数 单位 us
 void TIM2_IRQHandler(void)//2ms_intrupt
{ static u8 flag_int=0;
	static u16 ms8=0,ms40=0,ms6=0,ms_sd=0,ms1000;	//中断次数计数器
	
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //溢出中断
	{		
	//TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //清除中断标志位
		
		 now2ms = micros();  //读取时间
  if(now2ms<lastUpdate2ms){ //定时器溢出过了。
  int_2ms =  ((float)(now2ms + (0xffff- lastUpdate2ms)) / 1000000.0f);
  }
  else	{
  int_2ms =  ((float)(now2ms - lastUpdate2ms) / 1000000.0f);
  }
  lastUpdate2ms = now2ms;	//更新时间
	
		IWDG_Feed();//喂狗
		ms8++;ms40++;ms6++;
    if(ms_sd++>save_num)//10MS 100HZ
		{
		ms_sd=0;
			if(!en_save&&sdcard_force_out)
			sdcard_force_out=0;
			else if(sdcard_force_out)
			en_save=0;
		SD_Save_Task(0);
		}			
//	 if(nrf_right) 	 {
//		Nrf_Check_Event1();//2ms
//		Nrf_Check_Event2();//2ms
//   
//		RC_Send_Task();}
	 
	 
	if(ms1000++>500)
	{ms1000=0;
	//Ds1302_Read_Time();	
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //清除中断标志位
	}
}
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
 
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
