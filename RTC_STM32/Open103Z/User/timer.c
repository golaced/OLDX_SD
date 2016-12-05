#include "head.h"



//定时器配置并开启 使用定时器3
void Time4ON(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		//开启定时器外设时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		//配置定时器参数
		TIM_DeInit(TIM4); 
		TIM_TimeBaseStructure.TIM_Period = 1000; 								 	//1ms定时			 
		TIM_TimeBaseStructure.TIM_Prescaler = (72000000/1000000 - 1);              
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);	
		//中断配置
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //抢占优先级2 低优先级别中断 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  	 //响应优先级0 高级别的响应中断
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
		NVIC_Init(&NVIC_InitStructure);	  
		//开中断
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);					  
		TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); 
		//开启定时器			 
		TIM_Cmd(TIM4, ENABLE); 
}

//定时器中断处理 从stm32f10x_it.c添加

u8 test_itoa[3];
void Time4_IntHandle(void)
{ static u8 cnt_2ms=0,cnt_10ms=0,cnt_5s;
	u8 i;
	static u16 pic_cnt=0;u8 time_now_char[15]={0};
		//清中断标识
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
		//---------------- 中断处理  ---------------------
		{
			if(cnt_2ms++>2)
			{cnt_2ms=0;
	     
				// if(dma_can_tx && uart_test[0])
				//	{USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
				//		DMA_Cmd(DMA1_Channel7, ENABLE);//使能DMA通道，开始传输数
				//		dma_can_tx=0;}
			////GIF_SHOW_IRQ("gif","badapple",pic_cnt++);if(pic_cnt>=866)pic_cnt=0;		
			//GIF_SHOW_IRQ("gif","dongfang1",pic_cnt++);if(pic_cnt>=387)pic_cnt=0;		
			}//--end of 2ms task
			
			if(cnt_10ms++>100)
			{cnt_10ms=0;
      Ds1302_Read_Time();	
				Send_Card_Info();
			}
		}
}



