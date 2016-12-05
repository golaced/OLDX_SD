#include "head.h"



//��ʱ�����ò����� ʹ�ö�ʱ��3
void Time4ON(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		//������ʱ������ʱ��
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		//���ö�ʱ������
		TIM_DeInit(TIM4); 
		TIM_TimeBaseStructure.TIM_Period = 1000; 								 	//1ms��ʱ			 
		TIM_TimeBaseStructure.TIM_Prescaler = (72000000/1000000 - 1);              
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);	
		//�ж�����
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�2 �����ȼ����ж� 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  	 //��Ӧ���ȼ�0 �߼������Ӧ�ж�
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
		NVIC_Init(&NVIC_InitStructure);	  
		//���ж�
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);					  
		TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); 
		//������ʱ��			 
		TIM_Cmd(TIM4, ENABLE); 
}

//��ʱ���жϴ��� ��stm32f10x_it.c���

u8 test_itoa[3];
void Time4_IntHandle(void)
{ static u8 cnt_2ms=0,cnt_10ms=0,cnt_5s;
	u8 i;
	static u16 pic_cnt=0;u8 time_now_char[15]={0};
		//���жϱ�ʶ
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
		//---------------- �жϴ���  ---------------------
		{
			if(cnt_2ms++>2)
			{cnt_2ms=0;
	     
				// if(dma_can_tx && uart_test[0])
				//	{USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
				//		DMA_Cmd(DMA1_Channel7, ENABLE);//ʹ��DMAͨ������ʼ������
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



