#include "head.h"
void OPEN_KEY_init(void)
{
 		NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //ʹ��GPIOB�˿�ʱ��
 
 GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13;				 //BEEP-->PB.8 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //�ٶ�Ϊ50MHz
 GPIO_Init(GPIOC, &GPIO_InitStructure);	 //���ݲ�����ʼ��GPIOB.8
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//ʹ�ܸ��ù���ʱ��
 
  //GPIOE.2 �ж����Լ��жϳ�ʼ������   �½��ش���
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource13);
 
  	EXTI_InitStructure.EXTI_Line=EXTI_Line13;	//KEY2
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
 
 //GPIOE.3	  �ж����Լ��жϳ�ʼ������ �½��ش��� //KEY1
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource13);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line13;
  	EXTI_Init(&EXTI_InitStructure);	  	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//ʹ�ܰ���KEY2���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//��ռ���ȼ�2�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;					//�����ȼ�2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);
}

void EXTI15_10_IRQHandler(void)
{ u8 key=Get_open_key;
	delay_ms(10);//����
	if(key==0)	 //����KEY0
	{  if(open_lock1==0)
	      {open_lock1=1;
					
				OLED_Fill(0x00);OLED_P8x16Str(30,3,"See You!");	
				}	
	}		 
	EXTI_ClearITPendingBit(EXTI_Line13);  //���LINE4�ϵ��жϱ�־λ  
}   


void LED_init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��GPIOB�˿�ʱ��
 
 GPIO_InitStructure.GPIO_Pin =GPIO_Pin_9|GPIO_Pin_8;				 //BEEP-->PB.8 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //�ٶ�Ϊ50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	 //���ݲ�����ʼ��GPIOB.8

 


LED_RED_ON;
LED_GREEN_ON;
	
delay_ms(1000);
LED_GREEN_OFF;	
}
void Lock_init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��GPIOB�˿�ʱ��
 
 GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1;				 //BEEP-->PB.8 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //�ٶ�Ϊ50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	 //���ݲ�����ʼ��GPIOB.8
OPEN_KEY_init();	
LED_init();
LOCK1_CLOSE;
LOCK2_CLOSE;
}

void Lock_tast(u8 lock , u8 state)
{
 
switch(lock){
	case 1:
		if(!state)
			LOCK1_CLOSE;
	  else 
			LOCK1_OPEN;
	break;
		case 2:
		if(!state)
			LOCK2_CLOSE;
	  else 
			LOCK2_OPEN;
	break;
	}		
}

void Lock_Task(void)
{
 static u8 state=0;
 static u16 cnt;
switch(state){
	case 0:
		if(open_lock1==1)
		 {LOCK1_OPEN;LOCK2_OPEN;LED_GREEN_ON;LED_RED_OFF;state=1;cnt=0;}
		else
		{LOCK1_CLOSE;LOCK2_CLOSE;}
	break;
	case 1:
      if(cnt++>3500)
			{cnt=0;TIM_Cmd(TIM4, DISABLE); LOCK1_CLOSE;LOCK2_CLOSE;	LED_GREEN_OFF;LED_RED_ON;	delay_ms(7000);OLED_Init();open_lock1=0;state=0; OLED_Fill(0x00);OLED_P8x16Str(43,3,"Welcome!");OLED_P6x8Str(3,0,"By Golaced-BIT ");	TIM_Cmd(TIM4, ENABLE);}
			else
			 delay_ms(1);
	break;
	}		
}