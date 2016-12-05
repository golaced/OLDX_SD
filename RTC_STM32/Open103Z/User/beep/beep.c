#include "head.h"

//��ʼ��PB8Ϊ�����.��ʹ������ڵ�ʱ��		    
//��������ʼ��
void BEEP_Init(void)
{
 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_BEEP| RCC_APB2Periph_AFIO, ENABLE);	 //ʹ��GPIOB�˿�ʱ��
 
 GPIO_InitStructure.GPIO_Pin = PIN_BEEP;				 //BEEP-->PB.8 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //�ٶ�Ϊ50MHz
 GPIO_Init(GPIO_BEEP, &GPIO_InitStructure);	 //���ݲ�����ʼ��GPIOB.8
 
 //GPIO_SetBits(GPIO_BEEP,PIN_BEEP);//���0���رշ��������

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	//ʹ�ܶ�ʱ��4ʱ��
	 
   //��ʼ��TIM4
	TIM_TimeBaseStructure.TIM_Period = 499; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =71; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM4 Channel3 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //TIM1_OutputNStateѡ�񻥲�����Ƚ�״̬
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; //TIM1_OutputNStateѡ�񻥲�����Ƚ�״̬
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC3
	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
  TIM_CtrlPWMOutputs(TIM1, ENABLE); 
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM4
	TIM_SetCompare1(TIM1,240);
}

void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO, ENABLE);	 //ʹ��GPIOB�˿�ʱ��
 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;				 //BEEP-->PB.8 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //�ٶ�Ϊ50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);	 //���ݲ�����ʼ��GPIOB.8
 
 GPIO_SetBits(GPIOA,GPIO_Pin_11);//���0���رշ��������
 GPIO_SetBits(GPIOA,GPIO_Pin_12);//���0���رշ��������
}

u8 RED_State=0,RED_Stater=0;;
void RED_Init(void)
{
 GPIO_InitTypeDef GPIO_InitStructure;
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO, ENABLE);	 //ʹ��GPIOB�˿�ʱ��
 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;				 //BEEP-->PB.8 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //�ٶ�Ϊ50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	 //���ݲ�����ʼ��GPIOB.8
}

void GET_RED(void)
{
if(GPIO_ReadInputDataBit(GPIOB,PIN_RED1))
RED_State=RED_State|SETBIT0;
else
RED_State=RED_State&CLRBIT0;	
	
if(GPIO_ReadInputDataBit(GPIOB,PIN_RED2))
RED_State=RED_State|SETBIT1;
else
RED_State=RED_State&CLRBIT1;	

if(GPIO_ReadInputDataBit(GPIOB,PIN_RED3))
RED_State=RED_State|SETBIT2;
else
RED_State=RED_State&CLRBIT2;	

if(GPIO_ReadInputDataBit(GPIOB,PIN_RED4))
RED_State=RED_State|SETBIT3;
else
RED_State=RED_State&CLRBIT3;	
}

void RED_TASK(void)
{
GET_RED();

	
RED_Stater=RED_State;	
}
