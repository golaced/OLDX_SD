#include "head.h"

//初始化PB8为输出口.并使能这个口的时钟		    
//蜂鸣器初始化
void BEEP_Init(void)
{
 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_BEEP| RCC_APB2Periph_AFIO, ENABLE);	 //使能GPIOB端口时钟
 
 GPIO_InitStructure.GPIO_Pin = PIN_BEEP;				 //BEEP-->PB.8 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //速度为50MHz
 GPIO_Init(GPIO_BEEP, &GPIO_InitStructure);	 //根据参数初始化GPIOB.8
 
 //GPIO_SetBits(GPIO_BEEP,PIN_BEEP);//输出0，关闭蜂鸣器输出

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	//使能定时器4时钟
	 
   //初始化TIM4
	TIM_TimeBaseStructure.TIM_Period = 499; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =71; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM4 Channel3 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //TIM1_OutputNState选择互补输出比较状态
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; //TIM1_OutputNState选择互补输出比较状态
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC3
	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
  TIM_CtrlPWMOutputs(TIM1, ENABLE); 
	TIM_Cmd(TIM1, ENABLE);  //使能TIM4
	TIM_SetCompare1(TIM1,240);
}

void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO, ENABLE);	 //使能GPIOB端口时钟
 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;				 //BEEP-->PB.8 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //速度为50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);	 //根据参数初始化GPIOB.8
 
 GPIO_SetBits(GPIOA,GPIO_Pin_11);//输出0，关闭蜂鸣器输出
 GPIO_SetBits(GPIOA,GPIO_Pin_12);//输出0，关闭蜂鸣器输出
}

u8 RED_State=0,RED_Stater=0;;
void RED_Init(void)
{
 GPIO_InitTypeDef GPIO_InitStructure;
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO, ENABLE);	 //使能GPIOB端口时钟
 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;				 //BEEP-->PB.8 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //速度为50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	 //根据参数初始化GPIOB.8
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
