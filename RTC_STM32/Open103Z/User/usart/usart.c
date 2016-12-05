#include "usart.h"
#include "stm32f10x_it.h"
#include "stm32f10x_dma.h"
#include "des.h"	
#include "head.h"	
#include "rtc.h"
#include "math.h"
/******************************************************************************/
uint8_t SendBuff[SENDBUFF_SIZE]={' ','D','M','A','1',' '};//数组【0】无法使用
u8 uart_test[4]={0,0,0,0}; 
u8 dma_can_tx=1;
u8 Addr_485=0;
void USART_ON(u8 uart_num,u32 baud,u8 m_irq,u8 s_irq)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure; 
		/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	/* config USART clock */
	if (uart_num == 1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	else if(uart_num==2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	else if(uart_num==3)									
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
		if (uart_num == 1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	else if(uart_num==2)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	else if(uart_num==3)									
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	
		if (uart_num == 1)
		{	/* USART GPIO config */
	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);}
	else if(uart_num==2)
			{	/* USART GPIO config */
	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);}
	else if(uart_num==3)									
				{	/* USART GPIO config */
	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);    
	/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);}

	
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	if (uart_num == 1)
	{USART_Init(USART1, &USART_InitStructure);
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
   	USART_Cmd(USART1, ENABLE);}
	else if(uart_num==2)
		{USART_Init(USART2, &USART_InitStructure);
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
   	USART_Cmd(USART2, ENABLE);}
	else if(uart_num==3)									
			{USART_Init(USART3, &USART_InitStructure);
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
   	USART_Cmd(USART3, ENABLE);}
	
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
if (uart_num == 1)
	{/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = m_irq;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = s_irq;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);}
	else if(uart_num==2)
		{/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = m_irq;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = s_irq;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);}
	else if(uart_num==3)									
		{/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = m_irq;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = s_irq;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);}
	

}

void RS485_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO, ENABLE);	 //使能GPIOB端口时钟
 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;				 //BEEP-->PB.8 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //速度为50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);	 //根据参数初始化GPIOB.8
 
	EN_485_RX;
}


void USART1_DMA_Config(void)
{
		DMA_InitTypeDef DMA_InitStructure;
	
		/*开启DMA时钟*/
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
		//NVIC_Config();	   			//配置DMA中断

		/*设置DMA源：串口数据寄存器地址*/
		DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;	   

		/*内存地址(要传输的变量的指针)-----------------------分配数组*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)SendBuff;

		/*方向：从内存到外设*/		
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	

		/*传输大小DMA_BufferSize=SENDBUFF_SIZE*/	
		DMA_InitStructure.DMA_BufferSize = SENDBUFF_SIZE;

		/*外设地址不增*/	    
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 

		/*内存地址自增*/
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	

		/*外设数据单位*/	
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

		/*内存数据单位 8bit*/
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 

		/*DMA模式：不断循环*/
		//DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	 

		/*优先级：中*/	
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  

		/*禁止内存到内存的传输	*/
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

		/*配置DMA1的4通道*/		   
		DMA_Init(DMA1_Channel4, &DMA_InitStructure); 	   
		
		/*使能DMA*/
		DMA_Cmd (DMA1_Channel4,ENABLE);					
		//DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
}


void USART2_DMA_Config(void)
{	NVIC_InitTypeDef NVIC_InitStructure; 
		DMA_InitTypeDef DMA_InitStructure;
	
		/*开启DMA时钟*/
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
		//NVIC_Config();	   			//配置DMA中断

		/*设置DMA源：串口数据寄存器地址*/
		DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_DR_Base;	   

		/*内存地址(要传输的变量的指针)-----------------------分配数组*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)SendBuff;

		/*方向：从内存到外设*/		
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	

		/*传输大小DMA_BufferSize=SENDBUFF_SIZE*/	
		DMA_InitStructure.DMA_BufferSize = SENDBUFF_SIZE;

		/*外设地址不增*/	    
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 

		/*内存地址自增*/
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	

		/*外设数据单位*/	
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

		/*内存数据单位 8bit*/
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 

		/*DMA模式：不断循环*/
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
		//DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	 

		/*优先级：中*/	
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  

		/*禁止内存到内存的传输	*/
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

		/*配置DMA1的4通道*/		   
		DMA_Init(DMA1_Channel7, &DMA_InitStructure); 	   
		
		/*使能DMA*/
		DMA_Cmd (DMA1_Channel7,ENABLE);					
		DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
		
			//DMA发送中断设置  
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;  
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
		NVIC_Init(&NVIC_InitStructure);  
}
void USART_init(void)
{
	USART_ON(1,115200,2,1);//485
//	USART_ON(2,115200,2,2);//SCAN
//	USART2_DMA_Config();
	
}
ID CARD[MAX_SAVE];
u8 open_lock1=0;
void UsartSend(uint8_t ch)
{
	USART_SendData(USART1, (uint8_t) ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{}
}

void RS_485_send(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend(dataToSend[i]);
}

void RS_485_send_over(u8 *dataToSend , u8 length)
{
     UsartSend('O');
		 UsartSend('V');
		 UsartSend('E');
		 UsartSend('R');
}

void Send_Card_Info(void)
{u8 i;	u8 sum = 0;u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//校验
	_temp = Readtime.year;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  	_temp = Readtime.month;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
		_temp = Readtime.day;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
		_temp = Readtime.hour;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
		_temp = Readtime.min;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
		_temp = Readtime.sec;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	RS_485_send(data_to_send, _cnt);
}


void USART1_IRQ_TASK(u8 res)//485
{
	
				
}	


void my_itoa(int num,u8* str)  
{  
    int sign = num;  
    int i = 0;  
    int j = 0;  
    char temp[100];  
    //如果是负数就去掉符号,将-1234转成1234  
    if(sign < 0)  
    {  
        num = -num;  
    }  
    //转成字符串，1234转成"4321"  
    do  
    {  
        temp[i] = num % 10 + '0';  
        num /= 10;  
        i++;  
    }while(num > 0);  
    //如果是负数的话，加个符号在末尾，如："4321-"  
    if(sign < 0)  
    {  
        temp[i++] = '-';  
    }  
    temp[i] = '\0';  
    i--;  
    //将temp数组中逆序输入到str数组中  
    //将"4321-" ====> "-1234"  
    while(i >= 0)  
    {  
        str[j] = temp[i];  
        j++;  
        i--;  
    }  
    //字符串结束标识  
    str[j] = '\0';   
}  


#define MAX_NUM 125

char RX_BUF[MAX_NUM],RX_BUF_OUT[MAX_NUM];
void USART2_IRQ_TASK(	u8 res)//FLY
{  u8 BUF1[14]={0},BUF2[14]={0};
u8 BUF3[8]={0},BUF4[8]={0};
	ID CARDr;
	u8 j,num_cnt,num_cnt1,num;
	u16 l;
	u8 already_have=0,in_right_time=0;
	u8 ge,shi,bai,qian,wang;
	u16 k,loc;
  static u8 i;	
	static u8 state;
		switch(state){
			case 0:
			if(res==0x0D)
			state=1;	
			else
			RX_BUF[i++]=res;		//记录接收到的值
			break;
			case 1:
      if(res==0x0A)
			{ for(j=0;j<MAX_NUM;j++)
				RX_BUF_OUT[j]=' ';
				for(j=0;j<MAX_NUM-2;j++)
				RX_BUF_OUT[j]=RX_BUF[j]; 
			//	Show_Str(2,2,240,16,RX_BUF_OUT,32,0);
				OLED_Init();
				//des_test(RX_BUF_OUT,KEY_LOCK,KEY_UNLOCK,MAX_NUM);	
				des_unlock(RX_BUF_OUT,KEY_LOCK,KEY_UNLOCK,MAX_NUM,0);		
			/*OLED_Fill(0x00);
	    for(i=0;i<14;i++)BUF1[i]=RX_BUF_OUT[i];
			OLED_P6x8Str(3,0,"IN :");OLED_P6x8Str(27,0,BUF1);		
			for(i=0;i<8;i++)BUF3[i]=KEY_LOCK[i];
			OLED_P6x8Str(3,2,"LKE:");OLED_P6x8Str(27,2,BUF3);		
		
			for(i=0;i<8;i++)BUF4[i]=KEY_UNLOCK[i];
			OLED_P6x8Str(3,4,"UKE:");OLED_P6x8Str(27,4,BUF4);
			for(i=0;i<14;i++)BUF2[i]=MyMessage_Kick[i];
			OLED_P6x8Str(3,6,"OUT:");OLED_P6x8Str(27,6,BUF2);*/				
 /*
				     CARDr.id= (MyMessage_Kick[0]-48)*10000+ (MyMessage_Kick[1]-48)*1000
							+(MyMessage_Kick[2]-48)*100+(MyMessage_Kick[3]-48)*10+(MyMessage_Kick[4]-48);
						 CARDr.time_s.month=(MyMessage_Kick[5]-48)*10+(MyMessage_Kick[6]-48);
						 CARDr.time_s.day=  (MyMessage_Kick[7]-48)*10+(MyMessage_Kick[8]-48);
						 CARDr.time_s.hour= (MyMessage_Kick[9]-48)*10+(MyMessage_Kick[10]-48);
						 CARDr.time_s.min=  (MyMessage_Kick[11]-48)*10+(MyMessage_Kick[12]-48);
						 CARDr.time_e.month=(MyMessage_Kick[13]-48)*10+(MyMessage_Kick[14]-48);
						 CARDr.time_e.day=  (MyMessage_Kick[15]-48)*10+(MyMessage_Kick[16]-48);
						 CARDr.time_e.hour= (MyMessage_Kick[17]-48)*10+(MyMessage_Kick[18]-48);
						 CARDr.time_e.min=  (MyMessage_Kick[19]-48)*10+(MyMessage_Kick[20]-48);
						 CARDr.times=       (MyMessage_Kick[21]-48)*10+(MyMessage_Kick[22]-48);*/
						 if(MyMessage_Kick[0]=='B'&&MyMessage_Kick[1]=='I'&&MyMessage_Kick[2]=='T'){
		#define  DATA_OFF_SET  3		 
						 CARDr.id= (MyMessage_Kick[0+DATA_OFF_SET]-48)*10000+ (MyMessage_Kick[1+DATA_OFF_SET]-48)*1000
							+(MyMessage_Kick[2+DATA_OFF_SET]-48)*100+(MyMessage_Kick[4+DATA_OFF_SET]-48)*10+(MyMessage_Kick[5+DATA_OFF_SET]-48);
						 CARDr.time_s.month=(MyMessage_Kick[6+DATA_OFF_SET]-48)*10+(MyMessage_Kick[8+DATA_OFF_SET]-48);
						 CARDr.time_s.day=  (MyMessage_Kick[9+DATA_OFF_SET]-48)*10+(MyMessage_Kick[10+DATA_OFF_SET]-48);
						 CARDr.time_s.hour= (MyMessage_Kick[12+DATA_OFF_SET]-48)*10+(MyMessage_Kick[13+DATA_OFF_SET]-48);
						 CARDr.time_s.min=  (MyMessage_Kick[14+DATA_OFF_SET]-48)*10+(MyMessage_Kick[16+DATA_OFF_SET]-48);
						 CARDr.time_e.month=(MyMessage_Kick[17+DATA_OFF_SET]-48)*10+(MyMessage_Kick[18+DATA_OFF_SET]-48);
						 CARDr.time_e.day=  (MyMessage_Kick[20+DATA_OFF_SET]-48)*10+(MyMessage_Kick[21+DATA_OFF_SET]-48);
						 CARDr.time_e.hour= (MyMessage_Kick[22+DATA_OFF_SET]-48)*10+(MyMessage_Kick[24+DATA_OFF_SET]-48);
						 CARDr.time_e.min=  (MyMessage_Kick[25+DATA_OFF_SET]-48)*10+(MyMessage_Kick[26+DATA_OFF_SET]-48);
						 CARDr.times=       (MyMessage_Kick[28+DATA_OFF_SET]-48)*10+(MyMessage_Kick[29+DATA_OFF_SET]-48);
		  if(CARDr.id==998)//管理员		
			{
			open_lock1=1;
			OLED_Fill(0x00);
			//OLED_P8x16Str(10,3,"Admin OpenDoor!");	
			//delay_ms(1500);	
			OLED_Fill(0x00);
			}
			else{
									for(k=0;k<MAX_SAVE;k++)
									 {
											if(CARD[k].id==CARDr.id)
											{loc=k;k=MAX_SAVE+1;already_have=1;}
									 }
									 
									if(!already_have){
											for(k=0;k<MAX_SAVE;k++){
											if(CARD[k].id==0)
											{
											 CARD[k].id=          CARDr.id;
											 CARD[k].time_s.month=CARDr.time_s.month;
											 CARD[k].time_s.day=  CARDr.time_s.day;
											 CARD[k].time_s.hour= CARDr.time_s.hour;
											 CARD[k].time_s.min=  CARDr.time_s.min;
											 CARD[k].time_e.month=CARDr.time_e.month;;
											 CARD[k].time_e.day=  CARDr.time_e.day;
											 CARD[k].time_e.hour= CARDr.time_e.hour;
											 CARD[k].time_e.min=  CARDr.time_e.min;
											 CARD[k].times=       CARDr.times;
											 loc=k;k=MAX_SAVE+1;}
											}
									 
									 if(CARD[loc].times>0){
											if(Readtime.month>=CARD[loc].time_s.month&&Readtime.month<=CARD[loc].time_e.month&&
												Readtime.day>=CARD[loc].time_s.day&&Readtime.day<=CARD[loc].time_e.day)
											 {if(Readtime.hour>CARD[loc].time_s.hour&&Readtime.hour<CARD[loc].time_e.hour)
													 in_right_time=1;
											 else if( Readtime.hour==CARD[loc].time_s.hour&&CARD[loc].time_s.min<=Readtime.min)
									
														   in_right_time=1;
											 else if( Readtime.hour==CARD[loc].time_e.hour&&CARD[loc].time_e.min>=Readtime.min)
														   in_right_time=1;
											  }
			
											
											if(in_right_time)
											{	if(!open_lock1)CARD[loc].times--;
											OLED_Fill(0x00);
										//	OLED_P8x16Str(10,3,"User OpenDoor!");	
											//delay_ms(3500);	
											OLED_Fill(0x00);
												my_itoa(CARD[loc].id,BUF3);
											OLED_P6x8Str(0,0,"Id :");OLED_P6x8Str(27,0,BUF3);	
											
											my_itoa(CARD[loc].times,BUF4);				
											OLED_P6x8Str(67,0,"Num:");OLED_P6x8Str(67+27,0,BUF4);
												
											Time_Conver_char(1,loc,BUF1);	
										
																for(i=0;i<15;i++)
														 if(BUF1[i]==0)
																 BUF1[i]=' ';
														 BUF1[13]=0;	OLED_P6x8Str(3,2,"Str:");OLED_P6x8Str(27,2,BUF1);	
											Time_Conver_char(2,loc,BUF2);	
														for(i=0;i<15;i++)
														 if(BUF2[i]==0)
																 BUF2[i]=' ';
														 BUF2[13]=0;
											OLED_P6x8Str(3,4,"End:");OLED_P6x8Str(27,4,BUF2);		
													
											open_lock1=1;
											}//--right time
											else
											{
											 OLED_Fill(0x00);
													 OLED_Fill(0x00);
												Time_Conver_char(1,loc,BUF1);	
																for(i=0;i<15;i++)
														 if(BUF1[i]==0)
																 BUF1[i]=' ';
														 BUF1[13]=0;OLED_P6x8Str(3,0,"Str:");OLED_P6x8Str(27,0,BUF1);	
											Time_Conver_char(2,loc,BUF2);	
														for(i=0;i<15;i++)
														 if(BUF2[i]==0)
																 BUF2[i]=' ';
														 BUF2[13]=0;
											OLED_P6x8Str(3,1,"End:");OLED_P6x8Str(27,1,BUF2);		
											 OLED_P8x16Str(10,3,"Cardfail:Time!");	
											 delay_ms(1500);
											 CARD[loc].id=          0;
											 CARD[loc].time_s.month=0;
											 CARD[loc].time_s.day=  0;
											 CARD[loc].time_s.hour= 0;
											 CARD[loc].time_s.min=  0;
											 CARD[loc].time_e.month=0;
											 CARD[loc].time_e.day=  0;
											 CARD[loc].time_e.hour= 0;
											 CARD[loc].time_e.min=  0;
											 CARD[loc].times=       0;
											}
										}//times
										else
										{
										OLED_Fill(0x00);
										OLED_P8x16Str(10,3,"Cardfail:Num!");
										delay_ms(1500);						
										}
								  }//--already have1
									else if(already_have&&CARD[loc].times!=0)
									{
									 if(CARD[loc].times>0){
											if(Readtime.month>=CARD[loc].time_s.month&&Readtime.month<=CARD[loc].time_e.month&&
												Readtime.day>=CARD[loc].time_s.day&&Readtime.day<=CARD[loc].time_e.day)
											 {if(Readtime.hour>CARD[loc].time_s.hour&&Readtime.hour<CARD[loc].time_e.hour)
													 in_right_time=1;
											 else if( Readtime.hour==CARD[loc].time_s.hour&&CARD[loc].time_s.min<=Readtime.min)
									
														   in_right_time=1;
											 else if( Readtime.hour==CARD[loc].time_e.hour&&CARD[loc].time_e.min>=Readtime.min)
														   in_right_time=1;
											  }

											
											if(in_right_time)
											{	
												if(!open_lock1)
												CARD[loc].times--;
											OLED_Fill(0x00);
											//OLED_P8x16Str(10,3,"User OpenDoor!");	
											//delay_ms(1500);	
											OLED_Fill(0x00);
											my_itoa(CARD[loc].id,BUF3);
											OLED_P6x8Str(0,0,"Id :");OLED_P6x8Str(27,0,BUF3);	
											
											my_itoa(CARD[loc].times,BUF4);				
											OLED_P6x8Str(67,0,"Num:");OLED_P6x8Str(67+27,0,BUF4);
												
											Time_Conver_char(1,loc,BUF1);	
											
																for(i=0;i<15;i++)
														 if(BUF1[i]==0)
																 BUF1[i]=' ';
														 BUF1[13]=0;OLED_P6x8Str(3,2,"Str:");OLED_P6x8Str(27,2,BUF1);	
											Time_Conver_char(2,loc,BUF2);	
														for(i=0;i<15;i++)
														 if(BUF2[i]==0)
																 BUF2[i]=' ';
														 BUF2[13]=0;
											OLED_P6x8Str(3,4,"End:");OLED_P6x8Str(27,4,BUF2);		
											open_lock1=1;}
											else
											{
											 OLED_Fill(0x00);
												Time_Conver_char(1,loc,BUF1);	
																for(i=0;i<15;i++)
														 if(BUF1[i]==0)
																 BUF1[i]=' ';
														 BUF1[13]=0;OLED_P6x8Str(3,0,"Str:");OLED_P6x8Str(27,0,BUF1);	
											Time_Conver_char(2,loc,BUF2);	
														for(i=0;i<15;i++)
														 if(BUF2[i]==0)
																 BUF2[i]=' ';
														 BUF2[13]=0;
											OLED_P6x8Str(3,1,"End:");OLED_P6x8Str(27,1,BUF2);		
												OLED_P8x16Str(10,3,"Cardfail:Time!");	
												
												delay_ms(1500);
											}
										}
										else
										{
										OLED_Fill(0x00);
										OLED_P8x16Str(10,3,"Cardfail:Num!");
										delay_ms(1500);						
										}
									
									
									
									}
									 else if(already_have&&CARD[loc].times==0)
									 {
									 
										OLED_Fill(0x00);
										OLED_P8x16Str(10,3,"Cardfail:Num!");
										delay_ms(1500);				
									 }
										 
			 }//--end of guanli
		 }
	  else
		{
										OLED_Fill(0x00);
										OLED_P8x16Str(10,3,"Wrong Card!");
										delay_ms(1500);	
		}
				//if(BUF2[0]=='6')
				//open_lock1=1;
				for(j=0;j<14;j++)
				BUF2[j]=' ';
				for(j=0;j<MAX_NUM;j++)
				RX_BUF[j]=' ';
				state=0;i=0;	
			}
			else
			{state=0;i=0;}
			break;
		 }
}			
	
void USART3_IRQ_TASK(void)//FLOW
{
	
}

void UART_TX_CHAR(u8 sel ,char data)
{
	if(sel==1)
	{
	USART_SendData(USART1, (u8) data);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
	}
	else 	if(sel==2)
	{
	USART_SendData(USART2, (u8) data);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		;
	}
	else 	if(sel==3)
	{
	USART_SendData(USART3, (u8) data);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
		;
	}
}


/// 重定向c库函数printf到USART
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到USART1 */
		USART_SendData(TEST_USART, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(TEST_USART, USART_FLAG_TC) == RESET);		
	
		return (ch);
}

/// 重定向c库函数scanf到USART
int fgetc(FILE *f)
{
		/* 等待串口1输入数据 */
		while (USART_GetFlagStatus(TEST_USART, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(TEST_USART);
}

/*********************************************END OF FILE**********************/
