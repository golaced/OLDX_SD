#include  <stm32f10x.h>
#include "stm32f10x_conf.h"
#include "LDchip.h"
#include "Reg_RW.h"
#include "LD3320_config.h"


/************************************************************************************
//	nAsrStatus ������main�������б�ʾ�������е�״̬������LD3320оƬ�ڲ���״̬�Ĵ���
//	LD_ASR_NONE:		��ʾû������ASRʶ��
//	LD_ASR_RUNING��		��ʾLD3320������ASRʶ����
//	LD_ASR_FOUNDOK:		��ʾһ��ʶ�����̽�������һ��ʶ����
//	LD_ASR_FOUNDZERO:	��ʾһ��ʶ�����̽�����û��ʶ����
//	LD_ASR_ERROR:		��ʾһ��ʶ��������LD3320оƬ�ڲ����ֲ���ȷ��״̬
*********************************************************************************/
uint8 nAsrStatus=0;	

void LD3320_Init(void);
uint8 RunASR(void);
void ProcessInt0(void);
void LD3320_EXTI_Cfg(void);
void LD3320_Spi_cfg(void);
void LD3320_GPIO_Cfg(void);
void LED_gpio_cfg(void);

/***********************************************************
* ��    �ƣ� LD3320_main(void)
* ��    �ܣ� ������LD3320�������
* ��ڲ�����  
* ���ڲ�����
* ˵    ����
* ���÷����� 
**********************************************************/ 

void  LD3320_main(void)
{
	uint8 nAsrRes=0;
	LD3320_Init();	  
  PrintCom(TEST_USART," ����������....\r\n"); 
	PrintCom(TEST_USART,"���1����ˮ��\r\n"); 
	PrintCom(TEST_USART,"2����˸\r\n"); 				
	PrintCom(TEST_USART,"3����������\r\n"); 		
	PrintCom(TEST_USART,"4��ȫ��\r\n"); 			

	nAsrStatus = LD_ASR_NONE;		//	��ʼ״̬��û������ASR

	while(1)
	{
 	switch(nAsrStatus)
		{
			case LD_ASR_RUNING:
			case LD_ASR_ERROR:		
					break;
			case LD_ASR_NONE:
					nAsrStatus=LD_ASR_RUNING;
					if (RunASR()==0)	//	����һ��ASRʶ�����̣�ASR��ʼ����ASR��ӹؼ��������ASR����
					{		
						nAsrStatus = LD_ASR_ERROR;
					}
					break;

			case LD_ASR_FOUNDOK:
					 nAsrRes = LD_GetResult( );	//	һ��ASRʶ�����̽�����ȥȡASRʶ����										 
						PrintCom(TEST_USART,"\r\nʶ����:");			
						USART_SendData(TEST_USART,nAsrRes+0x30); 		
								
					 switch(nAsrRes)		   /*�Խ��ִ����ز���,�ͻ��޸�*/
						{
							case CODE_LSD:			/*�����ˮ�ơ�*/
								PrintCom(TEST_USART,"����ˮ�ơ�����ʶ��ɹ�\r\n"); 
															 break;
							case CODE_SS:	 /*�����˸��*/
								PrintCom(TEST_USART,"����˸������ʶ��ɹ�\r\n"); 
															 break;
							case CODE_AJCF:		/*�������������*/
					
								PrintCom(TEST_USART,"����������������ʶ��ɹ�\r\n"); 
															break;
							case CODE_QM:		/*���ȫ��*/
					
								PrintCom(TEST_USART,"��ȫ������ʶ��ɹ�\r\n");
															break;
							default:break;
						}	
						nAsrStatus = LD_ASR_NONE;
					break;
			
			case LD_ASR_FOUNDZERO:
			default:
						nAsrStatus = LD_ASR_NONE;
						break;
			}//switch
	//�����������ʱ���֣��û���ɾ����		
		Board_text(nAsrRes );
	}// while

}
/***********************************************************
* ��    �ƣ�LD3320_Init(void)
* ��    �ܣ�ģ�������˿ڳ�ʼ����
* ��ڲ�����  
* ���ڲ�����
* ˵    ����
* ���÷����� 
**********************************************************/ 
void LD3320_Init(void)
{
	LD3320_GPIO_Cfg();	
	LD3320_EXTI_Cfg();
	LD3320_Spi_cfg();	 
	LED_gpio_cfg();
	LD_reset();
}

/***********************************************************
* ��    �ƣ� void Delay_(int i)
* ��    �ܣ� ����ʱ
* ��ڲ�����  
* ���ڲ�����
* ˵    ����
* ���÷����� 
**********************************************************/ 
void Delay_( int i)
 {     
    while(i--);
 }	
/***********************************************************
* ��    �ƣ�	LD3320_delay(unsigned long uldata)
* ��    �ܣ�	����ʱ����
* ��ڲ�����  
* ���ڲ�����
* ˵    ����
* ���÷����� 
**********************************************************/ 
void LD3320_delay(unsigned long uldata)
{
	unsigned int j  =  0;
	unsigned int g  =  0;
	for (j=0;j<5;j++)
	{
		for (g=0;g<uldata;g++)
		{
			Delay_(120);
		}
	}
}

/***********************************************************
* ��    �ƣ�	RunASR(void)
* ��    �ܣ�	����ASR
* ��ڲ�����  
* ���ڲ�����
* ˵    ����
* ���÷����� 
**********************************************************/ 
uint8 RunASR(void)
{
	uint8 i=0;
	uint8 asrflag=0;
	for (i=0; i<5; i++)					//	��ֹ����Ӳ��ԭ����LD3320оƬ����������������һ������5������ASRʶ������
	{
		LD_AsrStart();						//��ʼ��ASR
		LD3320_delay(100);
		if (LD_AsrAddFixed()==0)	//��ӹؼ����ﵽLD3320оƬ��
		{
			LD_reset();							//	LD3320оƬ�ڲ����ֲ���������������LD3320оƬ
			LD3320_delay(50);				//	���ӳ�ʼ����ʼ����ASRʶ������
			continue;
		}
		LD3320_delay(10);
		if (LD_AsrRun() == 0)
		{
			LD_reset();							//	LD3320оƬ�ڲ����ֲ���������������LD3320оƬ
			LD3320_delay(50);				//	���ӳ�ʼ����ʼ����ASRʶ������
			continue;
		}
		asrflag=1;
		break;										//	ASR���������ɹ����˳���ǰforѭ������ʼ�ȴ�LD3320�ͳ����ж��ź�
	}	
	return asrflag;
}

/***********************************************************
* ��    �ƣ�LD3320_GPIO_Cfg(void)
* ��    �ܣ���ʼ����Ҫ�õ���IO��
* ��ڲ�����  
* ���ڲ�����
* ˵    ����
* ���÷����� 
**********************************************************/ 
void LD3320_GPIO_Cfg(void)
{	
		GPIO_InitTypeDef GPIO_InitStructure;	
		RCC_APB2PeriphClockCmd(LD3320RST_GPIO_CLK | LD3320CS_GPIO_CLK,ENABLE);
		//LD_CS	/RSET
		GPIO_InitStructure.GPIO_Pin =LD3320CS_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(LD3320CS_GPIO_PORT,&GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin =LD3320RST_PIN;
		GPIO_Init(LD3320RST_GPIO_PORT,&GPIO_InitStructure);

}
/***********************************************************
* ��    �ƣ�LD3320_Spi_cfg(void)
* ��    �ܣ�����SPI���ܺͶ˿ڳ�ʼ��
* ��ڲ�����  
* ���ڲ�����
* ˵    ����
* ���÷����� 
**********************************************************/ 
void LD3320_Spi_cfg(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
  //spi�˿�����
	RCC_APB2PeriphClockCmd(LD3320SPI_CLK,ENABLE);	   //ʹ��SPI2����ʱ��
  RCC_APB2PeriphClockCmd(LD3320WR_GPIO_CLK | LD3320SPIMISO_GPIO_CLK | LD3320SPIMOSI_GPIO_CLK | LD3320SPISCK_GPIO_CLK,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = LD3320SPIMISO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(LD3320SPIMISO_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LD3320SPIMOSI_PIN;
	GPIO_Init(LD3320SPIMOSI_GPIO_PORT,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LD3320SPISCK_PIN;
	GPIO_Init(LD3320SPISCK_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LD3320WR_PIN;			//WR
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LD3320WR_GPIO_PORT, &GPIO_InitStructure);
	
	LD_CS_H();
	
	SPI_Cmd(LD3320SPI, DISABLE);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   	//ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						   						//��ģʽ
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					   					//8λ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;						   							//ʱ�Ӽ��� ����״̬ʱ��SCK���ֵ͵�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						   						//ʱ����λ ���ݲ����ӵ�һ��ʱ�ӱ��ؿ�ʼ
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							   							//�������NSS
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;   //�����ʿ��� SYSCLK/128
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   					//���ݸ�λ��ǰ
	SPI_InitStructure.SPI_CRCPolynomial = 7;							   							//CRC����ʽ�Ĵ�����ʼֵΪ7
	SPI_Init(LD3320SPI, &SPI_InitStructure);

	SPI_Cmd(LD3320SPI, ENABLE);

}
/***********************************************************
* ��    �ƣ� LD3320_EXTI_Cfg(void) 
* ��    �ܣ� �ⲿ�жϹ������ú���ض˿�����
* ��ڲ�����  
* ���ڲ�����
* ˵    ����
* ���÷����� 
**********************************************************/ 
void LD3320_EXTI_Cfg(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//ʹ�ܸ��ù���ʱ��

	RCC_APB2PeriphClockCmd(LD3320IRQ_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin =LD3320IRQ_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LD3320IRQ_GPIO_PORT, &GPIO_InitStructure);
	//�ⲿ�ж�������
  GPIO_EXTILineConfig(LD3320IRQEXIT_PORTSOURCE, LD3320IRQPINSOURCE);
  EXTI_InitStructure.EXTI_Line = LD3320IRQEXITLINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger =EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	//�ж�Ƕ������
  NVIC_InitStructure.NVIC_IRQChannel = LD3320IRQN;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


/***********************************************************
* ��    �ƣ�  EXTI IRQHandler(void)
* ��    �ܣ� �ⲿ�жϺ���
* ��ڲ�����  
* ���ڲ�����
* ˵    ����
* ���÷����� 
**********************************************************/ 
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(LD3320IRQEXITLINE)!= RESET ) 
	{
		ProcessInt0(); 
 		PrintCom(TEST_USART,"�����ж�12\r\n");	
		EXTI_ClearFlag(LD3320IRQEXITLINE);
		EXTI_ClearITPendingBit(LD3320IRQEXITLINE);  //���LINE2�ϵ��жϱ�־λ  
	} 
}

/***********************************************************
* ��    �ƣ�void LED_gpio_cfg(void)
* ��    �ܣ�LED�˿�����
* ��ڲ�����  
* ���ڲ�����
* ˵    ����
* ���÷����� 
**********************************************************/ 
void LED_gpio_cfg(void)
{	
		GPIO_InitTypeDef GPIO_InitStructure;
	
		RCC_APB2PeriphClockCmd(LED1_GPIO_CLK | LED2_GPIO_CLK | LED3_GPIO_CLK | LED4_GPIO_CLK,ENABLE);
	
		GPIO_InitStructure.GPIO_Pin = LED1_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = LED2_PIN;
		GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = LED3_PIN;
		GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = LED4_PIN;
		GPIO_Init(LED4_GPIO_PORT, &GPIO_InitStructure);	
	
		LED1_OFF;
		LED2_OFF;
		LED3_OFF;
		LED4_OFF;
}
