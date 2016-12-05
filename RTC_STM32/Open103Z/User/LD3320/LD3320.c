#include "LD3320.h"

/************************************************************************************
//	nAsrStatus ������main�������б�ʾ�������е�״̬������LD3320оƬ�ڲ���״̬�Ĵ���
//	LD_ASR_NONE:			��ʾû������ASRʶ��
//	LD_ASR_RUNING��		��ʾLD3320������ASRʶ����
//	LD_ASR_FOUNDOK:		��ʾһ��ʶ�����̽�������һ��ʶ����
//	LD_ASR_FOUNDZERO:	��ʾһ��ʶ�����̽�����û��ʶ����
//	LD_ASR_ERROR:			��ʾһ��ʶ��������LD3320оƬ�ڲ����ֲ���ȷ��״̬
*********************************************************************************/
uint8 nAsrStatus = 0;	
uint8 nLD_Mode = LD_MODE_IDLE;//������¼��ǰ���ڽ���ASRʶ�����ڲ���MP3
uint8 ucRegVal;
uint8 nAsrRes=0;
///�û��޸�
void LD3320_main(void)
{static u8 init_flag=0;

	//LD3320_init();	  
 /*	printf("1����ˮ��\r\n"); 
	printf("2����˸\r\n"); 				
	printf("3����������\r\n"); 		
	printf("4��ȫ��\r\n"); 			
	printf("5��״̬\r\n"); 		*/
	if(!init_flag)
	{nAsrStatus = LD_ASR_NONE;//��ʼ״̬��û������ASR
		init_flag=1;
	}

/*
	///�������״̬����������¼������������ASRʶ������е��ĸ�״̬
#define LD_ASR_NONE					0x00	//��ʾû������ASRʶ��
#define LD_ASR_RUNING				0x01	//��ʾLD3320������ASRʶ����
#define LD_ASR_FOUNDOK			0x10	//��ʾһ��ʶ�����̽�������һ��ʶ����
#define LD_ASR_FOUNDZERO 		0x11	//��ʾһ��ʶ�����̽�����û��ʶ����
#define LD_ASR_ERROR	 			0x31	//	��ʾһ��ʶ��������LD3320оƬ�ڲ����ֲ���ȷ��״̬
	*/
		switch(nAsrStatus)
		{
			case LD_ASR_RUNING:
				if(LD_ReadReg(0xb2)==0x21)
					ProcessInt(); 
			    break;
			case LD_ASR_ERROR:		
					break;
			case LD_ASR_NONE:
					nAsrStatus=LD_ASR_RUNING;
					if (RunASR()==0)//����һ��ASRʶ�����̣�ASR��ʼ����ASR��ӹؼ��������ASR����
					{		
						nAsrStatus = LD_ASR_ERROR;
					}
					break;
			case LD_ASR_FOUNDOK:
					 nAsrRes = LD_GetResult( );//һ��ASRʶ�����̽�����ȥȡASRʶ����										 
					 printf("\r\nʶ����:%d", nAsrRes);			 		
								
					 switch(nAsrRes)		   			//�Խ��ִ����ز���,�ͻ��޸�
						{
							case CODE_LSD:					//�����ˮ�ơ�
								printf(" ��ˮ�� ָ��ʶ��ɹ�\r\n"); 
															 break;
							case CODE_SS:	 					//�����˸��
								printf(" ��˸ ָ��ʶ��ɹ�\r\n"); 
															 break;
							case CODE_AJCF:					//�������������
								printf(" �������� ָ��ʶ��ɹ�\r\n"); 
															break;
							case CODE_QM:						//���ȫ��
								printf(" ȫ�� ָ��ʶ��ɹ�\r\n");
															break;
							case CODE_JT:						//���״̬��
								printf(" ״̬ ָ��ʶ��ɹ�\r\n");
							
							default:break;
						}	
					nAsrStatus = LD_ASR_NONE;
					break;
			case LD_ASR_FOUNDZERO:
			default:
					nAsrStatus = LD_ASR_NONE;
					break;
			}//switch
		//���������
		Board_text(nAsrRes );
	//}// while
}

void LD_AsrAddFixed_ByString(char * pRecogString, u8 k)
{
	u8 nAsrAddLength;
	if (*pRecogString==0)
		return;

	LD_WriteReg(0xc1, k );	   //ASR��ʶ����Index��00H��FFH��
	LD_WriteReg(0xc3, 0 );	   //ASR��ʶ�������ʱд��00
	LD_WriteReg(0x08, 0x04);	 //���FIFO���ݵ�0λ��д��1�����FIFO_DATA ��2λ��д��1�����FIFO_EXT
	LD3320_delay(1);
	LD_WriteReg(0x08, 0x00);	 //���FIFO���ݵ�0λ�����ָ��FIFO����д��һ��00H��
	LD3320_delay(1);
	for (nAsrAddLength=0; nAsrAddLength<50; nAsrAddLength++)
	{
		if (pRecogString[nAsrAddLength] == 0)
			break;
		LD_WriteReg(0x5, pRecogString[nAsrAddLength]);	  //дFIFO_EXT���ݿ�
	}
	
	LD_WriteReg(0xb9, nAsrAddLength);	  //��ǰ���ʶ�����ַ������ȳ�ʼ��ʱд��00H ÿ���һ��ʶ����Ҫ�趨һ��
	LD_WriteReg(0xb2, 0xff);	  //DSPæ��״̬ 0x21 ��ʾ�� ���Խ�����һ��	   ??
	LD_WriteReg(0x37, 0x04);	  //����ʶ����������·��Ĵ��� д04H��֪ͨDSPҪ���һ��ʶ���
	LD_WriteReg(0x37, 0x04);
}

//================================================================
/*/����
void LD_AsrAddFixed_ByIndex(u8 nIndex)	 
{
	switch(nIndex)
	{
		case  0: LD_AsrAddFixed_ByString(STR_00,nIndex); break;
		case  1: LD_AsrAddFixed_ByString(STR_01,nIndex); break;
		case  2: LD_AsrAddFixed_ByString(STR_02,nIndex); break;
		case  3: LD_AsrAddFixed_ByString(STR_03,nIndex); break;
		case  4: LD_AsrAddFixed_ByString(STR_04,nIndex); break;
		case  5: LD_AsrAddFixed_ByString(STR_05,nIndex); break;
		case  6: LD_AsrAddFixed_ByString(STR_06,nIndex); break;
		case  7: LD_AsrAddFixed_ByString(STR_07,nIndex); break;
		case  8: LD_AsrAddFixed_ByString(STR_08,nIndex); break;
		case  9: LD_AsrAddFixed_ByString(STR_09,nIndex); break;
		case 10: LD_AsrAddFixed_ByString(STR_10,nIndex); break;
		case 11: LD_AsrAddFixed_ByString(STR_11,nIndex); break;
		case 12: LD_AsrAddFixed_ByString(STR_12,nIndex); break;
		case 13: LD_AsrAddFixed_ByString(STR_13,nIndex); break;
		case 14: LD_AsrAddFixed_ByString(STR_14,nIndex); break;
		case 15: LD_AsrAddFixed_ByString(STR_15,nIndex); break;
		case 16: LD_AsrAddFixed_ByString(STR_16,nIndex); break;
		case 17: LD_AsrAddFixed_ByString(STR_17,nIndex); break;
		case 18: LD_AsrAddFixed_ByString(STR_18,nIndex); break;
		case 19: LD_AsrAddFixed_ByString(STR_19,nIndex); break;
		case 20: LD_AsrAddFixed_ByString(STR_20,nIndex); break;
		case 21: LD_AsrAddFixed_ByString(STR_21,nIndex); break;
		case 22: LD_AsrAddFixed_ByString(STR_22,nIndex); break;
		case 23: LD_AsrAddFixed_ByString(STR_23,nIndex); break;
		case 24: LD_AsrAddFixed_ByString(STR_24,nIndex); break;
		case 25: LD_AsrAddFixed_ByString(STR_25,nIndex); break;
		case 26: LD_AsrAddFixed_ByString(STR_26,nIndex); break;
		case 27: LD_AsrAddFixed_ByString(STR_27,nIndex); break;
		case 28: LD_AsrAddFixed_ByString(STR_28,nIndex); break;
		case 29: LD_AsrAddFixed_ByString(STR_29,nIndex); break;
		case 30: LD_AsrAddFixed_ByString(STR_30,nIndex); break;
		case 31: LD_AsrAddFixed_ByString(STR_31,nIndex); break;
		case 32: LD_AsrAddFixed_ByString(STR_32,nIndex); break;
		case 33: LD_AsrAddFixed_ByString(STR_33,nIndex); break;
		case 34: LD_AsrAddFixed_ByString(STR_34,nIndex); break;
		case 35: LD_AsrAddFixed_ByString(STR_35,nIndex); break;
		case 36: LD_AsrAddFixed_ByString(STR_36,nIndex); break;
		case 37: LD_AsrAddFixed_ByString(STR_37,nIndex); break;
		case 38: LD_AsrAddFixed_ByString(STR_38,nIndex); break;
		case 39: LD_AsrAddFixed_ByString(STR_39,nIndex); break;
		case 40: LD_AsrAddFixed_ByString(STR_40,nIndex); break;
		case 41: LD_AsrAddFixed_ByString(STR_41,nIndex); break;
		case 42: LD_AsrAddFixed_ByString(STR_42,nIndex); break;
		case 43: LD_AsrAddFixed_ByString(STR_43,nIndex); break;
		case 44: LD_AsrAddFixed_ByString(STR_44,nIndex); break;
		case 45: LD_AsrAddFixed_ByString(STR_45,nIndex); break;
		case 46: LD_AsrAddFixed_ByString(STR_46,nIndex); break;
		case 47: LD_AsrAddFixed_ByString(STR_47,nIndex); break;
		case 48: LD_AsrAddFixed_ByString(STR_48,nIndex); break;
		case 49: LD_AsrAddFixed_ByString(STR_49,nIndex); break;
	}
}*/

static uint8 LD_AsrAddFixed(void)
{
	uint8 k, flag;
	uint8 nAsrAddLength;
	#define DATE_A 5    //�����ά��ֵ   �������
	#define DATE_B 20		//����һά��ֵ   ��󵥴���
	//��ӹؼ��ʣ��û��޸�
	uint8  sRecog[DATE_A][DATE_B] = {
	 			"liu shui deng",\
				"shan shuo",\
				"an jian chu fa",\
				"quan mie",\
				"zhuang tai"\
		
																	};	
	uint8  pCode[DATE_A] = {
	 															CODE_LSD,	\
																CODE_SS,	\
																CODE_AJCF,\
																CODE_QM,	\
																CODE_JT		\
															};	//���ʶ���룬�û��޸�
	flag = 1;
	for (k=0; k<DATE_A; k++)
	{			
		if(LD_Check_ASRBusyFlag_b2() == 0)
		{
			flag = 0;
			break;
		}

		LD_WriteReg(0xc1, pCode[k] );
		LD_WriteReg(0xc3, 0);
		LD_WriteReg(0x08, 0x04);
		LD3320_delay(1);
		LD_WriteReg(0x08, 0x00);
		LD3320_delay(1);

		for (nAsrAddLength=0; nAsrAddLength<DATE_B; nAsrAddLength++)
		{
			if (sRecog[k][nAsrAddLength] == 0)
				break;
			LD_WriteReg(0x5, sRecog[k][nAsrAddLength]);
		}
		LD_WriteReg(0xb9, nAsrAddLength);
		LD_WriteReg(0xb2, 0xff);
		LD_WriteReg(0x37, 0x04);
		LD_WriteReg(0x37, 0x04);
	}	 
	return flag;
}

static void Board_text(uint8 Code_Val)
{																					 
	switch(Code_Val)  //�Խ��ִ����ز���
	{
		case CODE_LSD:  //�����ˮ�ơ�
			
		break;
		case CODE_SS:	  //�����˸��
	
		break;
		case CODE_AJCF:	//�������������
	
		break;
		case CODE_QM:		//���ȫ��
	
		break;
		case CODE_JT:		//���״̬��
	
		break;
		default:break;
	}	
}


///�û��޸� end

///��س�ʼ��
void LD3320_init(void)
{
	LD3320_GPIO_Cfg();	
	LD3320_EXTI_Cfg();
	LD3320_SPI_cfg();
	LD_reset();
}

static void LD3320_GPIO_Cfg(void)
{	
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(LD3320RST_GPIO_CLK | LD3320CS_GPIO_CLK|RCC_APB2Periph_AFIO,ENABLE);
		//LD_CS	/RSET
		GPIO_InitStructure.GPIO_Pin =LD3320CS_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(LD3320CS_GPIO_PORT,&GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin =LD3320RST_PIN;
		GPIO_Init(LD3320RST_GPIO_PORT,&GPIO_InitStructure);
}

static void LD3320_EXTI_Cfg(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	

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

static void LD3320_SPI_cfg(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
  //spi�˿�����
	RCC_APB2PeriphClockCmd(LD3320SPI_CLK,ENABLE);		
  RCC_APB2PeriphClockCmd(LD3320WR_GPIO_CLK | LD3320SPIMISO_GPIO_CLK | LD3320SPIMOSI_GPIO_CLK | LD3320SPISCK_GPIO_CLK,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = LD3320SPIMISO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(LD3320SPIMISO_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LD3320SPIMOSI_PIN;
	GPIO_Init(LD3320SPIMOSI_GPIO_PORT,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LD3320SPISCK_PIN;
	GPIO_Init(LD3320SPISCK_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LD3320WR_PIN;				
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


///��س�ʼ�� end 

///�м��
void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(LD3320IRQEXITLINE)!= RESET ) 
	{
		ProcessInt(); 
 	//	printf("�����ж�12\r\n");	
		EXTI_ClearFlag(LD3320IRQEXITLINE);
		EXTI_ClearITPendingBit(LD3320IRQEXITLINE);//���LINE�ϵ��жϱ�־λ  
	} 
}

static void LD3320_delay(unsigned long uldata)
{
	unsigned int i  =  0;
	unsigned int j  =  0;
	unsigned int k  =  0;
	for (i=0;i<5;i++)
	{
		for (j=0;j<uldata;j++)
		{
			k = 200;
			while(k--);
		}
	}
}

static uint8 RunASR(void)
{
	uint8 i=0;
	uint8 asrflag=0;
	for (i=0; i<5; i++)		//��ֹ����Ӳ��ԭ����LD3320оƬ����������������һ������5������ASRʶ������
	{
		LD_AsrStart();			//��ʼ��ASR
		LD3320_delay(100);
		if (LD_AsrAddFixed()==0)	//��ӹؼ����ﵽLD3320оƬ��
		{
			LD_reset();				//LD3320оƬ�ڲ����ֲ���������������LD3320оƬ
			LD3320_delay(50);	//���ӳ�ʼ����ʼ����ASRʶ������
			continue;
		}
		LD3320_delay(10);
		if (LD_AsrRun() == 0)
		{
			LD_reset();			 //LD3320оƬ�ڲ����ֲ���������������LD3320оƬ
			LD3320_delay(50);//���ӳ�ʼ����ʼ����ASRʶ������
			continue;
		}
		asrflag=1;
		break;						//ASR���������ɹ����˳���ǰforѭ������ʼ�ȴ�LD3320�ͳ����ж��ź�
	}	
	return asrflag;
}

static void LD_reset(void)
{
	LD_RST_H();
	LD3320_delay(100);
	LD_RST_L();
	LD3320_delay(100);
	LD_RST_H();
	LD3320_delay(100);
	LD_CS_L();
	LD3320_delay(100);
	LD_CS_H();		
	LD3320_delay(100);
}

static void LD_AsrStart(void)
{
	LD_Init_ASR();
}

uint8 LD_Check_ASRBusyFlag_b2(void)
{
	uint8 j;
	uint8 flag = 0;
	for (j=0; j<10; j++)
	{
		if (LD_ReadReg(0xb2) == 0x21)
		{
			flag = 1;
			break;
		}
		LD3320_delay(10);		
	}
	return flag;
}
///�м��end


///�Ĵ�������
static uint8 spi_send_byte(uint8 byte)
{
	while (SPI_I2S_GetFlagStatus(LD3320SPI, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(LD3320SPI,byte);
	while (SPI_I2S_GetFlagStatus(LD3320SPI,SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(LD3320SPI);
}

static void LD_WriteReg(uint8 data1,uint8 data2)
{
	LD_CS_L();
	LD_SPIS_L();
	spi_send_byte(0x04);
	spi_send_byte(data1);
	spi_send_byte(data2);
	LD_CS_H();
}

static uint8 LD_ReadReg(uint8 reg_add)
{
	uint8 i;
	LD_CS_L();
	LD_SPIS_L();
	spi_send_byte(0x05);
	spi_send_byte(reg_add);
	i=spi_send_byte(0x00);
	LD_CS_H();
	return(i);
}

static uint8 LD_GetResult(void)
{
	return LD_ReadReg(0xc5);
}

static uint8 LD_AsrRun(void)
{
	LD_WriteReg(0x35, MIC_VOL);
	LD_WriteReg(0xB3, MIC_SEN);	// �û��Ķ� �����ֲ� ���B3�Ĵ����ĵ������������Ⱥ�ʶ������Ӱ��
	LD_WriteReg(0x1C, 0x09);
	LD_WriteReg(0xBD, 0x20);
	LD_WriteReg(0x08, 0x01);
	LD3320_delay( 5 );
	LD_WriteReg(0x08, 0x00);
	LD3320_delay( 5);

	if(LD_Check_ASRBusyFlag_b2() == 0)
	{
		return 0;
	}

	LD_WriteReg(0xB2, 0xff);	
	LD_WriteReg(0x37, 0x06);
	LD_WriteReg(0x37, 0x06);
	LD_WriteReg(0x37, 0x06);
	
	LD3320_delay(5);
	LD_WriteReg(0x1C, 0x0b);
	LD_WriteReg(0x29, 0x10);
	LD_WriteReg(0xBD, 0x00);   
	return 1;
}

static void ProcessInt(void)
{
	uint8 nAsrResCount=0;

	ucRegVal = LD_ReadReg(0x2B);

// ����ʶ��������ж�
//�����������룬����ʶ��ɹ���ʧ�ܶ����жϣ�
	LD_WriteReg(0x29,0) ;
	LD_WriteReg(0x02,0) ;

	if((ucRegVal & 0x10) && LD_ReadReg(0xb2)==0x21 && LD_ReadReg(0xbf)==0x35)		
	{	 
			nAsrResCount = LD_ReadReg(0xba);

			if(nAsrResCount>0 && nAsrResCount<=4) 
			{
				nAsrStatus=LD_ASR_FOUNDOK; 				
			}
			else
			{
				nAsrStatus=LD_ASR_FOUNDZERO;
			}	
	}
	else
	{
		nAsrStatus=LD_ASR_FOUNDZERO;//ִ��û��ʶ��
	}

	LD_WriteReg(0x2b,0);
	LD_WriteReg(0x1C,0);//д0:ADC������
	LD_WriteReg(0x29,0);
	LD_WriteReg(0x02,0);
	LD_WriteReg(0x2B,0);
	LD_WriteReg(0xBA,0);	
	LD_WriteReg(0xBC,0);	
	LD_WriteReg(0x08,1);//���FIFO_DATA
	LD_WriteReg(0x08,0);//���FIFO_DATA�� �ٴ�д0
}

static void LD_Init_Common(void)
{
	LD_ReadReg(0x06);  
	LD_WriteReg(0x17, 0x35); 
	LD3320_delay(5);
	LD_ReadReg(0x06);  

	LD_WriteReg(0x89, 0x03);  
	LD3320_delay(5);
	LD_WriteReg(0xCF, 0x43);   
	LD3320_delay(5);
	LD_WriteReg(0xCB, 0x02);
	
	/*PLL setting*/
	LD_WriteReg(0x11, LD_PLL_11);       
	if (nLD_Mode == LD_MODE_MP3)
	{
		LD_WriteReg(0x1E, 0x00); 
		LD_WriteReg(0x19, LD_PLL_MP3_19);   
		LD_WriteReg(0x1B, LD_PLL_MP3_1B);   
		LD_WriteReg(0x1D, LD_PLL_MP3_1D);
	}
	else
	{
		LD_WriteReg(0x1E,0x00);
		LD_WriteReg(0x19, LD_PLL_ASR_19); 
		LD_WriteReg(0x1B, LD_PLL_ASR_1B);		
	  LD_WriteReg(0x1D, LD_PLL_ASR_1D);
	}
	LD3320_delay(5);
	
	LD_WriteReg(0xCD, 0x04);
	LD_WriteReg(0x17, 0x4c); 
	LD3320_delay(1);
	LD_WriteReg(0xB9, 0x00);
	LD_WriteReg(0xCF, 0x4F); 
	LD_WriteReg(0x6F, 0xFF); 
}

static void LD_Init_ASR(void)
{
	nLD_Mode=LD_MODE_ASR_RUN;
	LD_Init_Common();

	LD_WriteReg(0xBD, 0x00);
	LD_WriteReg(0x17, 0x48);	
	LD3320_delay(5);
	LD_WriteReg(0x3C, 0x80);    
	LD_WriteReg(0x3E, 0x07);
	LD_WriteReg(0x38, 0xff);    
	LD_WriteReg(0x3A, 0x07);
	LD_WriteReg(0x40, 0);          
	LD_WriteReg(0x42, 8);
	LD_WriteReg(0x44, 0);    
	LD_WriteReg(0x46, 8); 
	LD3320_delay( 1 );
}
///�Ĵ������� end
/*********************************************END OF FILE**********************/
