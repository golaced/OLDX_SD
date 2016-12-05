#include "head.h"
#include "rtc.h"

//unsigned char time_buf1[8] = {20,9,3,13,18,51,00,6};//��������ʱ������
//unsigned char time_buf[8] ;                         //��������ʱ������
#define Yanshi 5

TimeStruct Writetime;
TimeStruct Readtime;

void DS1302_GPIO_Config(void)
{		
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*��������ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 

	/*ѡ��Ҫ���Ƶ�����*/															   
  	GPIO_InitStructure.GPIO_Pin = P_CLK|P_DAT|P_RST;	

	/*��������ģʽΪͨ���������*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*������������Ϊ50MHz */   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*���ÿ⺯������ʼ��*/
  	GPIO_Init(GPIOB, &GPIO_InitStructure);		  
	 
}
void IO_Output(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =P_CLK|P_DAT|P_RST;	  //PB0 --DS18B20_SCLK
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void IO_Input(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =P_DAT;  //PC5 DS1302 ���ݽӿ�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}	
		
//BCD����
void BCDconvert1(TimeStruct*p)
{
	p->year	=(((p->year) /10)<<4)+(p->year) %10;
	p->month=(((p->month)/10)<<4)+(p->month)%10;
	p->day	=(((p->day)	 /10)<<4)+(p->day)  %10;
	p->week	=(((p->week) /10)<<4)+(p->week) %10;
	p->hour	=(((p->hour) /10)<<4)+(p->hour) %10;
	p->min	=(((p->min)  /10)<<4)+(p->min)  %10;
	p->sec	=(((p->sec)	 /10)<<4)+(p->sec)  %10;

}
void BCDconvert2(TimeStruct*p)
{
	p->year	=(((p->year) /16)*10)+(p->year) %16;
	p->month=(((p->month)/16)*10)+(p->month)%16;
	p->day	=(((p->day)	 /16)*10)+(p->day)  %16;
	p->week	=(((p->week) /16)*10)+(p->week) %16;
	p->hour	=(((p->hour) /16)*10)+(p->hour) %16;
	p->min	=(((p->min)	 /16)*10)+(p->min)	%16;
	p->sec	=(((p->sec)	 /16)*10)+(p->sec)	%16;

}

/*------------------------------------------------
           ��DS1302д��һ�ֽ�����
------------------------------------------------*/
void Ds1302_Write_Byte(unsigned char addr, unsigned char d)
{

	unsigned char i;
	IO_Output();
	RST_SET;	
	
	//д��Ŀ���ַ��addr
	addr = addr & 0xFE;     //���λ����
	for (i = 0; i < 8; i ++) 
	    { 
		if (addr & 0x01) 
		    {
			IO_SET;
			}
		else 
		    {
			IO_CLR;
			}
		delay_us(Yanshi);
		SCK_SET;
		SCK_CLR;
		delay_us(Yanshi);
		addr = addr >> 1;
		}
	
	//д�����ݣ�d
	for (i = 0; i < 8; i ++) 
	   {
		if (d & 0x01) 
		    {
			IO_SET;
			}
		else 
		    {
			IO_CLR;
			}
		delay_us(Yanshi);
		SCK_SET;
		SCK_CLR;
		delay_us(Yanshi);
		d = d >> 1;
		}
	RST_CLR;					//ֹͣDS1302����
}
/*------------------------------------------------
           ��DS1302����һ�ֽ�����
------------------------------------------------*/

unsigned char Ds1302_Read_Byte(unsigned char addr) 
{

	unsigned char i;
	unsigned char temp;
	IO_Output();
	RST_SET;	

	//д��Ŀ���ַ��addr
	addr = addr | 0x01;//���λ�ø�
	for (i = 0; i < 8; i ++) 
	    {
	     
		if (addr & 0x01) 
		   {
			IO_SET;
			}
		else 
		    {
			IO_CLR;
			}
		delay_us(Yanshi);
		SCK_SET;
		SCK_CLR;
		delay_us(Yanshi);
		addr = addr >> 1;
		}
	
	//������ݣ�temp
	//IO������Ϊ����ģʽ����DS1302��ȡ����
	
	IO_Input();
	delay_us(8);

	for (i = 0; i < 8; i ++) 
	    {
		temp = temp >> 1;
		if (IO_R) 
		   {
			temp |= 0x80;
			}
		else 
		   {
			temp &= 0x7F;
			}
		delay_us(Yanshi);
		SCK_SET;
		SCK_CLR;
		delay_us(Yanshi);
		}
	
	RST_CLR;	//ֹͣDS1302����
	return temp;
}

/*------------------------------------------------
           ��DS1302д��ʱ������
------------------------------------------------*/
void Ds1302_Write_Time(void) 
{
     
    //unsigned char i,tmp;
	/*
	for(i=0;i<8;i++)
	    {                  //BCD����
		tmp=time_buf1[i]/10;
		time_buf[i]=time_buf1[i]%10;
		time_buf[i]=time_buf[i]+tmp*16;
	    }
	*/
	BCDconvert1(&Writetime);
Writetime.year=15;
Writetime.month=10;
Writetime.day=21;
Writetime.week=3;
Writetime.hour=12;
Writetime.min=7+6;
Writetime.sec=0;
	Ds1302_Write_Byte(ds1302_control_add,0x00);			//�ر�д���� 
	Ds1302_Write_Byte(ds1302_sec_add,0x80);				//��ͣ 
	//Ds1302_Write_Byte(ds1302_charger_add,0xa9);			//������ 
	Ds1302_Write_Byte(ds1302_year_add,	Writetime.year);		//�� 
	Ds1302_Write_Byte(ds1302_month_add,	Writetime.month);	//�� 
	Ds1302_Write_Byte(ds1302_date_add,	Writetime.day);		//�� 
	Ds1302_Write_Byte(ds1302_day_add,	Writetime.week);		//�� 
	Ds1302_Write_Byte(ds1302_hr_add,	Writetime.hour);		//ʱ 
	Ds1302_Write_Byte(ds1302_min_add,	Writetime.min);		//��
	Ds1302_Write_Byte(ds1302_sec_add,	Writetime.sec);		//��
	//Ds1302_Write_Byte(ds1302_day_add,time_buf[7]);		//�� 
	Ds1302_Write_Byte(ds1302_control_add,0x80);			//��д���� 
}

/*------------------------------------------------
           ��DS1302����ʱ������
------------------------------------------------*/
void Ds1302_Read_Time(void)  
{ 
   	    //unsigned char i,tmp;
	Readtime.year =Ds1302_Read_Byte(ds1302_year_add);		//�� 
	Readtime.month=Ds1302_Read_Byte(ds1302_month_add);		//�� 
	Readtime.day  =Ds1302_Read_Byte(ds1302_date_add);		//�� 
	Readtime.hour =Ds1302_Read_Byte(ds1302_hr_add);			//ʱ 
	Readtime.min  =Ds1302_Read_Byte(ds1302_min_add);		//�� 
	Readtime.sec  =Ds1302_Read_Byte(ds1302_sec_add);		//�� 
	Readtime.week =Ds1302_Read_Byte(ds1302_day_add);		//�� 

	/*
	for(i=0;i<8;i++)
	   {           //BCD����
		tmp=time_buf[i]/16;
		time_buf1[i]=time_buf[i]%16;
		time_buf1[i]=time_buf1[i]+tmp*10;
	   }
	 */
	BCDconvert2(&Readtime);
}

/*------------------------------------------------
                DS1302��ʼ��
------------------------------------------------*/
void Ds1302_Init(void)
{
	DS1302_GPIO_Config();
	
	RST_CLR;			//RST���õ�
	SCK_CLR;			//SCK���õ�
    Ds1302_Write_Byte(ds1302_sec_add,0x00);				 
}

void Time_Conver_char(u8 sel,u16 loc,u8 * s)
{u8 temp1[2];
u8 i;
//my_itoa(Readtime.year,temp1);
//for(i=0;i<2;i++)
//	*s++=temp1[i];
//	*s++='/';
switch(sel){
	case 0:{	
my_itoa(Readtime.month,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];
	*s++='/';
my_itoa(Readtime.day,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];
	*s++=' ';	
my_itoa(Readtime.hour,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];
	*s++=':';
my_itoa(Readtime.min,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];
	*s++=':';	
my_itoa(Readtime.sec,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];
	break;}
		case 1:{	//s
my_itoa( CARD[loc].time_s.month,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];
	*s++='/';
my_itoa(CARD[loc].time_s.day,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];
	*s++=' ';	
my_itoa(CARD[loc].time_s.hour,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];
	*s++=':';
my_itoa(CARD[loc].time_s.min,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];
	break;}
	case 2:{	//e
my_itoa(CARD[loc].time_e.month,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];
	*s++='/';
my_itoa(CARD[loc].time_e.day,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];
	*s++=' ';	
my_itoa(CARD[loc].time_e.hour,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];
	*s++=':';
my_itoa(CARD[loc].time_e.min,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];
	break;}
}
}
