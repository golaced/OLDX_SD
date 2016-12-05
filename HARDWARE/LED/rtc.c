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
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOFʱ��

  //GPIOF9,F10��ʼ������
  GPIO_InitStructure.GPIO_Pin = P_CLK|P_DAT|P_RST;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��
	
	
}
void IO_Output(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;


  //GPIOF9,F10��ʼ������
  GPIO_InitStructure.GPIO_Pin = P_CLK|P_DAT|P_RST;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��
	
}
void IO_Input(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	 GPIO_InitStructure.GPIO_Pin = P_DAT; //KEY0 KEY1 KEY2��Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOE2,3,4

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
Writetime.day=16+6;
Writetime.week=5;
Writetime.hour=8;
Writetime.min=47+13+8+6;
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



void my_itoar(int num,u8* str)  
{  
    int sign = num;  
    int i = 0;  
    int j = 0;  
    char temp[100];  
    //����Ǹ�����ȥ������,��-1234ת��1234  
    if(sign < 0)  
    {  
        num = -num;  
    }  
    //ת���ַ�����1234ת��"4321"  
    do  
    {  
        temp[i] = num % 10 + '0';  
        num /= 10;  
        i++;  
    }while(num > 0);  
    //����Ǹ����Ļ����Ӹ�������ĩβ���磺"4321-"  
    if(sign < 0)  
    {  
        temp[i++] = '-';  
    }  
    temp[i] = '\0';  
    i--;  
    //��temp�������������뵽str������  
    //��"4321-" ====> "-1234"  
    while(i >= 0)  
    {  
        str[j] = temp[i];  
        j++;  
        i--;  
    }  
    //�ַ���������ʶ  
    str[j] = '\0';   
}  


void Time_Conver_char(u8 sel,u16 loc,u8 * s)
{u8 temp1[2];
u8 i;
//my_itoa(Readtime.year,temp1);
//for(i=0;i<2;i++)
//	*s++=temp1[i];
//	*s++='/';

my_itoar(Readtime.month,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];
	*s++='_';
my_itoar(Readtime.day,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];
	*s++='_';	
my_itoar(Readtime.hour,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];
	*s++='_';
my_itoar(Readtime.min,temp1);
for(i=0;i<2;i++)
	*s++=temp1[i];



}

