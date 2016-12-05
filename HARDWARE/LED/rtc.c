#include "head.h"
#include "rtc.h"

//unsigned char time_buf1[8] = {20,9,3,13,18,51,00,6};//空年月日时分秒周
//unsigned char time_buf[8] ;                         //空年月日时分秒周
#define Yanshi 5

TimeStruct Writetime;
TimeStruct Readtime;

void DS1302_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOF时钟

  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = P_CLK|P_DAT|P_RST;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化
	
	
}
void IO_Output(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;


  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = P_CLK|P_DAT|P_RST;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化
	
}
void IO_Input(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	 GPIO_InitStructure.GPIO_Pin = P_DAT; //KEY0 KEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输出模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE2,3,4

}	
		
//BCD处理
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
           向DS1302写入一字节数据
------------------------------------------------*/
void Ds1302_Write_Byte(unsigned char addr, unsigned char d)
{

	unsigned char i;
	IO_Output();
	RST_SET;	
	
	//写入目标地址：addr
	addr = addr & 0xFE;     //最低位置零
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
	
	//写入数据：d
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
	RST_CLR;					//停止DS1302总线
}
/*------------------------------------------------
           从DS1302读出一字节数据
------------------------------------------------*/

unsigned char Ds1302_Read_Byte(unsigned char addr) 
{

	unsigned char i;
	unsigned char temp;
	IO_Output();
	RST_SET;	

	//写入目标地址：addr
	addr = addr | 0x01;//最低位置高
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
	
	//输出数据：temp
	//IO口设置为输入模式，从DS1302读取数据
	
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
	
	RST_CLR;	//停止DS1302总线
	return temp;
}

/*------------------------------------------------
           向DS1302写入时钟数据
------------------------------------------------*/
void Ds1302_Write_Time(void) 
{
     
    //unsigned char i,tmp;
	/*
	for(i=0;i<8;i++)
	    {                  //BCD处理
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
	Ds1302_Write_Byte(ds1302_control_add,0x00);			//关闭写保护 
	Ds1302_Write_Byte(ds1302_sec_add,0x80);				//暂停 
	//Ds1302_Write_Byte(ds1302_charger_add,0xa9);			//涓流充电 
	Ds1302_Write_Byte(ds1302_year_add,	Writetime.year);		//年 
	Ds1302_Write_Byte(ds1302_month_add,	Writetime.month);	//月 
	Ds1302_Write_Byte(ds1302_date_add,	Writetime.day);		//日 
	Ds1302_Write_Byte(ds1302_day_add,	Writetime.week);		//周 
	Ds1302_Write_Byte(ds1302_hr_add,	Writetime.hour);		//时 
	Ds1302_Write_Byte(ds1302_min_add,	Writetime.min);		//分
	Ds1302_Write_Byte(ds1302_sec_add,	Writetime.sec);		//秒
	//Ds1302_Write_Byte(ds1302_day_add,time_buf[7]);		//周 
	Ds1302_Write_Byte(ds1302_control_add,0x80);			//打开写保护 
}

/*------------------------------------------------
           从DS1302读出时钟数据
------------------------------------------------*/
void Ds1302_Read_Time(void)  
{ 
   	    //unsigned char i,tmp;
	Readtime.year =Ds1302_Read_Byte(ds1302_year_add);		//年 
	Readtime.month=Ds1302_Read_Byte(ds1302_month_add);		//月 
	Readtime.day  =Ds1302_Read_Byte(ds1302_date_add);		//日 
	Readtime.hour =Ds1302_Read_Byte(ds1302_hr_add);			//时 
	Readtime.min  =Ds1302_Read_Byte(ds1302_min_add);		//分 
	Readtime.sec  =Ds1302_Read_Byte(ds1302_sec_add);		//秒 
	Readtime.week =Ds1302_Read_Byte(ds1302_day_add);		//周 

	/*
	for(i=0;i<8;i++)
	   {           //BCD处理
		tmp=time_buf[i]/16;
		time_buf1[i]=time_buf[i]%16;
		time_buf1[i]=time_buf1[i]+tmp*10;
	   }
	 */
	BCDconvert2(&Readtime);
}

/*------------------------------------------------
                DS1302初始化
------------------------------------------------*/
void Ds1302_Init(void)
{
	DS1302_GPIO_Config();
	
	RST_CLR;			//RST脚置低
	SCK_CLR;			//SCK脚置低
    Ds1302_Write_Byte(ds1302_sec_add,0x00);				 
}



void my_itoar(int num,u8* str)  
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

