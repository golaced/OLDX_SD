#include "sys.h"
#include "usart.h"	
#include "head.h" 
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F4探索者开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/6/10
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
 void Data_Receive_Anl_uart2(u8 *data_buf,u8 num);
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){ //test_uart
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
#if EN_USART1_RX	
	

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif
	
}

 void Data_Receive_Anl1(u8 *data_buf,u8 num)//nav_board
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	if(*(data_buf+2)==0x01)								//判断功能字0x80 导航IMU数据
	{
	
			Readtime.year = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
			Readtime.month = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			Readtime.day = ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
			Readtime.hour = ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
			Readtime.min = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
			Readtime.sec = ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));

	}
}

	u8 RxBuffer1[50];
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
u8 com_data;
static u8 RxState = 0;
static u8 RxBufferNum = 0;
static u8 RxBufferCnt = 0;
static u8 RxLen = 0;
static u8 _data_len = 0,_data_cnt = 0;
		if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)==SET)
	{
						USART_ClearFlag(USART1,USART_FLAG_ORE); //读SR其实就是清除标志
						USART_ReceiveData(USART1);    //读DR
	 }
  else if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		com_data =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
					if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer1[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)//AA AF _导航板帧头
		{
			RxState=2;
			RxBuffer1[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer1[2]=com_data;
		}
		else if(RxState==3&&com_data<50)
		{
			RxState = 4;
			RxBuffer1[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer1[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer1[4+_data_cnt]=com_data;
			Data_Receive_Anl1(RxBuffer1,_data_cnt+5);
		}
		else
			RxState = 0;
	
  } 

} 


 
void UARTSendByByter1(u8 Data)
{
		//发送数据	
		USART_SendData(USART1, (u8) Data);
while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
;
}


void Driver_UARTSendByByters1(char * msg,u16 num)
{  u16 i=0;
	
    while(i< num)
    {
        UARTSendByByter1(* msg++);
			  i++;
    }  
}


//初始化IO 串口1 
//bound:波特率
void uart2_init(u32 bound){ //uart_nav
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2| GPIO_Pin_3; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART2, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断

}
void UARTSendByByter2(u8 Data)
{
		//发送数据	
		USART_SendData(USART2, (u8) Data);
while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
;
}


void Driver_UARTSendByByters2(char * msg,u16 num)
{  u16 i=0;
	
    while(i< num)
    {
        UARTSendByByter2(* msg++);
			  i++;
    }  
}


	u8 RxBuffer2[50];
void USART2_IRQHandler(void)                	//串口1中断服务程序
{
	u8 com_data;

static u8 RxState = 0,led_flag,cnt;
static u8 RxBufferNum = 0;
static u8 RxBufferCnt = 0;
static u8 RxLen = 0;
static u8 _data_len = 0,_data_cnt = 0;
		if(USART_GetFlagStatus(USART2,USART_FLAG_ORE)==SET)
	{
						USART_ClearFlag(USART2,USART_FLAG_ORE); //读SR其实就是清除标志
						USART_ReceiveData(USART2);    //读DR
	 }
  else if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{ USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志
		com_data =USART_ReceiveData(USART2);//(USART1->DR);	//读取接收到的数据
					if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer2[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)//AA AF _导航板帧头
		{
			RxState=2;
			RxBuffer2[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer2[2]=com_data;
		}
		else if(RxState==3&&com_data<50)
		{
			RxState = 4;
			RxBuffer2[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer2[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer2[4+_data_cnt]=com_data;
			Data_Receive_Anl_uart2(RxBuffer2,_data_cnt+5);
			if(!en_save){
				if(cnt++>2){cnt=0;
				 if(led_flag){led_flag=0;
					GPIO_SetBits(GPIOE,GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);}
				 else{led_flag=1;
					 GPIO_ResetBits(GPIOE,GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);}
				 }
			 }
		}
		else
			RxState = 0;
	
  } 

} 
void data_check_float(float *data,float *data_r,float in,float min,float max)
{
if(in<max&&in>min)
	{*data=in;*data_r=in;}
}

void data_check_int(int *data,int* data_r,int in,float min,float max)
{
if(in<max&&in>min)
	{*data=in;*data_r=in;}
}
float flow_matlab_data[4],baro_matlab_data[4],qr_matlab_data[4],qr_matlab_data_att[3];
struct _FLY fly_controller,fly_controller_r;
int k_scale_pix;
 void Data_Receive_Anl_uart2(u8 *data_buf,u8 num)//fly_board
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i,j;

	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		
	{
		j++;return;	
	}		//判断帧头
	if(*(data_buf+2)==0x05)								//判断功能字0x80 飞控IMU数据
	{
	     
			data_check_float(&fly_controller.imu.roll,&fly_controller_r.imu.roll,(float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/10,-180,180);
			data_check_float(&fly_controller.imu.pitch , &fly_controller_r.imu.pitch,(float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10,-180,180);
			data_check_float(&fly_controller.imu.yaw, &fly_controller_r.imu.yaw,(float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/10,-180,180);
			data_check_int(&fly_controller.imu.gro_x, &fly_controller_r.imu.gro_x,((vs16)(*(data_buf+10)<<8)|*(data_buf+11)),-1024,1024);
			data_check_int(&fly_controller.imu.gro_y, &fly_controller_r.imu.gro_y,((vs16)(*(data_buf+12)<<8)|*(data_buf+13)),-1024,1024);
			data_check_int(&fly_controller.imu.gro_z, &fly_controller_r.imu.gro_z,((vs16)(*(data_buf+14)<<8)|*(data_buf+15)),-1024,1024);
			
			data_check_int(&fly_controller.imu.acc_x, &fly_controller_r.imu.acc_x,((vs16)(*(data_buf+16)<<8)|*(data_buf+17)),-6000,6000);
			data_check_int(&fly_controller.imu.acc_y, &fly_controller_r.imu.acc_y,((vs16)(*(data_buf+18)<<8)|*(data_buf+19)),-6000,6000);
			data_check_int(&fly_controller.imu.acc_z, &fly_controller_r.imu.acc_z,((vs16)(*(data_buf+20)<<8)|*(data_buf+21)),-6000,6000);
			
			data_check_int(&fly_controller.imu.q0,		&fly_controller_r.imu.q0,((vs16)(*(data_buf+22)<<8)|*(data_buf+23)),-2000,2000);
			data_check_int(&fly_controller.imu.q1, 		&fly_controller_r.imu.q1,((vs16)(*(data_buf+24)<<8)|*(data_buf+25)),-2000,2000);
			data_check_int(&fly_controller.imu.q2, 		&fly_controller_r.imu.q2,((vs16)(*(data_buf+26)<<8)|*(data_buf+27)),-2000,2000);
			data_check_int(&fly_controller.imu.q3, 		&fly_controller_r.imu.q3,((vs16)(*(data_buf+28)<<8)|*(data_buf+29)),-2000,2000);
			data_check_int(&fly_controller.sensor.hmx,		&fly_controller_r.sensor.hmx_c,((vs16)(*(data_buf+30)<<8)|*(data_buf+31)),-1000,1000);
			data_check_int(&fly_controller.sensor.hmy, 		&fly_controller_r.sensor.hmy_c,((vs16)(*(data_buf+32)<<8)|*(data_buf+33)),-1000,1000);
			data_check_int(&fly_controller.sensor.hmz, 		&fly_controller_r.sensor.hmz_c,((vs16)(*(data_buf+34)<<8)|*(data_buf+35)),-1000,1000);
			data_check_int(&fly_controller.sensor.hmx_c,		&fly_controller_r.sensor.hmx_c,((vs16)(*(data_buf+36)<<8)|*(data_buf+37)),-1000,1000);
			data_check_int(&fly_controller.sensor.hmy_c, 		&fly_controller_r.sensor.hmy_c,((vs16)(*(data_buf+38)<<8)|*(data_buf+39)),-1000,1000);
			data_check_int(&fly_controller.sensor.hmz_c, 		&fly_controller_r.sensor.hmz_c,((vs16)(*(data_buf+40)<<8)|*(data_buf+41)),-1000,1000);
		   en_save = *(data_buf+42);//sd_使能存储		

	}
	else if(*(data_buf+2)==0x01)								//判断功能字0x82 飞控姿态PID控制量
	{
	
			fly_controller.set.pitch = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/10;
			fly_controller.set.roll = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10;
			fly_controller.set.yaw=(float) ((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/10;
	
	}
	else if(*(data_buf+2)==0x02)								//飞控光流PID控制
	{  fly_controller.set.pos[0]= ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
		 fly_controller.set.pos[1]= ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
		 fly_controller.now.pos[0]= ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
		 fly_controller.now.pos[1]= ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
		 fly_controller.set.spd[0]= ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
		 fly_controller.set.spd[1]= ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
		 fly_controller.now.spd[0]= ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
		 fly_controller.now.spd[1]= ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
		 fly_controller.now.nav[0]= (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/10;
		 fly_controller.now.nav[1]= (float)((vs16)(*(data_buf+22)<<8)|*(data_buf+23))/10;
		
	}
		else if(*(data_buf+2)==0x03)								//飞控高度PID控制
	{
	
			data_check_int(&fly_controller.set.alt ,&fly_controller_r.set.alt , ((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/10,-10*100,10*100);
		  data_check_int(&fly_controller.now.alt ,&fly_controller_r.now.alt , ((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10,-10*100,10*100);
		  data_check_float(&fly_controller.set.spd_alt,&fly_controller_r.set.spd_alt , ((vs16)(*(data_buf+8)<<8)|*(data_buf+9)),-500,500);
	    fly_controller.now.spd_alt = ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.now.thr = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
		  data_check_int(&fly_controller.now.alt_bmp ,&fly_controller_r.now.alt_bmp , ((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/10,-10*100,10*100);
		  data_check_int(&fly_controller.now.alt_fushion_sonar ,&fly_controller_r.now.alt_fushion_sonar , ((vs16)(*(data_buf+16)<<8)|*(data_buf+17)),0,10000);
		   baro_matlab_data[2]= ((vs16)(*(data_buf+19)<<8)|*(data_buf+20));
			 baro_matlab_data[3]= ((vs16)(*(data_buf+21)<<8)|*(data_buf+22));
		data_check_int(&fly_controller.now.alt_sonar ,&fly_controller_r.now.alt_sonar , ((vs16)(*(data_buf+23)<<8)|*(data_buf+25)),0,10000);
			 
	}
			else if(*(data_buf+2)==0x04)								//飞控模式
	{
	
			en_save = *(data_buf+4);//sd_使能存储		
	}
			else if(*(data_buf+2)==0x06)								//飞控使用光流数据
	{
	
			 fly_controller.flow.spd_f[0] = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
	     fly_controller.flow.spd_f[1] = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			 fly_controller.flow.spd[0] =  ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
	     fly_controller.flow.spd[1] =  ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
		   fly_controller.flow.pos_t[0] = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
	     fly_controller.flow.pos_t[1] = ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
		   fly_controller.flow.pos[0] = ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
	     fly_controller.flow.pos[1] = ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
			 flow_matlab_data[0]= ((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
			 flow_matlab_data[1]= ((vs16)(*(data_buf+22)<<8)|*(data_buf+23));
			 flow_matlab_data[2]= ((vs16)(*(data_buf+24)<<8)|*(data_buf+25));
			 flow_matlab_data[3]= ((vs16)(*(data_buf+26)<<8)|*(data_buf+27));
		   baro_matlab_data[0]= ((vs16)(*(data_buf+28)<<8)|*(data_buf+29));
			 baro_matlab_data[1]= ((vs16)(*(data_buf+30)<<8)|*(data_buf+31));
		   qr_matlab_data[0]= *(data_buf+32);
			 qr_matlab_data[1]= ((vs16)(*(data_buf+33)<<8)|*(data_buf+34));
			 qr_matlab_data[2]= ((vs16)(*(data_buf+35)<<8)|*(data_buf+36));
			 qr_matlab_data[3]= ((vs16)(*(data_buf+35)<<8)|*(data_buf+36));
			 qr_matlab_data_att[0]= ((vs16)(*(data_buf+37)<<8)|*(data_buf+38));
			 qr_matlab_data_att[1]= ((vs16)(*(data_buf+39)<<8)|*(data_buf+40));
			 qr_matlab_data_att[2]= ((vs16)(*(data_buf+41)<<8)|*(data_buf+42));
       k_scale_pix= ((vs16)(*(data_buf+43)<<8)|*(data_buf+44));
		
	}
				else if(*(data_buf+2)==0x07)								//飞控使用光流数据
	{
	
			 fly_controller.slam_sonar[0] = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
	     fly_controller.slam_sonar[1] = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			 fly_controller.slam_sonar[2] =  ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
	     fly_controller.slam_sonar[3] =  ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
		   fly_controller.slam_sonar[4] = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));	
	}
				else if(*(data_buf+2)==0x09)								//飞控使用GPS数据
	{
	
	fly_controller.gps.J = 			  ((long)(*(data_buf+4)<<24)|(*(data_buf+5)<<16)|(*(data_buf+6)<<8)|*(data_buf+7));//W
	fly_controller.gps.W = 			  ((long)(*(data_buf+8)<<24)|(*(data_buf+9)<<16)|(*(data_buf+10)<<8)|*(data_buf+11));//J
	fly_controller.gps.spd=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/100.;	
	fly_controller.gps.angle=(float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/10.;		
	fly_controller.gps.gps_mode =  *(data_buf+16);//W
	fly_controller.gps.star_num =  *(data_buf+17);//J	
	fly_controller.gps.svnum=*(data_buf+18);
	fly_controller.gps.yaw=(float)((int16_t)(*(data_buf+19)<<8)|*(data_buf+20))/10.;	
	}
	//---------------------------------------------------------------------
	else if(*(data_buf+2)==0x71)								//判断功能字0x81 飞控Sensor数据
	{
			fly_controller.sensor.accx = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.sensor.accy = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.sensor.accz= ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.sensor.grox = ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.sensor.groy = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.sensor.groz = ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
			fly_controller.sensor.hmx = ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
			fly_controller.sensor.hmy = ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
			fly_controller.sensor.hmz = ((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
		  fly_controller.sensor.bmp = ((vs16)(*(data_buf+22)<<8)|*(data_buf+23));
		  fly_controller.sensor.temp = ((vs16)(*(data_buf+24)<<8)|*(data_buf+25));
		  fly_controller.sensor.sonar = ((vs16)(*(data_buf+26)<<8)|*(data_buf+27));
	}
		else if(*(data_buf+2)==0x74)								//判断功能字0x82 PID参数
	{
	
			fly_controller.pid.pp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.pi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.pd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.pp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.pi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.pd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
		  fly_controller.pid.rp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.ri_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.rd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.rp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.ri_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.rd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
		  fly_controller.pid.yp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.yi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.yd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.yp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.yi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.yd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
			
			fly_controller.pid.hp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.hi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.hd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.hp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.hi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.hd_i=((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
	}
		else if(*(data_buf+2)==0x74)								//判断功能字0x82 PID参数
	{
	
			fly_controller.pid.pp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.pi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.pd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.pp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.pi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.pd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
		  fly_controller.pid.rp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.ri_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.rd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.rp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.ri_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.rd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
		  fly_controller.pid.yp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.yi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.yd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.yp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.yi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.yd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
			
			fly_controller.pid.hp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.hi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.hd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.hp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.hi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.hd_i=((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
	}
}


 struct _NAV nav;
 void Data_Receive_Anl2(u8 *data_buf,u8 num)//nav_board
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	if(*(data_buf+2)==0x80)								//判断功能字0x80 导航IMU数据
	{
	
			nav.imu.pitch = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
			nav.imu.roll = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			nav.imu.yaw= ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
			nav.imu.J = (long)(*(data_buf+10)<<24|*(data_buf+11)<<16|*(data_buf+12)<<8|*(data_buf+13));
			nav.imu.W = (long)(*(data_buf+14)<<24|*(data_buf+15)<<16|*(data_buf+16)<<8|*(data_buf+17));
			nav.imu.H = ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));

	}
	else if(*(data_buf+2)==0x81)								//判断功能字0x81 导航Sensor数据
	{
			nav.sensor.accx = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
			nav.sensor.accy = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			nav.sensor.accz= ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
			nav.sensor.grox = ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
			nav.sensor.groy = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
			nav.sensor.groz = ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
			nav.sensor.hmx = ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
			nav.sensor.hmy = ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
			nav.sensor.hmz = ((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
		  nav.sensor.bmp = ((vs16)(*(data_buf+22)<<8)|*(data_buf+23));
		  nav.sensor.sonar = ((vs16)(*(data_buf+24)<<8)|*(data_buf+25));
		  nav.sensor.temp = ((vs16)(*(data_buf+26)<<8)|*(data_buf+27));
		
	}
	else if(*(data_buf+2)==0x82)								//判断功能字0x82 导航Fushion数据
	{
	
			nav.fushion.alt = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
			nav.fushion.spd = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			nav.fushion.alt_acc_bmp= ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
			nav.fushion.alt_acc_sonar = ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
			nav.fushion.spd_acc_bmp = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
			nav.fushion.spd_acc_sonar = ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
		
	}
		else if(*(data_buf+2)==0x83)								//判断功能字0x82 导航Fushion数据
	{
	
			nav.imu_nav.alt = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
			nav.imu_nav.spd_w = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			nav.imu_nav.spd_e= ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
			nav.imu_nav.spd_h = ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
			nav.imu_nav.spd_x = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
			nav.imu_nav.spd_y = ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
			nav.imu_nav.DJ = (long)(*(data_buf+10)<<24|*(data_buf+11)<<16|*(data_buf+12)<<8|*(data_buf+13));
			nav.imu_nav.DW = (long)(*(data_buf+14)<<24|*(data_buf+15)<<16|*(data_buf+16)<<8|*(data_buf+17));
			nav.imu_nav.GJ = (long)(*(data_buf+10)<<24|*(data_buf+11)<<16|*(data_buf+12)<<8|*(data_buf+13));
			nav.imu_nav.GW = (long)(*(data_buf+14)<<24|*(data_buf+15)<<16|*(data_buf+16)<<8|*(data_buf+17));
		
	}
}


void UsartSend2(uint8_t ch)
{
	USART_SendData(USART2, (uint8_t) ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
	{}
}

void Send_Data2(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend2(dataToSend[i]);
}


void Send_1(void)
{u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
	u8 data_to_send[50];
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x00;//功能字
	data_to_send[_cnt++]=0;//校验
	_temp = 0*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data2(data_to_send, _cnt);
}
//--------------------------------------------------------------------------------------------------------
//初始化IO 串口1 
//bound:波特率
void uart3_init(u32 bound){ //uart_fly
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART3, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断

}
void UARTSendByByter3(u8 Data)
{
		//发送数据	
		USART_SendData(USART3, (u8) Data);
while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
;
}


void Driver_UARTSendByByters3(char * msg,u16 num)
{  u16 i=0;
	
    while(i< num)
    {
        UARTSendByByter3(* msg++);
			  i++;
    }  
}
 void Data_Receive_Anl3(u8 *data_buf,u8 num);//fly_board
u8 RxBuffer3[50];
void USART3_IRQHandler(void)                	//串口1中断服务程序
{
	u8 com_data;
	static u8 RxState = 0;
static u8 RxBufferNum = 0;
static u8 RxBufferCnt = 0;
static u8 RxLen = 0;
static u8 _data_len = 0,_data_cnt = 0;
		if(USART_GetFlagStatus(USART3,USART_FLAG_ORE)==SET)
	{
						USART_ClearFlag(USART3,USART_FLAG_ORE); //读SR其实就是清除标志
						USART_ReceiveData(USART3);    //读DR
	 }
  else if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志
			com_data =USART_ReceiveData(USART3);//(USART1->DR);	//读取接收到的数据
					if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer3[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)
		{
			RxState=2;
			RxBuffer3[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer3[2]=com_data;
		}
		else if(RxState==3&&com_data<50)
		{
			RxState = 4;
			RxBuffer3[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer3[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer3[4+_data_cnt]=com_data;
			Data_Receive_Anl3(RxBuffer3,_data_cnt+5);
		}
		else
			RxState = 0;
	
  } 
   

} 

 void Data_Receive_Anl3(u8 *data_buf,u8 num)//fly_board
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i,j;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		
	{
		j++;return;		//判断sum
	}
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		
	{
		j++;return;	
	}		//判断帧头
	if(*(data_buf+2)==0x05)								//判断功能字0x80 飞控IMU数据
	{
	
			fly_controller.imu.roll = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/10;
			fly_controller.imu.pitch = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10;
			fly_controller.imu.yaw= (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/10;
			fly_controller.imu.gro_x= ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.imu.gro_y= ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.imu.gro_z= ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
			
			fly_controller.imu.acc_x= ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
			fly_controller.imu.acc_y= ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
			fly_controller.imu.acc_z= ((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
			
			fly_controller.imu.q0=		((vs16)(*(data_buf+22)<<8)|*(data_buf+23));
			fly_controller.imu.q1= 		((vs16)(*(data_buf+24)<<8)|*(data_buf+25));
			fly_controller.imu.q2= 		((vs16)(*(data_buf+26)<<8)|*(data_buf+27));
			fly_controller.imu.q3= 		((vs16)(*(data_buf+28)<<8)|*(data_buf+29));

	}
	else if(*(data_buf+2)==0x01)								//判断功能字0x82 飞控姿态PID控制量
	{
	
			fly_controller.set.pitch = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/10;
			fly_controller.set.roll = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10;
			fly_controller.set.yaw=(float) ((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/10;
	
	}
	else if(*(data_buf+2)==0x02)								//飞控光流PID控制
	{  fly_controller.set.pos[0]= ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
		 fly_controller.set.pos[1]= ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
		 fly_controller.now.pos[0]= ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
		 fly_controller.now.pos[1]= ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
		 fly_controller.set.spd[0]= ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
		 fly_controller.set.spd[1]= ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
		 fly_controller.now.spd[0]= ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
		 fly_controller.now.spd[1]= ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
		 fly_controller.now.nav[0]= (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/10;
		 fly_controller.now.nav[1]= (float)((vs16)(*(data_buf+22)<<8)|*(data_buf+23))/10;
	}
		else if(*(data_buf+2)==0x03)								//飞控高度PID控制
	{
	
			fly_controller.set.alt = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
		  fly_controller.now.alt = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
		  fly_controller.set.spd_alt = ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
	    fly_controller.now.spd_alt = ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.now.thr = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
		  fly_controller.now.alt_bmp = ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
		  fly_controller.now.alt_fushion = ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
		
	}
			else if(*(data_buf+2)==0x04)								//飞控模式
	{
	
			en_save = *(data_buf+4);//sd_使能存储
	
	
		
	}
			else if(*(data_buf+2)==0x06)								//飞控使用光流数据
	{
	
			 fly_controller.flow.spd[0] = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
	     fly_controller.flow.spd[1] = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			 fly_controller.flow.spd_x =  ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
	     fly_controller.flow.spd_y =  ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
		   fly_controller.flow.pos[0] = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
	     fly_controller.flow.pos[1] = ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
	
		
	}
				else if(*(data_buf+2)==0x07)								//飞控使用光流数据
	{
	
			 fly_controller.slam_sonar[0] = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
	     fly_controller.slam_sonar[1] = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			 fly_controller.slam_sonar[2] =  ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
	     fly_controller.slam_sonar[3] =  ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
		   fly_controller.slam_sonar[4] = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));	
	}
				else if(*(data_buf+2)==0x08)								//飞控使用GPS数据
	{
	
	fly_controller.gps.J = 			  ((long)(*(data_buf+4)<<24)|(*(data_buf+5)<<16)|(*(data_buf+6)<<8)|*(data_buf+7));//W
	fly_controller.gps.W = 			  ((long)(*(data_buf+8)<<24)|(*(data_buf+9)<<16)|(*(data_buf+10)<<8)|*(data_buf+11));//J
	fly_controller.gps.gps_mode =  *(data_buf+12);//W
	fly_controller.gps.star_num =  *(data_buf+13);//J
	fly_controller.gps.X_O = 			((long)(*(data_buf+14)<<24)|(*(data_buf+15)<<16)|(*(data_buf+16)<<8)|*(data_buf+17));//W
	fly_controller.gps.Y_O = 			((long)(*(data_buf+18)<<24)|(*(data_buf+19)<<16)|(*(data_buf+20)<<8)|*(data_buf+21));//J
	fly_controller.gps.X_UKF = 		((long)(*(data_buf+22)<<24)|(*(data_buf+23)<<16)|(*(data_buf+24)<<8)|*(data_buf+25));//W
	fly_controller.gps.Y_UKF = 		((long)(*(data_buf+26)<<24)|(*(data_buf+27)<<16)|(*(data_buf+28)<<8)|*(data_buf+29));//J
	}
	//---------------------------------------------------------------------
	else if(*(data_buf+2)==0x71)								//判断功能字0x81 飞控Sensor数据
	{
			fly_controller.sensor.accx = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.sensor.accy = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.sensor.accz= ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.sensor.grox = ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.sensor.groy = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.sensor.groz = ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
			fly_controller.sensor.hmx = ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
			fly_controller.sensor.hmy = ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
			fly_controller.sensor.hmz = ((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
		  fly_controller.sensor.bmp = ((vs16)(*(data_buf+22)<<8)|*(data_buf+23));
		  fly_controller.sensor.temp = ((vs16)(*(data_buf+24)<<8)|*(data_buf+25));
		  fly_controller.sensor.sonar = ((vs16)(*(data_buf+26)<<8)|*(data_buf+27));
	}
		else if(*(data_buf+2)==0x74)								//判断功能字0x82 PID参数
	{
	
			fly_controller.pid.pp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.pi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.pd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.pp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.pi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.pd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
		  fly_controller.pid.rp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.ri_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.rd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.rp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.ri_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.rd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
		  fly_controller.pid.yp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.yi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.yd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.yp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.yi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.yd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
			
			fly_controller.pid.hp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.hi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.hd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.hp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.hi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.hd_i=((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
	}
		else if(*(data_buf+2)==0x74)								//判断功能字0x82 PID参数
	{
	
			fly_controller.pid.pp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.pi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.pd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.pp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.pi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.pd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
		  fly_controller.pid.rp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.ri_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.rd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.rp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.ri_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.rd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
		  fly_controller.pid.yp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.yi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.yd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.yp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.yi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.yd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
			
			fly_controller.pid.hp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.hi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.hd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.hp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.hi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.hd_i=((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
	}
}
