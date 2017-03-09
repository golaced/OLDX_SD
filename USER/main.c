#include "head.h"
u8 res=0;	
FIL fsrc, fdst;      /* file objects */
char path0[512]="0:";
char buffer[1000];   /* file copy buffer */
uint8_t textFileBuffer[] = "Test\r\n";
uint8_t textFileBuffer2[] ="Done\r\n";

void sd_write_9channal(float ch1,float ch2,float ch3,float ch4,float ch5,float ch6,float ch7,float ch8,float ch9);
void sd_write_sensor_test(float new_data,u8 ndigit);
void SD_READ_FILE(char * path);
u16 creat_floder(void);
void SD_Save_Task(u16 delay);
char *my_ftoa(double number,int ndigit,char *buf);
void show_sdcard_info(void)
{
	switch(SDCardInfo.CardType)
	{
		case SDIO_STD_CAPACITY_SD_CARD_V1_1:printf("Card Type:SDSC V1.1\r\n");break;
		case SDIO_STD_CAPACITY_SD_CARD_V2_0:printf("Card Type:SDSC V2.0\r\n");break;
		case SDIO_HIGH_CAPACITY_SD_CARD:printf("Card Type:SDHC V2.0\r\n");break;
		case SDIO_MULTIMEDIA_CARD:printf("Card Type:MMC Card\r\n");break;
	}	
  printf("Card ManufacturerID:%d\r\n",SDCardInfo.SD_cid.ManufacturerID);	//制造商ID
 	printf("Card RCA:%d\r\n",SDCardInfo.RCA);								//卡相对地址
	printf("Card Capacity:%d MB\r\n",(u32)(SDCardInfo.CardCapacity>>20));	//显示容量
 	printf("Card BlockSize:%d\r\r\n\n",SDCardInfo.CardBlockSize);			//显示块大小
}


void SD_INIT(void)
{
	res = 	f_mount(fs[0],"0:",1); 					//挂载SD卡 
	if(res != FR_OK){
		printf("mount filesystem 0 failed : %d\r\n",res);
	}
}

void SD_TEST(void)
{

	res = 	f_mount(fs[0],"0:",1); 					//挂载SD卡 f_mount(0,&fs);
	if(res != FR_OK){
		printf("mount filesystem 0 failed : %d\r\n",res);
	}
	//写文件测试
	printf("write file test......\r\n");
    res = f_open(&fdst, "0:/testnow.txt", FA_CREATE_ALWAYS | FA_WRITE);//以只写的方式打开，如果文件不存在的话则创建这个文件
	if(res != FR_OK){
		printf("open file error : %d\r\n",res);
	}else{
		 bw=55; 
	    res = f_write(&fdst, textFileBuffer, sizeof(textFileBuffer), &bw);               /* Write it to the dst file */
		if(res == FR_OK){
			printf("write data ok! write num=%d\r\n",bw);
		}else{
			printf("write data error : %d\r\n",res);
		}
		/*close file */
		f_close(&fdst);
	}

	//读文件测试
	printf("read file test......\r\n");
    res = f_open(&fsrc, "0:/testnow.txt", FA_OPEN_EXISTING | FA_READ);
    if(res != FR_OK){
		printf("open file error : %d\r\n",res);
	}else{
	    res = f_read(&fsrc, buffer, sizeof(textFileBuffer), &br);     /* Read a chunk of src file */
		if(res==FR_OK){
			printf("read data num : %d\r\n",br);
			printf("%s\r\n",buffer);
		}else{
			printf("read file error : %d\r\n",res);
		}
		/*close file */
		f_close(&fsrc);
	}
}

void Uart_zero_fix(char* buf ,u32 num)
{u32 i;
	for(i=0;i<num-1;i++)
			{
			 if(buf[i]==0x0)
				   buf[i]=' ';
		 }

}	

void SD_TEST2(void)
{
u16 i;
	res = 	f_mount(fs[0],"0:",1); 					//挂载SD卡 f_mount(0,&fs);
	if(res != FR_OK){
		printf("mount filesystem 0 failed : %d\r\n",res);
	}
	//写文件测试
	printf("write file test......\r\n");
	
    res = f_open(&fdst, "0:/testnow.txt", FA_OPEN_ALWAYS | FA_WRITE);//以只写的方式打开，如果文件不存在的话则创建这个文件
	if(res != FR_OK){
		printf("open file error : %d\r\n",res);
	}else{
		   f_lseek(&fdst,fdst.fsize);
	    res = f_write(&fdst, textFileBuffer2, sizeof(textFileBuffer2), &bw);               /* Write it to the dst file */
		if(res == FR_OK){
			printf("write data ok! write num=%d\r\n",bw);
		}else{
			printf("write data error : %d\r\n",res);
		}
	
		/*close file */
		f_close(&fdst);
	}

	//读文件测试
	printf("read file test......\r\n");
    res = f_open(&fsrc, "0:/testnow.txt", FA_OPEN_EXISTING | FA_READ);
    if(res != FR_OK){
		printf("open file error : %d\r\n",res);
	}else{
	    res = f_read(&fsrc, buffer, fsrc.fsize, &br);     /* Read a chunk of src file */
		if(res==FR_OK){
			printf("read data num : %d\r\n",br);
		 Uart_zero_fix(buffer,100);
			printf("%s\r\n",buffer);
		}else{
			printf("read file error : %d\r\n",res);
		}
		/*close file */
		f_close(&fsrc);
	}
}

volatile uint32_t lastUpdater, nowr; // 采样周期计数 单位 us
/**************************实现函数********************************************
*函数原型:		void Initial_Timer3(void)
*功　　能:	  初始化Tim2  Tim3 将两个定时器级联，以产生一个32位的定时器来提供系统us 级的计时	
输入参数：无
输出参数：没有	
*******************************************************************************/
void Initial_Timer_Sys(void)
{
  RCC->APB1ENR |= 0x0008;	//使能TIM5时钟
	TIM5->CR1 = 0x0080; //TIMx_ARR buffered  upcounter
	TIM5->CR2 = 0x0000;
	TIM5->CNT = 0x0000;
	TIM5->ARR = 0xFFFFFFFF;
	TIM5->PSC = 84 - 1;	//分出 1M 的时钟 保证每个周期为1us
	TIM5->EGR = 0x0001;
	TIM5->CR1 |= 0x0001; //启动定时器           
}

/**************************实现函数********************************************
*函数原型:		uint32_t micros(void)
*功　　能:	  读取系统运行的时间 ，返回单位为us 的时间数。	
输入参数：无
输出参数：处理器当前时间，从上电开始计时  单位 us
*******************************************************************************/
uint32_t micros(void)
{
 	uint32_t temp=0 ;
 	temp = TIM5->CNT;
 	return temp;
}
char data_conv_test[20];
u8 nrf_right;
int main(void)//---------------------------------------------------------------------------------------
{ float test_data=123.f; 
  	static u8 LED_STATE=0;
 	u32 total,free;
	u8 t=0;	


	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	Initial_Timer_Sys();
	uart_init(256000L);		//初始化测试串口
	uart2_init(256000L);//导航板数据
	uart3_init(256000L);//飞控板数据
	LED_Init();					//初始化LED 
 	KEY_Init();					//按键初始化 
  GPIO_ResetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);//GPIOF9,F10设置高，灯灭
	//delay_ms(500);	
	my_mem_init(SRAMIN);		//初始化内部内存池 
	my_mem_init(SRAMCCM);		//初始化CCM内存池

	GPIO_SetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);//GPIOF9,F10设置高，灯灭 
  //delay_ms(500);			
 	while(SD_Init())//检测不到SD卡
	{ printf("检测不到SD卡\r\n");			//显示块大小
		 delay_ms(200);		
	}
		show_sdcard_info();	//打印SD卡相关信息
		exfuns_init();							//为fatfs相关变量申请内存				 
	  SD_INIT();

	//SD_READ_FILE("0:/DATA_OUT/ODS_7/PID_HIGH.txt");
// 	Spi1_Init();		
//	Nrf24l01_Init(MODEL_TX,40,1);// ???  ???
//	Nrf24l01_Init(MODEL_RX,40,2);// ???  ???
//	Nrf24l01_Check(2);
//	nrf_right=Nrf24l01_Check(1);
	Nvic_Init();
	my_ftoa(-13.3,2,data_conv_test);
	my_ftoa(13.3,2,data_conv_test);
	my_ftoa(-13.3,5,data_conv_test);
	my_ftoa(13.3,5,data_conv_test);
	sd_had_init=1;
//	Ds1302_Init();
//	Ds1302_Write_Time();
//	Ds1302_Read_Time();	
	IWDG_Init(4,500*2); //与分频数为64,重载值为500,溢出时间为1s		
	TIM2_Int_Init(2000-1,84-1);	//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数5000次为500ms      TIM2_Config(

	while(1)
	{
  if(en_save)
		;//;GPIO_SetBits(GPIOE,GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);//GPIOF9,F10设置高，灯灭
		else
		{
//			switch(LED_STATE){
//				case 0:
//					GPIO_SetBits(GPIOE,GPIO_Pin_0 );
//					GPIO_ResetBits(GPIOE, GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);//GPIOF9,F10设置高，灯灭
//				  delay_ms(300);
//				LED_STATE++;
//				break;
//				case 1:
//					GPIO_SetBits(GPIOE,GPIO_Pin_1 );
//					GPIO_ResetBits(GPIOE,GPIO_Pin_0 |GPIO_Pin_2|GPIO_Pin_3);//GPIOF9,F10设置高，灯灭
//				  delay_ms(300);
//				LED_STATE++;
//				break;
//				case 2:
//					GPIO_SetBits(GPIOE,GPIO_Pin_2 );
//					GPIO_ResetBits(GPIOE,GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_3);//GPIOF9,F10设置高，灯灭
//				  delay_ms(300);
//				LED_STATE++;
//				break;
//				case 3:
//					GPIO_SetBits(GPIOE,GPIO_Pin_3 );
//					GPIO_ResetBits(GPIOE,GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2);//GPIOF9,F10设置高，灯灭
//				  delay_ms(300);
//				LED_STATE=0;
//				break;
//				}
		}	
		//SD_Save_Task(20);

	} 
}

char* my_itoa(int value,char *str,int radix)
{
	int sign = 0;
	//char *s = str;
	char ps[32];int i=0,j=0;
	memset(ps,0,32);
	
	if(value < 0)
	{
		sign = -1;
		value = -value;
	}
	do
	{
		if(value%radix>9)
			ps[i] = value%radix +'0'+7;
		else 
			ps[i] = value%radix +'0';
		i++;
	}while((value/=radix)>0);
	if(sign<0)
		ps[i] = '-';
	else
		i--;
	for(j=i;j>=0;j--)
	{
		str[i-j] = ps[j];
	}
	return str;
}

char *my_ftoa(double number,int ndigit,char *buf)
{
  long int_part;int i=0;
	double float_part;
	char str_int[32];
	char str_float[32];
	memset(str_int,0,32);
	memset(str_float,0,32);
	int_part = (long)number;
	float_part = number - int_part;
	my_itoa(int_part,str_int,10);
	if(ndigit>0)
	{
		float_part = pow(10.0,(double)ndigit)*fabs(float_part);
		my_itoa((long)float_part,str_float,10);
	}
	 i= strlen(str_int);
	str_int[i] = '.';
	strcat(str_int,str_float);
	strcpy(buf,str_int);
	return buf;
}



void SD_READ_FILE(char * path)
{
u16 i;
char buffer_rx[1000]={0};   /* file copy buffer */
//	res = 	f_mount(fs[0],"0:",1); 					//挂载SD卡 f_mount(0,&fs);
	//读文件测试
		printf("read file:%s\r\n",path);
    res = f_open(&fsrc, path, FA_OPEN_EXISTING | FA_READ);
    if(res != FR_OK){
		//printf("open file error : %d\r\n",res);
	}else{
	    res = f_read(&fsrc, buffer_rx, fsrc.fsize, &br);     /* Read a chunk of src file */
		if(res==FR_OK){
		//	printf("read data num : %d\r\n",br);
		 Uart_zero_fix(buffer_rx,sizeof(buffer_rx));
			printf("%s\r\n",buffer_rx);
		}else{
		//	printf("read file error : %d\r\n",res);
		}
		/*close file */
		f_close(&fsrc);
	}
}


u8 BUF_TIME[14];
u16 creat_floder(void)
{	char file_name[20]={"0:/DATA_OUT"};
	u8 res,i,j,k;					 
	u16 index=0;
   res=f_mkdir(file_name);//!=FR_EXIST);		//创建PHOTO文件夹
for (k=0;k<20;k++)
	file_name[k]=0;
 for(index=0;index<1024;index++)
{ 
			Time_Conver_char(0,0,BUF_TIME);	
														for(i=0;i<15;i++)
														 if(BUF_TIME[i]==0)
																 BUF_TIME[i]=' ';
														 BUF_TIME[13]=0;
	 //sprintf(file_name,"0:/DATA_OUT/X%d_%s",index,BUF_TIME);												 
	sprintf(file_name,"0:/DATA_OUT/ODS_%d",index);
   res=f_mkdir(file_name);//!=FR_EXIST);		//创建PHOTO文件夹
 	for (k=0;k<20;k++)
	file_name[k]=0;
	
	if(res==FR_OK)
		break;
	
	}

return 	index;
} 
u8 en_save=0,init_sd_write=0,sdcard_force_out=0,sd_had_init=0;

void sd_write(float *ch,char *file_name,u16 index, u32 t_now,u8 max_num,u8 point)
{	char folder[40]={0};
u16 i=0,j=0,size=0,k=0,l=0;
char data_conv1[20]={0};
char data_write[200]={0};	
float halfT;
static u8 init;

	my_ftoa(t_now,1,data_conv1);
	
	while(data_conv1[i]!= '.')
	{data_write[i]=data_conv1[i];i++;}
	data_write[i++]='m';
	data_write[i++]='s';
	data_write[i++]=' ';
	data_write[i++]=' ';
	
for(l=0;l<max_num;l++){//--ch write task
	for (k=0;k<20;k++)
	data_conv1[k]=0;
	my_ftoa(*ch++,point,data_conv1);
	while(data_conv1[j]!= '\0')
	data_write[i++]=data_conv1[j++];	j=0;
	data_write[i++]=' ';
	data_write[i++]=' ';
}
  data_write[i++]='\r';
	data_write[i++]='\n';

	while(data_write[size]!= '\0')
	size++;
	//--写入文件夹
	//sprintf(folder,"0:/DATA_OUT/X%d_%s",index,BUF_TIME);			
		sprintf(folder,"0:/DATA_OUT/ODS_%d",index);
	for(i=0;i<40;i++)
	 if(folder[i]==0)
		   break;
	 
	 folder[i++]='/';
	 for(j=0;j<20;j++)
	 { folder[i+j]=file_name[j];
	 }
	 //--
	  res = f_open(&fdst, folder, FA_OPEN_ALWAYS | FA_WRITE);//以只写的方式打开，如果文件不存在的话则创建这个文件
	if(res != FR_OK){
		printf("open file error : %d\r\n",res);
		//en_save=0;
	}else{
		  
		   f_lseek(&fdst,fdst.fsize);
	    res = f_write(&fdst, data_write,size, &bw);               /* Write it to the dst file */
		if(res == FR_OK){
		//	printf("write data ok! write num=%d\r\n",bw);
		}else{
			printf("write data error : %d\r\n",res);
		}
	
		/*close file */
		f_close(&fdst);
	}
}

long t_sd_save;
void SD_Save_Task(u16 delay){
	
static u8 state=0;
static u16 index=0;
float ch[20]={0};
float halfT;
static float test_data[9];
u8 delta_t;
static u32 t_now;
 nowr = micros();  //读取时间
  if(nowr<lastUpdater){ //定时器溢出过了。
  halfT =  ((float)(nowr + (0xffff- lastUpdater)) / 1000000.0f);
  }
  else	{
  halfT =  ((float)(nowr - lastUpdater) / 1000000.0f);
  }
  lastUpdater = nowr;	//更新时间
	delta_t=halfT*1000;
	
switch (state)
{
	case 0:
	if(en_save)
  state=1;		
	break;
	case 1:
		index=creat_floder();
	  t_now=0;
	  state=2;
	break;
	case 2:
	if(!en_save)
  {	init_sd_write=0;t_now=0;state=0;}
	else
	{u8 cnt_;
		 if(en_save)
		GPIO_SetBits(GPIOE,GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);//GPIOF9,F10设置高，灯灭
	 cnt_=0;	
	

//ALT UKF		 
//	 ch[cnt_++]=fly_controller.imu.roll;
//	 ch[cnt_++]=fly_controller.imu.pitch;
//	 ch[cnt_++]=fly_controller.imu.yaw;
//	 ch[cnt_++]=qr_matlab_data_att[0];
//	 ch[cnt_++]=qr_matlab_data_att[1];
//	 ch[cnt_++]=qr_matlab_data_att[2];
//	 ch[cnt_++]=fly_controller.imu.acc_x;
//	 ch[cnt_++]=fly_controller.imu.acc_y;
//	 ch[cnt_++]=fly_controller.imu.acc_z;
//		 
//	ch[cnt_++]=fly_controller.now.alt_sonar ;//z sonar
//	ch[cnt_++]=fly_controller.now.alt_fushion_sonar ;//z sonar acc
//	ch[cnt_++]=qr_matlab_data[3];//z qr
//	ch[cnt_++]=k_scale_pix;
//	 sd_write(ch,"imu_fly_board.txt", index, t_now,cnt_,1);cnt_=0;
//  HML CAL	
//	 ch[cnt_++]=fly_controller.imu.roll;
//	 ch[cnt_++]=fly_controller.imu.pitch;
//	 ch[cnt_++]=fly_controller.imu.yaw;
//	 ch[cnt_++]=fly_controller.sensor.hmx;
//	 ch[cnt_++]=fly_controller.sensor.hmy;
//	 ch[cnt_++]=fly_controller.sensor.hmz;
//	 ch[cnt_++]=fly_controller.sensor.hmx_c;
//	 ch[cnt_++]=fly_controller.sensor.hmy_c;
//	 ch[cnt_++]=fly_controller.sensor.hmz_c;
//	 sd_write(ch,"hml.txt", index, t_now,cnt_,1);cnt_=0;
		
//  FLOW QR		
//		ch[cnt_++]=fly_controller.flow.spd_f[0];
//		ch[cnt_++]=fly_controller.flow.spd_f[1];
//		ch[cnt_++]=fly_controller.flow.pos_t[0];
//		ch[cnt_++]=fly_controller.flow.pos_t[1];
//		ch[cnt_++]=fly_controller.flow.pos[0];
//		ch[cnt_++]=fly_controller.flow.pos[1];
//		ch[cnt_++]=fly_controller.flow.spd[0];
//		ch[cnt_++]=fly_controller.flow.spd[1];
//	  ch[cnt_++]=flow_matlab_data[0];
//		ch[cnt_++]=flow_matlab_data[1];
//		ch[cnt_++]=flow_matlab_data[2];
//		ch[cnt_++]=flow_matlab_data[3];	
//	 	ch[cnt_++]=baro_matlab_data[0];
//		ch[cnt_++]=baro_matlab_data[1];
//		ch[cnt_++]=baro_matlab_data[2];
//		ch[cnt_++]=baro_matlab_data[3];
//		ch[cnt_++]=qr_matlab_data[0];
//		ch[cnt_++]=qr_matlab_data[1];
//		ch[cnt_++]=qr_matlab_data[2];

//		
//	 sd_write(ch,"flow_sensor.txt", index, t_now,cnt_,3);cnt_=0;
//GPS
	 
//	  ch[cnt_++]=flow_matlab_data[0];
//		ch[cnt_++]=flow_matlab_data[1];
//		ch[cnt_++]=flow_matlab_data[2];
//		ch[cnt_++]=flow_matlab_data[3];
//		ch[cnt_++]=(float)fly_controller.gps.J/10000000.;	
//		ch[cnt_++]=(float)fly_controller.gps.W/10000000.;			
//		ch[cnt_++]=fly_controller.gps.gps_mode;
//		ch[cnt_++]=fly_controller.gps.spd;
//		ch[cnt_++]=fly_controller.gps.angle;
//		ch[cnt_++]=fly_controller.gps.yaw;
//		ch[cnt_++]=fly_controller.gps.star_num;
//		ch[cnt_++]=fly_controller.gps.svnum;
//		sd_write(ch,"gps.txt", index, t_now,cnt_,7);cnt_=0;
//		
//		
//		ch[cnt_++]=qr_matlab_data[0];
//		ch[cnt_++]=qr_matlab_data[1];
//		ch[cnt_++]=qr_matlab_data[2];
//		ch[cnt_++]=fly_controller.now.alt_fushion_sonar;
//		sd_write(ch,"bai.txt", index, t_now,cnt_,7);cnt_=0;
// HML YAW_CAL
	  ch[cnt_++]=fly_controller.imu.roll;
		ch[cnt_++]=fly_controller.imu.pitch;
		ch[cnt_++]=fly_controller.imu.yaw;
		
		ch[cnt_++]=fly_controller.sensor.hmx_c;
		ch[cnt_++]=fly_controller.sensor.hmy_c;
		ch[cnt_++]=fly_controller.sensor.hmz_c;
		ch[cnt_++]=fly_controller.imu.acc_x;
		ch[cnt_++]=fly_controller.imu.acc_y;
		ch[cnt_++]=fly_controller.imu.acc_z;
		sd_write(ch,"hml_yaw.txt", index, t_now,cnt_,2);cnt_=0;
		
		
//		ch[cnt_++]=qr_matlab_data[0];
//		ch[cnt_++]=qr_matlab_data[1];
//		ch[cnt_++]=qr_matlab_data[2];
//		ch[cnt_++]=fly_controller.now.alt_fushion_sonar;
//		sd_write(ch,"bai.txt", index, t_now,cnt_,7);cnt_=0;
//		
//	 sd_write(ch,"baro_sensor.txt", index, t_now,4,3);
	 
	 //ch[0]=1;ch[1]=2;ch[2]=3;ch[3]=4;ch[4]=5;ch[5]=6;ch[6]=7;ch[7]=8;ch[8]=9;
	 //sd_write(ch,"imu_nav_board.txt", index, t_now);
		
	 //ch[0]=1;ch[1]=2;ch[2]=3;ch[3]=4;ch[4]=5;ch[5]=6;ch[6]=7;ch[7]=8;ch[8]=9;	
	 //sd_write(ch,"sensor_fly_board.txt", index, t_now);	
		
	 //ch[0]=1;ch[1]=2;ch[2]=3;ch[3]=4;ch[4]=5;ch[5]=6;ch[6]=7;ch[7]=8;ch[8]=9;	
	 //sd_write(ch,"sensor_nav_board.txt", index, t_now);		
		

//		ch[0]=fly_controller.set.roll ;
//		ch[1]=fly_controller.set.pitch ;
//		ch[2]=fly_controller.set.yaw ;
//		ch[3]=fly_controller.imu.roll;
//		ch[4]=fly_controller.imu.pitch;
//		ch[5]=fly_controller.imu.yaw;
//		sd_write(ch,"pid_att.txt", index, t_now,6,1);	

//		ch[0]=fly_controller.set.alt;ch[1]=fly_controller.now.alt;
//		ch[2]=fly_controller.set.spd_alt;ch[3]=fly_controller.now.spd_alt;
//		ch[4]=fly_controller.now.thr;ch[5]=fly_controller.now.alt_bmp;
//		ch[6]=fly_controller.now.alt_fushion;
//		sd_write(ch,"pid_alt.txt", index, t_now,7,1);			

//		ch[0]=fly_controller.set.pos[0];ch[1]=fly_controller.set.pos[1];
//		ch[2]=fly_controller.now.pos[0];ch[3]=fly_controller.now.pos[1];
//		ch[4]=fly_controller.set.spd[0];ch[5]=fly_controller.set.spd[1];
//		ch[6]=fly_controller.now.spd[0];ch[7]=fly_controller.now.spd[1];
//		ch[8]=fly_controller.now.nav[0];ch[9]=fly_controller.now.nav[1];
//		sd_write(ch,"pid_flow.txt", index, t_now,10,1);	

//		ch[0]=fly_controller.flow.spd[0];ch[1]=fly_controller.flow.spd[1];
//		ch[2]= fly_controller.flow.spd_x;ch[3]= fly_controller.flow.spd_y;
//		ch[4]=fly_controller.flow.pos[0];ch[5]=fly_controller.flow.pos[1];
//		sd_write(ch,"fly_flow_data.txt", index, t_now,6,1);


//		ch[0]=fly_controller.slam_sonar[0];ch[1]=fly_controller.slam_sonar[1];
//		ch[2]=fly_controller.slam_sonar[2];ch[3]=fly_controller.slam_sonar[3];
//		ch[4]=fly_controller.slam_sonar[4];
//		sd_write(ch,"slam_sonar.txt", index, t_now,5,1);
		
//		ch[0]=fly_controller.gps.W/100000;ch[1]=fly_controller.gps.J/100000;
//		ch[2]=fly_controller.gps.Y_O/100;ch[3]=fly_controller.gps.X_O/100;
//		ch[4]=fly_controller.gps.Y_UKF/100;ch[5]=fly_controller.gps.X_UKF/100;
//		sd_write(ch,"gps_data.txt", index, t_now,6,6);	


//		ch[0]=fly_controller.imu.gro_x;ch[1]=fly_controller.imu.gro_y;ch[2]=fly_controller.imu.gro_z;
//		ch[3]=fly_controller.imu.acc_x;ch[4]=fly_controller.imu.acc_y;ch[5]=fly_controller.imu.acc_z;
//		ch[6]=fly_controller.imu.q0;ch[7]=fly_controller.imu.q1;ch[8]=fly_controller.imu.q2;ch[9]=fly_controller.imu.q3;
//		sd_write(ch,"imu_sensor_data.txt", index, t_now,10,1);	



	 //ch[0]=1;ch[1]=2;ch[2]=3;ch[3]=4;ch[4]=5;ch[5]=6;ch[6]=7;ch[7]=8;ch[8]=9;	
	 //sd_write(ch,"gps.txt", index, t_now);		
	  t_now=t_now+delta_t;
		t_sd_save=(float)t_now/1000.;
		//delay_ms(delay);
	}
	
	break;
	default:state=0;en_save=0;init_sd_write=0;t_now=0;break;

}	
}