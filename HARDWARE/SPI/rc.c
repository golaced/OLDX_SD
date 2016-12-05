/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * ???  :RC.c
 * ??    :???????         
 * ????:Air Nano?????
 * ???  :ST3.5.0
 * ??    :Air Nano Team 
 * ??    :http://byd2.taobao.com
**********************************************************************************/
#include "head.h"


#define RX_DR			6		//????
#define TX_DS			5
#define MAX_RT		4

RC_GETDATA Rc_Get;//????RC??,1000~2000

RC_GETDATA2 RC_Data;//????RC??,1000~2000
float RC_Target_ROL=0,RC_Target_PIT=0,RC_Target_YAW=0,RC_Target_THR=0;
vs16 QH,ZY,XZ;
u8 is_lock=1;
u8 EN_FIX_GPSF=0;
u8 EN_FIX_LOCKWF=0;
u8 EN_CONTROL_IMUF=0;
u8 EN_FIX_INSF=0;
u8 EN_FIX_HIGHF=0;
u8 tx_lock=1;
u8 EN_FIX_GPS=0;
u8 EN_FIX_LOCKW=0;
u8 EN_CONTROL_IMU=0;
u8 EN_FIX_INS=0;
u8 EN_FIX_HIGH=0;
u8 EN_TX_GX=1;
u8 EN_TX_AX=1;
u8 EN_TX_HM=1;
u8 EN_TX_YRP=1;
u8 EN_TX_GPS=1;
u8 EN_TX_HIGH=1;
u8 up_load_set=0;
u8 up_load_pid=0;
u8 key_rc[6];
u16 Yaw_sb_rc=0;


PID_STA HPID,SPID,FIX_PID;

float fix_pitch,fix_rol;
#define control_by_sb 1

u8 cnt_rst=0,delta_pitch=0,delta_roll=0,delta_yew=0;




float control_scale=1;
void NRF_DataAnl(void)
{
	u8 i=0,sum = 0;
	for( i=0;i<31;i++)
		sum += NRF24L01_RXDATA[i];
	if(!(sum==NRF24L01_RXDATA[31]))		return;		//??sum
	if(!(NRF24L01_RXDATA[0]==0x8A))		return;		//????
	
	if(NRF24L01_RXDATA[1]==0x8A)								//?????,=0x8a,?????
	{
		Rc_Get.THROTTLE = (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		Rc_Get.YAW			= (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		Rc_Get.PITCH		= (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		Rc_Get.ROLL 		= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		RC_Data.YAW			= (vs16)((NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6])/3+1000;
		RC_Data.PITCH		= (vs16)((NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8])/3+1000;
		RC_Data.ROLL 		= (vs16)((NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10])/3+1000;
		Rc_Get.AUX1			= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];
		Rc_Get.AUX2			= (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		Rc_Get.AUX3			= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		Rc_Get.AUX4			= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];
		Rc_Get.AUX5			= (vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20];
		Rc_Get.RST			= (vs16)NRF24L01_RXDATA[21];
		key_rc[0]=(NRF24L01_RXDATA[22])&0x01;
		key_rc[1]=(NRF24L01_RXDATA[22]>>1)&0x01;
		key_rc[2]=(NRF24L01_RXDATA[22]>>2)&0x01;
		key_rc[3]=(NRF24L01_RXDATA[22]>>3)&0x01;
		key_rc[4]=(NRF24L01_RXDATA[22]>>4)&0x01;
		key_rc[5]=(NRF24L01_RXDATA[22]>>5)&0x01;
		Yaw_sb_rc=(vs16)(NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24];
		
		
	 }
	else if(NRF24L01_RXDATA[1]==0x8B)								//?????,=0x8a,?????
	{
			tx_lock=(NRF24L01_RXDATA[3]);
			EN_FIX_GPS=(NRF24L01_RXDATA[4]);
			EN_FIX_LOCKW=(NRF24L01_RXDATA[5]);
			EN_CONTROL_IMU=(NRF24L01_RXDATA[6]);
			EN_FIX_INS=(NRF24L01_RXDATA[7]);
			EN_FIX_HIGH=(NRF24L01_RXDATA[8]);
			EN_TX_GX=(NRF24L01_RXDATA[9]);
			EN_TX_AX=(NRF24L01_RXDATA[10]);
			EN_TX_HM=(NRF24L01_RXDATA[11]);
			EN_TX_YRP=(NRF24L01_RXDATA[12]);
			EN_TX_GPS=(NRF24L01_RXDATA[13]);
			EN_TX_HIGH=(NRF24L01_RXDATA[14]);
			(up_load_set)=(NRF24L01_RXDATA[15]);
			(up_load_pid)=(NRF24L01_RXDATA[16]);
		
	EN_FIX_GPSF=EN_FIX_GPS;
	EN_FIX_LOCKWF=EN_FIX_GPS;
	EN_CONTROL_IMUF=EN_FIX_GPS;
	EN_FIX_INSF=EN_FIX_GPS;
	EN_FIX_HIGHF=EN_FIX_GPS;	
	}
	else 		if(NRF24L01_RXDATA[1]==0x8C)	//??PID1
		{
		  
			SPID.OP = (float)((vs16)(NRF24L01_RXDATA[4]<<8)|NRF24L01_RXDATA[5]);
			SPID.OI = (float)((vs16)(NRF24L01_RXDATA[6]<<8)|NRF24L01_RXDATA[7]);
			SPID.OD = (float)((vs16)(NRF24L01_RXDATA[8]<<8)|NRF24L01_RXDATA[9]);
		  SPID.IP = (float)((vs16)(NRF24L01_RXDATA[10]<<8)|NRF24L01_RXDATA[11]);
			SPID.II = (float)((vs16)(NRF24L01_RXDATA[12]<<8)|NRF24L01_RXDATA[13]);
			SPID.ID = (float)((vs16)(NRF24L01_RXDATA[14]<<8)|NRF24L01_RXDATA[15]);
		  SPID.YP = (float)((vs16)(NRF24L01_RXDATA[16]<<8)|NRF24L01_RXDATA[17]);
			SPID.YI = (float)((vs16)(NRF24L01_RXDATA[18]<<8)|NRF24L01_RXDATA[19]);
			SPID.YD = (float)((vs16)(NRF24L01_RXDATA[20]<<8)|NRF24L01_RXDATA[21]);
	
				
				
			
		//	EE_SAVE_PID();
		}
	else 		if(NRF24L01_RXDATA[1]==0x8D)	//??PID2
		{
			HPID.OP = (float)((vs16)(NRF24L01_RXDATA[4]<<8)|NRF24L01_RXDATA[5]);
			HPID.OI = (float)((vs16)(NRF24L01_RXDATA[6]<<8)|NRF24L01_RXDATA[7]);
			HPID.OD = (float)((vs16)(NRF24L01_RXDATA[8]<<8)|NRF24L01_RXDATA[9]);
			
		//	EE_SAVE_PID();
		}

	

}


int cnt_timer2_r=0;
u8 cnt_led_rx=0;
void Nrf_Check_Event1(void)
{ static uint8_t led2_state = 0;
	u8 rx_len =0;
	static u16 cnt_loss_rc=0;
	u8 sta;

	sta= NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS,1);		//??2401?????
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<RX_DR))	//??ing 
	{ 
		cnt_loss_rc=0;
			
				
				
				rx_len= NRF_Read_Reg(R_RX_PL_WID,1);
				
				if(rx_len<33)	//??????33?????,???????
				{
					NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,RX_PLOAD_WIDTH,1);// read receive payload from RX_FIFO buffer
					NRF_DataAnl();	//??2401??????
				
				}
				else 
				{ 
					NRF_Write_Reg(FLUSH_RX,0xff,1);//?????
				}
			
	}
	else//---------losing_nrf
	 {	
		 if(cnt_loss_rc++>200 )//0.5ms
		 {	NRF_Write_Reg(FLUSH_RX,0xff,1);//?????
	
			 
		 }
  }
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<TX_DS))	//????,?????
	{
	
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	
	if(sta & (1<<MAX_RT))//??,????
	{
		if(led2_state)
		{
			led2_state = 0;
		}
		else
		{
			led2_state = 1;
		}
		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF_Write_Reg(FLUSH_TX,0xff,1);
		}
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta,1);
}


void Nrf_Check_Event2(void)
{ static uint8_t led2_state = 0;
	u8 rx_len =0;
	static u16 cnt_loss_rc=0;
	u8 sta;

	sta= NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS,2);		//??2401?????
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<RX_DR))	//??ing 
	{ 
		cnt_loss_rc=0;
			
				
				
				rx_len= NRF_Read_Reg(R_RX_PL_WID,2);
				
				if(rx_len<33)	//??????33?????,???????
				{
					NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,RX_PLOAD_WIDTH,2);// read receive payload from RX_FIFO buffer
					NRF_DataAnl();	//??2401??????
				
				}
				else 
				{ 
					NRF_Write_Reg(FLUSH_RX,0xff,2);//?????
				}
			
	}
	else//---------losing_nrf
	 {	
		 if(cnt_loss_rc++>200 )//0.5ms
		 {	NRF_Write_Reg(FLUSH_RX,0xff,2);//?????
	
			 
		 }
  }
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<TX_DS))	//????,?????
	{
	
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	
	if(sta & (1<<MAX_RT))//??,????
	{
		if(led2_state)
		{
			led2_state = 0;
		}
		else
		{
			led2_state = 1;
		}
		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF_Write_Reg(FLUSH_TX,0xff,2);
		}
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta,2);
}
void NRF_Send_ARMED(void)//????
{
	uint8_t i,sum;
	u32 _temp;
	u8 cnt=0;

	NRF24L01_TXDATA[cnt++]=0x88;
	NRF24L01_TXDATA[cnt++]=0xAC;
	NRF24L01_TXDATA[cnt++]=0x1C;

	sum = 0;
	for(i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[31]=sum;
	
	NRF_TxPacket(NRF24L01_TXDATA,32,1);
}

void NRF_Send_RC_PID1(void)//????
{	vs16 _temp;	u8 i;	u8 sum = 0;

	NRF24L01_TXDATA[0] = 0x88;	//?????8BSET??
	NRF24L01_TXDATA[1] = 0x8C;	//?????8BSET??
	NRF24L01_TXDATA[2] = 0x1C;
	NRF24L01_TXDATA[3]=0xAD;
	
	_temp =SPID.OP;
	NRF24L01_TXDATA[4]=BYTE1(_temp);
	NRF24L01_TXDATA[5]=BYTE0(_temp);
	_temp =SPID.OI;
	NRF24L01_TXDATA[6]=BYTE1(_temp);
	NRF24L01_TXDATA[7]=BYTE0(_temp);
	_temp = SPID.OD;
	NRF24L01_TXDATA[8]=BYTE1(_temp);
	NRF24L01_TXDATA[9]=BYTE0(_temp);
	_temp =SPID.IP;
	NRF24L01_TXDATA[10]=BYTE1(_temp);
	NRF24L01_TXDATA[11]=BYTE0(_temp);
	_temp =SPID.II;
	NRF24L01_TXDATA[12]=BYTE1(_temp);
	NRF24L01_TXDATA[13]=BYTE0(_temp);
	_temp = SPID.ID;
	NRF24L01_TXDATA[14]=BYTE1(_temp);
	NRF24L01_TXDATA[15]=BYTE0(_temp);
	_temp = SPID.YP;//.YP * 1;
	NRF24L01_TXDATA[16]=BYTE1(_temp);
	NRF24L01_TXDATA[17]=BYTE0(_temp);
	_temp = SPID.YI;//SPID.YI * 1;
	NRF24L01_TXDATA[18]=BYTE1(_temp);
	NRF24L01_TXDATA[19]=BYTE0(_temp);
  _temp = SPID.YD;//.YD * 1;
	NRF24L01_TXDATA[20]=BYTE1(_temp);
	NRF24L01_TXDATA[21]=BYTE0(_temp);
	
	

	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
	NRF_TxPacket(NRF24L01_TXDATA,32,1);
}

void NRF_Send_RC_PID2(void)//????
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0;

	NRF24L01_TXDATA[_cnt++] = 0x88;	//?????8BSET??
	NRF24L01_TXDATA[_cnt++] = 0x8D;	//?????8BSET??
	NRF24L01_TXDATA[_cnt++] = 0x1C;
	NRF24L01_TXDATA[_cnt++]=	0xAD;
	_temp = 	  HPID.OP;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		_temp = 	HPID.OI;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		_temp = 	HPID.OD;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		_temp = 	fix_pitch*100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		_temp = 	fix_rol*100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	
	
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32,1);
}

void NRF_Send_RC_Sensor(void)//????
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0;

	NRF24L01_TXDATA[_cnt++] = 0x88;	//?????8BSET??
	NRF24L01_TXDATA[_cnt++] = 0x8E;	//?????8BSET??
	NRF24L01_TXDATA[_cnt++] = 0x1C;
	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  0;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);

	_temp =  0;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  0;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  0;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  0;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32,1);
}
	
void RC_Send_Task(void)
{
static u16 cnt[4]={0,0,0,0};

if(cnt[0]++>1)
{NRF_Send_ARMED();//????
 cnt[0]=0;}
if(cnt[1]++>1)
{	
 NRF_Send_RC_Sensor();//????
 cnt[1]=0;
}
if(cnt[2]++>2)
{ NRF_Send_RC_PID1();//????
 cnt[2]=0;}

if(cnt[3]++>2)
{ NRF_Send_RC_PID2();//????
 cnt[3]=0;}


}
