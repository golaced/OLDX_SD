#ifndef _RC_H_
#define _RC_H_
#include "head.h"

typedef   signed short     int int16_t;
typedef struct int16_rcget{
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
				int16_t AUX1;
				int16_t AUX2;
				int16_t AUX3;
				int16_t AUX4;
				int16_t AUX5;
				u8 RST;}RC_GETDATA;

extern RC_GETDATA Rc_Get;//????RC??,1000~2000
typedef struct PID_STA{u16 OP,OI,OD,IP,II,ID,YP,YI,YD;}PID_STA;
extern float RC_Target_ROL,RC_Target_PIT,RC_Target_YAW;
extern PID_STA HPID,SPID;
void Nrf_Check_Event1(void);
void Nrf_Check_Event2(void);
void NRF_Send_AF(void);
void NRF_Send_AE(void);
void NRF_Send_OFFSET(void);
void NRF_Send_PID(void);
void NRF_Send_ARMED(void);
void NRF_SEND_test(void);
extern unsigned int cnt_timer2;
				typedef struct {
	      u8  NRF24L01_RXDATA[33];
	      u8  NRF24L01_TXDATA[33];
	      int16_t rc_data[4];
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
				int16_t SENSITIVITY;}RC_GETDATA2;
extern RC_GETDATA2 RC_Data;		
extern void RC_Send_Task(void);
extern float fix_pitch,fix_rol,control_scale;
extern u8 key_rc[6];;
#endif
