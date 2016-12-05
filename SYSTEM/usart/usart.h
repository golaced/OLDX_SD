#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.csom
//修改日期:2011/6/14
//版本：V1.4
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
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
////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
extern void Driver_UARTSendByByters1(char * msg,u16 num);
extern float flow_matlab_data[4],baro_matlab_data[4],qr_matlab_data[4],qr_matlab_data_att[3];
extern int k_scale_pix;
//nav_board
struct _IMU{
		float pitch;
		float roll;
		float yaw;
	  int gro_x;
		int gro_y;
		int gro_z;
		int acc_x;
		int acc_y;
		int acc_z;
		int q0;
		int q1;
		int q2;
		int q3;
		long J;
		long W;
		int H;
	      };

struct _SENSOR{
	int accx;
	int accy;
	int accz;
	int grox;
	int groy;
	int groz;
	int hmx;
	int hmy;
	int hmz;
	int hmx_c;
	int hmy_c;
	int hmz_c;
	int bmp;
	int temp;
	int sonar;
	      };

struct _FUSHION{
	int alt;
	int spd;
	int alt_acc_bmp;
	int alt_acc_sonar;
	int spd_acc_bmp;
	int spd_acc_sonar;
	      };
struct _IMU_NAV{
	int alt;
	int spd_w;
	int spd_e;
	int spd_h;
	int spd_x;
	int spd_y;
	long DJ,DW;
	long GJ,GW;
	      };				
struct _NAV{
				struct _IMU imu;    
				struct _SENSOR sensor;  
				struct _FUSHION fushion; 
				struct _IMU_NAV imu_nav;
	      u8 fame;
            };

extern struct _NAV nav;
						
struct _SET{
float pitch;
float roll;
float yaw;
int alt;
int alt_bmp;
int alt_fushion;
int spd_x;
int spd_y;
int pos[2];
int pos_t[2];
int spd[2];
int spd_f[2];	
float nav[2];
float spd_alt;
int thr;
int alt_sonar;	
int alt_fushion_sonar;

		};

struct _STATE{
u8 alt_data ;
u8 gps ;
u8 flow;
u8 imu_nav;
u8 pid_fuzzy ;
		};	

struct _PID{
u16 pp_o;
u16 pi_o;
u16 pd_o;
u16 pp_i;
u16 pi_i;
u16 pd_i;

u16 rp_o;
u16 ri_o;
u16 rd_o;
u16 rp_i;
u16 ri_i;
u16 rd_i;

u16 yp_o;
u16 yi_o;
u16 yd_o;
u16 yp_i;
u16 yi_i;
u16 yd_i;

u16 hp_o;
u16 hi_o;
u16 hd_o;
u16 hp_i;
u16 hi_i;
u16 hd_i;

		};	

struct _PIDOUT{
int pitch;
int roll;
int yaw;
int alt;
int spd_x;
int spd_y;

		};	
struct _POS_GPS_NAV {
long J;
long W;
long X_O;
long Y_O;
long X_UKF;
long Y_UKF;
u8 gps_mode;
u8 star_num;
u8 svnum;
float angle;
float spd;
float yaw;
};
struct _FLY{
				struct _IMU imu;    
				struct _SENSOR sensor;  
				struct _SET set; 
				struct _SET flow; 	
				struct _SET now;  
	      struct _STATE state;
	      struct _PID pid;
	      struct _PIDOUT pid_out;
				struct _POS_GPS_NAV gps;
	      int slam_sonar[5];
	      u8 fame;
            };

extern struct _FLY fly_controller;
 void Data_Receive_Anl2(u8 *data_buf,u8 num);//nav_board
						
void uart2_init(u32 bound);					
void uart3_init(u32 bound);
#endif


