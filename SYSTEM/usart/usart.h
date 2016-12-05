#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.csom
//�޸�����:2011/6/14
//�汾��V1.4
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
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


