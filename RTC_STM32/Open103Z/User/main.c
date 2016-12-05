#include "head.h"
#include "table.h"
extern uint8_t write[];
extern uint8_t read[];
u8 state_lock[2];

u8 test;
int main(void)
{	u8 i;
		delay_init(72);		//—” ±≥ı ºªØ
		//OLED_Init();
		//delay_ms(1000);	
		//RS485_Init();
	  USART_init();
	  
		//delay_ms(1000);//	OLED_Fill(0x00);//OLED_P8x16Str(10,3,"UART Initdone");
    //Lock_init();
	

	 Ds1302_Write_Time();
	
  	Ds1302_Init();
		Time4ON();
	
	 __enable_irq();
		while(1){

	
		}
}
/*********************************************END OF FILE**********************/

