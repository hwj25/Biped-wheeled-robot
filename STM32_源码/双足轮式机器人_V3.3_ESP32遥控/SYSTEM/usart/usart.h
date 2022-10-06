#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define APPCONTROLMODE 1
#define DEBUGMODE 1	  	

extern u8 USART1_APP_Date[20];


void uart_init(u32 bound);

void virtual_Osc_send_array_float(float *array,u8 len);

void Usart1_UpdataAPP();

#endif


