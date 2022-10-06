#ifndef __LORA_USART2_H
#define __LORA_USART2_H	 
#include "sys.h"

void LORA_USART2_init(u32 bound);
void USART2_Receive_Prepare(u8 data);
void USART2_Receive_Data(u8 data[],u8 i);
//void LORA_Read();

extern u8 USART2_LORA_date[15];

		 				    
#endif



