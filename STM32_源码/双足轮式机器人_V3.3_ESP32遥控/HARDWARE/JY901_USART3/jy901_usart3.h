#ifndef __JY901_USART3_H
#define __JY901_USART3_H	 
#include "sys.h"

void JY901_USART3_init(u32 bound);
void USART3_Receive_Prepare(u8 data);
void USART3_Receive_Data(u8 data[],u8 i);
void JY910_Set();

extern float Aacx,Aacy,Aacz;
extern float Gyrox,Gyroy,Gyroz;
extern float Roll_X,Pitch_Y,Yaw_Z;

				    
#endif


