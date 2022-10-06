#ifndef __GPIO_H
#define __GPIO_H	 
#include "sys.h"

//#define LED0 PCout(13)// PB5

#define Mode_KEY PBin(13)
#define PickUpClearKEY PBin(14)

void GPIO_In_Init(void);

		 				    
#endif