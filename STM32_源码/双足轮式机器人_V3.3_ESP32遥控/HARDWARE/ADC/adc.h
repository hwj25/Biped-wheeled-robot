#ifndef __ADC_H
#define __ADC_H
#include "sys.h"


 
#define Sample_Num 10
#define Channel_Num 3
 

extern u8 Electric_Quantity_12, Electric_Quantity_24, Air_Quality;
 
void ADC1_Init(void);
void Read_ADC_AverageValue();
void Adc_Init();
void Get_Electric_Quantity_12_Average(u8 ch,u8 times);
void Get_Air_Quality_Average(u8 ch,u8 times);
void Get_Electric_Quantity_24_Average(u8 ch,u8 times);

#endif
