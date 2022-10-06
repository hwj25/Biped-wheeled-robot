#ifndef _SYSTEMCONTROL_H
#define _SYSTEMCONTROL_H
#include "sys.h"

extern u8 temperature;  	    
extern u8 humidity; 
//extern s16 L_Rocker_X, L_Rocker_Y; //
extern s16 R_Rocker_X, R_Rocker_Y;
//extern u8 L_Pot, R_Pot;  //
extern u8 Botton[8];
extern u8 LoRa_Remote_Control_Flag;

extern u8 APP_Remote_Control_Flag;
extern u32 APP_Time_Reset;
extern u8 Angle_Reset;
extern u8 Walking_mode;


#endif

