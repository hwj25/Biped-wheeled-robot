#ifndef __LEG_CONTROL_H
#define __LEG_CONTROL_H	 
#include "sys.h"

#define L1 11
#define L2 12

#define L_THIGH_FRONT_MIN 707
#define L_THIGH_FRONT_MIDDLE 787   //3        540
#define L_THIGH_FRONT_MAX 1590

#define L_THIGH_BACK_MIN 1120
#define L_THIGH_BACK_MIDDLE 1650    //4        530
#define L_THIGH_BACK_MAX 1700

#define R_THIGH_FRONT_MIN 1010
#define R_THIGH_FRONT_MIDDLE 1550   //1       
#define R_THIGH_FRONT_MAX 1600

#define R_THIGH_BACK_MIN 1300     
#define R_THIGH_BACK_MIDDLE 1350    //2
#define R_THIGH_BACK_MAX 1880

#define CHANGE_ANGLE 7.407
#define CHANGE_ANGLE_180 11.111


extern float R_Pot_Leg_Height;
extern float Leg_Balance;

void TIM3_PWM_Init(u32 arr,u32 psc);
void Leg_Left_Height(float x, float y);
void Leg_Right_Height(float x, float y) ;

		 				    
#endif 


