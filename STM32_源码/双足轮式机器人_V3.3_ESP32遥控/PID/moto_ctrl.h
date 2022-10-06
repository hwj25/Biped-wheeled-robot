#ifndef MOTO_CTRL_H
#define MOTO_CTRL_H
#include "sys.h"

#define IsReady  1
#define NotReady 0

#define ERROR_MAX 23



extern int Left_Wheel_Speed, Right_Wheel_Speed;
extern u16 Time_Reset;
extern float Med_Angle;
extern s32 Travel_Speed, Turn_Speed;
extern u8 StartUp_Flag, Median_Flag, Pick_Up_Flag;


void pid_moto_calculation(void);
void pid_first_param_init(void);
void TIM1_Init_Clock(u16 arr,u16 psc);
void Set_Median_Height();
void Start_Up();
void Pick_Up();
void Get_Median();

#endif
