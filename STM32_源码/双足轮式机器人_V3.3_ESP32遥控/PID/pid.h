#ifndef __pid_H
#define __pid_H
/* Includes ------------------------------------------------------------------*/
#include "sys.h"

#define ABS(x)		((x>0)? (x): (-x))

enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
    
    POSITION_PID,
    DELTA_PID,
};
typedef struct __pid_t
{
    float p;
    float i;
    float d;
    
    float set[3];				//目标值,包含NOW， LAST， LLAST上上次
    float get[3];				//测量值
    float err[3];				//误差
	
    
    float pout;							//p输出
    float iout;							//i输出
    float dout;							//d输出
    
    float pos_out;						//本次位置式输出
    float last_pos_out;					//上次输出
    float delta_u;						//本次增量值
    float delta_out;					//本次增量式输出 = last_delta_out + delta_u
    float last_delta_out;
    
	  float max_err;
	  float deadband;				//err < deadband return
    uint32_t pid_mode;
    uint32_t MaxOutput;				//输出限幅
    uint32_t IntegralLimit;		//积分限幅
    
    void (*f_param_init)(struct __pid_t *pid,  //PID参数初始化
                    uint32_t pid_mode,
                    uint32_t maxOutput,
                    uint32_t integralLimit,
                    float p,
                    float i,
                    float d);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);		//pid三个参数修改

}pid_t;





void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd);

float pid_calc(pid_t* pid, float fdb, float ref);
void pid_reset(pid_t	*pid, float kp, float ki, float kd);

//定义成全局变量
extern pid_t pid_2006_speed[2];



/*---------------------

新的pid
-------------------------*/


typedef struct __pid_d{
    float ActualOut;        //定义实际输出
    float err;              //定义偏差值
    float err_last;         //定义上一个偏差值
    float err_last1;         //定义上一个偏差值
    float Kp,Ki,Kd;         //定义比例、积分、微分系数
    float pidout;           //定义控制执行器的变量
    float integral;         //定义积分值
    float derivative;       //定义微分
    float newerr;
    float newerr1;
}pid_d;


extern pid_d pid_Balance_R,pid_Balance_L;
extern pid_d pid_Velocity;
extern pid_d pid_Turn;
extern pid_d pid_Height;
float Balance_pin(pid_d *pid, float Target,float Xangle, float gyro_X);
float Velocity_pin(pid_d *pid, float Target,float Aspeed,float Bspeed);  //速度环PID控制器
float Turn_PID (float Target,float gyro_Z);
float Height_pid(pid_d *pid, float Target,float pitch_y);


#endif



