/**
  ******************************************************************************
  * @file			pid.c
  * @version		V1.0.0
  * @date			2016年11月11日17:21:36
  * @brief   		对于PID， 反馈/测量习惯性叫get/measure/real/fdb,
						  期望输入一般叫set/target/ref
  *******************************************************************************/
  
  
  
/* Includes ------------------------------------------------------------------*/
#include "pid.h"

//定义pid结构体
pid_t pid_2006_speed[2];




 

void abs_limit(float *a, float ABS_MAX){
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}
/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(
    pid_t *pid, 
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    
}
/*中途更改参数设定(调试)------------------------------------------------------------*/
void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/**
    *@bref. calculate delta PID and position PID
    *@param[in] set： target
    *@param[in] real	measure
    */
float pid_calc(pid_t* pid, float get, float set)
	{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
		return 0;
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
		return 0;
    
    if(pid->pid_mode == POSITION_PID) //位置式p
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//增量式P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
//	
}

/*pid总体初始化-----------------------------------------------------------------*/
void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;
		
    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
	
}



/*---------------------

新的pid
-------------------------*/

pid_d pid_Balance_R,pid_Balance_L;
pid_d pid_Velocity;
pid_d pid_Turn;
pid_d pid_Height;
/**************************************************************************
函数功能：平衡环PID
入口参数：
返回  值：
**************************************************************************/
float Balance_pin(pid_d *pid, float Target,float Xangle, float gyro_X)  
{     
    pid->err = Xangle-Target;
       
    pid->integral += pid->err;

    if(pid->integral>3000) pid->integral = 3000;    //积分限幅
    if(pid->integral<(-3000)) pid->integral = -3000;   

    pid->derivative = gyro_X;//(Abalance.err-Abalance.err_last)/0.01;;
    
    pid->pidout = pid->Kp*pid->err + pid->Ki*pid->integral + pid->Kd*pid->derivative;
   
    pid->err_last = pid->err;
    
    pid->ActualOut = pid->pidout*1.0;
       
   

    if(pid->ActualOut>6500) 
    pid->ActualOut = 6500;
    else if(pid->ActualOut<(-6500)) 
    pid->ActualOut = -6500;
       
    return pid->ActualOut;
}


#define N_SAMPLEL_V 4
float Sample_Array_V[N_SAMPLEL_V] = {0}; 
float V_Sliding_weighted_filter(float xdat1) //滑动加权滤波算法
{
  float array_sum = 0; //采样队列和                             
  for(int i=1;i<N_SAMPLEL_V;i++)
  {
      Sample_Array_V[i-1] = Sample_Array_V[i];
      array_sum += Sample_Array_V[i] * i;
  }  
  Sample_Array_V[N_SAMPLEL_V-1] = xdat1;
  array_sum += xdat1 * N_SAMPLEL_V;
  float filte_value= (array_sum / ((1+N_SAMPLEL_V)*N_SAMPLEL_V/2.0)); //
  return filte_value;
}

/**************************************************************************
函数功能：速度环PID
入口参数：
返回  值：
**************************************************************************/
float Velocity_pin(pid_d *pid, float Target,float Speed_L,float Speed_R)  
{   
    float newerr =Speed_L+Speed_R; 
    
    newerr = newerr-Target; 
           
    pid->newerr = newerr*0.3+pid->newerr*0.7;
    
    pid->newerr1 = V_Sliding_weighted_filter(pid->newerr);
	pid->err = pid->newerr1/100;
//	pid->err = pid->newerr/100;


	pid->integral += pid->err/100;
	if(pid->integral>10) pid->integral = 10;   
	if(pid->integral<(-10)) pid->integral = -10;    
    

    pid->derivative = (pid->err-pid->err_last)/100;
    
    pid->pidout = pid->Kp*pid->err + pid->Ki*pid->integral + pid->Kd*pid->derivative;
          
    pid->ActualOut = pid->pidout*1.0;

    if(pid->ActualOut>20) pid->ActualOut = 20;   
    if(pid->ActualOut<(-20)) pid->ActualOut = -20;
    
    return pid->ActualOut;
}
#define N_SAMPLEL_T 5
float Sample_Array_T[N_SAMPLEL_T] = {0}; 
float T_Sliding_weighted_filter(float xdat1) //滑动加权滤波算法
{
  long array_sum = 0; //采样队列和                             
  for(int i=1;i<N_SAMPLEL_T;i++)
  {
      Sample_Array_T[i-1] = Sample_Array_T[i];
      array_sum += Sample_Array_T[i] * i;
  }  
  Sample_Array_T[N_SAMPLEL_T-1] = xdat1;
  array_sum += xdat1 * N_SAMPLEL_T;
  float filte_value= (array_sum / ((1+N_SAMPLEL_T)*N_SAMPLEL_T/2.0)); //
  return filte_value;
}

/**************************************************************************
函数功能：转弯环环PID
入口参数：
返回  值：
**************************************************************************/
float Turn_PID (float Target,float gyro_Z)
{      
   float zx = T_Sliding_weighted_filter(gyro_Z);    
   pid_Turn.err = zx-Target;


   pid_Turn.pidout=pid_Turn.Kp*pid_Turn.err;         //位置式PID控制器    
   pid_Turn.ActualOut = pid_Turn.pidout*1.0;
             
   return pid_Turn.ActualOut;   
}

/**************************************************************************
函数功能：腿部控制PID         --增量式
入口参数：
返回  值：
**************************************************************************/
float Height_pid(pid_d *pid, float Target,float pitch_y)
{         
    pid->err =  pitch_y-Target;

    if(ABS(pid->err)<=0.2)
    {
       pid->err = 0;  
    }

    float p = pid->Kp*(pid->err - pid->err_last);
    float i = pid->Ki*pid->err;
    float d = pid->Kd*(pid->err-2*pid->err_last+pid->err_last1);
      
    pid->pidout = pid->pidout+p+i+d;
   
    pid->err_last = pid->err;
    
    pid->err_last1 = pid->err_last;

    if(pid->pidout>10)
    pid->pidout = 10;  
    else if(pid->pidout<(-10))
    pid->pidout = -10;   
    
    pid->ActualOut = pid->pidout;
    
    return pid->ActualOut;
}