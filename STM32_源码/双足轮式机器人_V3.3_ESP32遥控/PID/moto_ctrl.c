#include "moto_ctrl.h"

   
int Left_Wheel_Speed= 0 ,Right_Wheel_Speed= 0;   //平衡相关PID计算速度之和
u16 Time_Reset=0;            //遥控断电计数值
s32 Travel_Speed=0, Turn_Speed=0;    //前进速度  转弯速度
float Med_Angle=177.85, Med_Angle_Low=177.85;	//机械中值。  181.8 - 178.8
u8 StartUp_Flag=false, Median_Flag=false, Pick_Up_Flag = true; //启动标记位   平衡自适应读取标记位   拿起标记位


float Balance_L_Out,Balance_R_Out,Velocity_out=0.0,Turn_out=0.0;//直立环&速度环&转向环 的输出变量



//定时器7中断服务函数进行PID
void TIM1_UP_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET) //溢出中断
	{
		pid_moto_calculation();
	}
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);  //清除中断标志位
}


/**************************************************************************
函数功能：平衡控制相关PID执行函数
入口参数：
返回  值：
**************************************************************************/
void pid_moto_calculation(void)
{   

	Time_Reset++;
	APP_Time_Reset++;
	Start_Up();   //判断是否可以启动
	Pick_Up();     //判断是否被拿起或失控
	Set_Median_Height();     //动态调整平衡中值
	//Get_Median();
	if(StartUp_Flag)    
	{
		if(Walking_mode==false)   //平衡模式
		{
			Velocity_out = Velocity_pin(&pid_Velocity,(float)Travel_Speed,moto_chassis[0].speed_rpm,-moto_chassis[1].speed_rpm);    //速度环PID
			Balance_L_Out = Balance_pin(&pid_Balance_L, Velocity_out+Med_Angle, Roll_X,Gyrox);         //平衡环PID
			//Balance_R_Out = Balance_pin(&pid_Balance_R, Velocity_out+Med_Angle, Roll_X,Gyrox);
			Turn_out = Turn_PID(Turn_Speed,Gyroz);         //转弯控制PID      
			Left_Wheel_Speed =   Balance_L_Out + Turn_out;      //最终输出速度
			Right_Wheel_Speed = -(Balance_L_Out - Turn_out);
		}
		else          //匍匐模式
		{
			Left_Wheel_Speed =   Travel_Speed/2 + Turn_Speed*2;      //最终输出速度
			Right_Wheel_Speed = -(Travel_Speed/2 - Turn_Speed*2 );
		}
	}
	else     //机器人关闭
	{
		
		pid_Balance_L.integral = 0;   //
		pid_Balance_R.integral = 0;
		Left_Wheel_Speed =  0;
		Right_Wheel_Speed =  0;
		
		
	}
	
	pid_calc(&pid_2006_speed[0],moto_chassis[0].speed_rpm,Left_Wheel_Speed);  //左轮转速环PID
	pid_calc(&pid_2006_speed[1],moto_chassis[1].speed_rpm,Right_Wheel_Speed);  //右轮转速环PID
	send_chassis_cur1_4(pid_2006_speed[0].pos_out,pid_2006_speed[1].pos_out, 0, 0);		//传递1-2数据给can收发器
//	send_chassis_cur1_4(Left_Wheel_Speed,Right_Wheel_Speed, 0, 0);	
}

/**************************************************************************
函数功能：是否可以启动控制函数
入口参数：
返回  值：
**************************************************************************/
void Start_Up()   
{
	if(Roll_X > Med_Angle - ERROR_MAX && Roll_X < Med_Angle + ERROR_MAX && Pick_Up_Flag == false)
		StartUp_Flag = true;
	else 
		StartUp_Flag = false;
		
}

/**************************************************************************
函数功能：检测小车是否被拿起
入口参数：
返回  值：
**************************************************************************/
void Pick_Up()
{
	static u16 count = 0;
	if(Pick_Up_Flag == false)
	{
		if(ABS(moto_chassis[0].speed_rpm) + ABS(moto_chassis[1].speed_rpm)>12500)  		
			count++;		//采样
		else
			count = 0;
		if(count > 100)
			Pick_Up_Flag = true;
	}
}

/**************************************************************************
函数功能：根据腿长自适应平衡角度
入口参数：
返回  值：
**************************************************************************/
void Set_Median_Height()
{
	Med_Angle = Med_Angle_Low - (R_Pot_Leg_Height - 6.0)*0.08;
}


/**************************************************************************
函数功能：自适应中值
入口参数：无
返回  值：无
**************************************************************************/
void Get_Median()
{
	 static int  count;
     if(StartUp_Flag==true&&Median_Flag==false)
	{
		if(ABS(moto_chassis[0].speed_rpm)<1000&&ABS(moto_chassis[1].speed_rpm)<1000)  		
			count++;		//采样
		else			                                	
			count=0;	
		if(count>200)  //连线3秒处于平衡位置，读取中值
		{	
			Med_Angle_Low=Roll_X;	
			Median_Flag=true;
		}
	} 
}


/**************************************************************************
函数功能：PID初始化函数
入口参数：
返回  值：
**************************************************************************/
void pid_first_param_init(void)
{
	int i;
	
    for ( i = 0; i < 2; i++)  //16384.0f对应20A	15.5f	 0.000100			15.5f,0,0 // 16.0f, 0.001f   20.0f, 0.004297f   22.0f, 0.01399f
    {
		PID_struct_init(&pid_2006_speed[i], POSITION_PID, 10000.0f, 3000.0f, 1.5f, 0.100f, 0.0f);		//速度环PID（pid结构体，PID类型，最大输出，积分限制，P , I , D ）
	}
			//	PID_struct_init(&pid_3508_jy901b, POSITION_PID, 1000.0f, 1000.0f, 50.0f, 0.0f, 16.0f);		//位置环PID（&pid结构体，PID类型，最大输出，积分限制，P , I , D ） 使用新的PID必须先初始化
	pid_Balance_L.ActualOut = 0; 
	pid_Balance_L.err = 0;  
	pid_Balance_L.err_last = 0;
	pid_Balance_L.pidout = 0;
	pid_Balance_L.integral = 0;
	pid_Balance_L.Kp = 290.0;      //135        290
	pid_Balance_L.Ki = 12.5;		//2.6       12.5
	pid_Balance_L.Kd = 12.0;       //8.3       12.3

	pid_Balance_R.ActualOut = 0; 
	pid_Balance_R.err = 0;  
	pid_Balance_R.err_last = 0;
	pid_Balance_R.pidout = 0;
	pid_Balance_R.integral = 0;
	pid_Balance_R.Kp = 0;      //113
	pid_Balance_R.Ki = 0;		//2.3
	pid_Balance_R.Kd = 0;       //7.6

	pid_Velocity.ActualOut = 0; 
	pid_Velocity.err = 0;  
	pid_Velocity.err_last = 0;
	pid_Velocity.pidout = 0;
	pid_Velocity.integral = 0;
	pid_Velocity.Kp = -0.08;      
	pid_Velocity.Ki = -0.01;		
	pid_Velocity.Kd = 0;   

	
	pid_Turn.ActualOut = 0; 
	pid_Turn.err = 0;  
	pid_Turn.err_last = 0;
	pid_Turn.pidout = 0;
	pid_Turn.integral = 0;
	pid_Turn.Kp = -0.8;      
	pid_Turn.Ki = 0;		
	pid_Turn.Kd = 0;   

	
	pid_Height.ActualOut = 0; 
	pid_Height.err = 0;  
	pid_Height.err_last = 0;
	pid_Height.pidout = 0;
	pid_Height.integral = 0;
	pid_Height.Kp = 0.1;      
	pid_Height.Ki = 0.06;		
	pid_Height.Kd = 0;  
}


//PID中断执行中断初始化
void TIM1_Init_Clock(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置自动重装载寄存器周期值
	TIM_TimeBaseStructure.TIM_Prescaler =psc;//设置预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//重复计数设置
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //参数初始化
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);//清中断标志位

	TIM_ITConfig(      //使能或者失能指定的TIM中断
	TIM1,            //TIM1
	TIM_IT_Update  | //TIM 更新中断源
	TIM_IT_Trigger,  //TIM 触发中断源 
	ENABLE  	     //使能
	);
	
	  //设置优先级
	  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//先占优先级0级
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  	   //从优先级0级
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure); 


	  TIM_Cmd(TIM1, ENABLE);  //使能TIMx外设
	
}
