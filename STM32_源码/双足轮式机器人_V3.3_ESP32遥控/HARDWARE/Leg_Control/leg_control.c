#include "leg_control.h"

float Height_out=0.0;
float R_Pot_Leg_Height=6.0; 
float Leg_Med_Angle = -6.5;
float Leg_Balance = 0.0;
void TIM3_PWM_Init(u32 arr,u32 psc)     //电机-定时器3，通道1和通道2，PC6和PC7
{		 					 
	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure; //定义一个引脚初始化的结构体
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //定义一个定时中断的结构体	
	TIM_OCInitTypeDef TIM_OCInitTypeStrue; //定义一个PWM输出的结构体
//	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能GPIOB时钟，GPIOB挂载在APB2时钟下，在STM32中使用IO口前都要使能对应时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //使能通用定时器3时钟
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;//引脚6  7
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; //复用推挽输出模式，定时器功能为A6 A7引脚复用功能
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; //定义该引脚输出速度为50MHZ
	GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化引脚GPIOA6  7

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_0;//引脚0
	GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化引脚GPIOB1
	
	TIM_TimeBaseInitStrue.TIM_Period=arr; //计数模式为向上计数时，定时器从0开始计数，计数超过到arr时触发定时中断服务函数
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc; //预分频系数，决定每一个计数的时长
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up; //计数模式：向上计数
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1; //一般不使用，默认TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStrue); //根据TIM_TimeBaseInitStrue的参数初始化定时器TIM3
	
	TIM_OCInitTypeStrue.TIM_OCMode=TIM_OCMode_PWM1; //PWM模式1，当定时器计数小于TIM_Pulse时，定时器对应IO输出有效电平
	TIM_OCInitTypeStrue.TIM_OCPolarity=TIM_OCNPolarity_High; //输出有效电平为高电平
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable; //使能PWM输出

	TIM_OCInitTypeStrue.TIM_Pulse = R_THIGH_FRONT_MIDDLE; //设置待装入捕获比较寄存器的脉冲值
	TIM_OC1Init(TIM3, &TIM_OCInitTypeStrue); //根TIM_OCInitTypeStrue参数初始化定时器3通道1
	
	TIM_OCInitTypeStrue.TIM_Pulse = R_THIGH_BACK_MIDDLE; //设置待装入捕获比较寄存器的脉冲值
	TIM_OC2Init(TIM3, &TIM_OCInitTypeStrue); //根TIM_OCInitTypeStrue参数初始化定时器3通道2

	TIM_OCInitTypeStrue.TIM_Pulse = L_THIGH_FRONT_MIDDLE; //设置待装入捕获比较寄存器的脉冲值
	TIM_OC3Init(TIM3, &TIM_OCInitTypeStrue); //根TIM_OCInitTypeStrue参数初始化定时器3通道3

	TIM_OCInitTypeStrue.TIM_Pulse = L_THIGH_BACK_MIDDLE; //设置待装入捕获比较寄存器的脉冲值
	TIM_OC4Init(TIM3, &TIM_OCInitTypeStrue); //根TIM_OCInitTypeStrue参数初始化定时器3通道4
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable); //CH4预装载使能  使能后改变TIM_Pulse(即PWM)的值立刻生效，不使能则下个周期生效
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable); //CH4预装载使能  使能后改变TIM_Pulse(即PWM)的值立刻生效，不使能则下个周期生效
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable); //CH4预装载使能  使能后改变TIM_Pulse(即PWM)的值立刻生效，不使能则下个周期生效
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable); //CH4预装载使能  使能后改变TIM_Pulse(即PWM)的值立刻生效，不使能则下个周期生效
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //TIM3预装载使能
	
	TIM_Cmd(TIM3, ENABLE); //使能定时器TIM3		

								
}  




/**************************************************************************
函数功能：左腿长度计算函数
入口参数：float  float
返回  值：
**************************************************************************/
void Leg_Left_Height(float x, float y)     
{
    float L=0;
    float N=0;
    double M=0;
    float A1=0;
    float A2=0;
	float theta1=0;
	float theta2=0;
	
	x=-x;
	if(x>10)
		x=10;
	else if(x<-10)
		x=-10;

	if(y>23)
		y=23;
	else if(y<4.5)
		y=4.5;
	
    L=sqrt(pow(x,2) + pow(y,2));      //pow 为求以x为底的2次方，sqrt为求算术平方根

    if(L<4.5) L=4.5;              //限制L的值
    else if(L>21.0) L=21.0;         //限制L的值


    N=asin(x/L)*180.0/PI;           //asin为转角度     PI等于3.14159265358979f
    M=acos(	(pow(L,2)+pow(L1,2)-pow(L2,2))/(2*L1*L)	)*180.0/PI;     //L1=10，L2=20
    A1=M-N;

    A2=M+N;

	theta2= -((A1-90) *  CHANGE_ANGLE_180) + L_THIGH_FRONT_MIDDLE;
	theta1=(A2-90) * CHANGE_ANGLE + L_THIGH_BACK_MIDDLE;

	if(theta2 < L_THIGH_FRONT_MIN)
		theta2 = L_THIGH_FRONT_MIN;
	else if(theta2 >  L_THIGH_FRONT_MAX)
		theta2 =  L_THIGH_FRONT_MAX;

	if(theta1 < L_THIGH_BACK_MIN)
		theta1 = L_THIGH_BACK_MIN;
	else if(theta1 >  L_THIGH_BACK_MAX)
		theta1 =  L_THIGH_BACK_MAX;
	TIM_SetCompare3(TIM3,(uint16_t)theta2);
	TIM_SetCompare4(TIM3,(uint16_t)theta1);

 //   printf("\r\n  theta1=%f  theta2=%f   ",theta1,theta2);
}


/**************************************************************************
函数功能：右腿长度计算函数
入口参数：float  float
返回  值：
**************************************************************************/
void Leg_Right_Height(float x, float y)     
{
    float L=0;
    float N=0;
    double M=0;
    float A1=0;
    float A2=0;
	float theta1=0;
	float theta2=0;

	if(x>10)
		x=10;
	else if(x<-10)
		x=-10;

	if(y>23)
		y=23;
	else if(y<4.5)
		y=4.5;
	
    L=sqrt(pow(x,2) + pow(y,2));      //pow 为求以x为底的2次方，sqrt为求算术平方根

    if(L<4.5) L=4.5;              //限制L的值
    else if(L>21.0) L=21.0;         //限制L的值


    N=asin(x/L)*180.0/PI;           //asin为转角度     PI等于3.14159265358979f
    M=acos(	(pow(L,2)+pow(L1,2)-pow(L2,2))/(2*L1*L)	)*180.0/PI;     //L1=10，L2=20
    A1=M-N;

    A2=M+N;
	
	theta1=(A2-90) * CHANGE_ANGLE + R_THIGH_FRONT_MIDDLE;
	theta2= -((A1-90) *  CHANGE_ANGLE) + R_THIGH_BACK_MIDDLE;
	

	if(theta1 < R_THIGH_FRONT_MIN)
		theta1 = R_THIGH_FRONT_MIN;
	else if(theta1 >  R_THIGH_FRONT_MAX)
		theta1 =  R_THIGH_FRONT_MAX;

	if(theta2 < R_THIGH_BACK_MIN)
		theta2 = R_THIGH_BACK_MIN;
	else if(theta2 >  R_THIGH_BACK_MAX)
		theta2 =  R_THIGH_BACK_MAX;


	TIM_SetCompare1(TIM3,(uint16_t)theta1);
 	TIM_SetCompare2(TIM3,(uint16_t)theta2);


   // printf("\r\n  theta1=%f  theta2=%f   ",theta1,theta2);
}


/**************************************************************************
函数功能：腿部平衡控制任务
入口参数：
返回  值：
**************************************************************************/
void Leg_Control_task(void *pvParameters)
{
    while(1)
    {
		if(StartUp_Flag)
		{
			if(Walking_mode==false)  //平衡模式
			{
				Height_out = Height_pid(&pid_Height, Leg_Med_Angle+Leg_Balance, Pitch_Y);
				Leg_Left_Height(0,R_Pot_Leg_Height+Height_out);
				Leg_Right_Height(0,R_Pot_Leg_Height-Height_out);
			}
			else        //匍匐模式
			{
				Leg_Left_Height(0,4.5);
				Leg_Right_Height(0,4.5);
			}
			
		}
		else        //机器人关闭
		{
			pid_Height.pidout = 0.0;
			Leg_Left_Height(0,4.5);
			Leg_Right_Height(0,4.5);
		}

	
        vTaskDelay(10); //延时200ms
    }
}
