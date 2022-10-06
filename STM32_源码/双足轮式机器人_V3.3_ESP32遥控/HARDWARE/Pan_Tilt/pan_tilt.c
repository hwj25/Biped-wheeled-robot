#include "pan_tilt.h"

void Pan_Tilt_TIM4_PWM_Init(u32 arr,u32 psc)     //电机-定时器4，通道3和通道4，PB8和PB9
{		 					 
	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure; //定义一个引脚初始化的结构体
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //定义一个定时中断的结构体	
	TIM_OCInitTypeDef TIM_OCInitTypeStrue; //定义一个PWM输出的结构体
//	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能GPIOB时钟，GPIOB挂载在APB2时钟下，在STM32中使用IO口前都要使能对应时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //使能通用定时器3时钟
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9;//引脚6  7
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; //复用推挽输出模式，定时器功能为A6 A7引脚复用功能
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; //定义该引脚输出速度为50MHZ
	GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化引脚GPIOB1
	
	TIM_TimeBaseInitStrue.TIM_Period=arr; //计数模式为向上计数时，定时器从0开始计数，计数超过到arr时触发定时中断服务函数
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc; //预分频系数，决定每一个计数的时长
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up; //计数模式：向上计数
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1; //一般不使用，默认TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStrue); //根据TIM_TimeBaseInitStrue的参数初始化定时器TIM3
	
	TIM_OCInitTypeStrue.TIM_OCMode=TIM_OCMode_PWM1; //PWM模式1，当定时器计数小于TIM_Pulse时，定时器对应IO输出有效电平
	TIM_OCInitTypeStrue.TIM_OCPolarity=TIM_OCNPolarity_High; //输出有效电平为高电平
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable; //使能PWM输出

	TIM_OCInitTypeStrue.TIM_Pulse = 1500; //设置待装入捕获比较寄存器的脉冲值


	TIM_OC3Init(TIM4, &TIM_OCInitTypeStrue); //根TIM_OCInitTypeStrue参数初始化定时器4通道3
	TIM_OC4Init(TIM4, &TIM_OCInitTypeStrue); //根TIM_OCInitTypeStrue参数初始化定时器4通道4
	

	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable); //CH4预装载使能  使能后改变TIM_Pulse(即PWM)的值立刻生效，不使能则下个周期生效
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Disable); //CH4预装载使能  使能后改变TIM_Pulse(即PWM)的值立刻生效，不使能则下个周期生效
	
	TIM_ARRPreloadConfig(TIM4, ENABLE); //TIM3预装载使能
	
	TIM_Cmd(TIM4, ENABLE); //使能定时器TIM3		

								
} 


uint16_t pan_tilt_x = 1400, pan_tilt_y = 1500; 
//OLED任务函数
void PAN_TILT_task(void *pvParameters)
{	
	
    while(1)
    {	
		
		
		if(Angle_Reset == true)
		{
			Angle_Reset = false;
			pan_tilt_x = 1470;
			pan_tilt_y = 1500;
		}
		else if(Botton[1]==0)
		{
			pan_tilt_x = pan_tilt_x + (uint16_t)(-R_Rocker_X);
			pan_tilt_y = pan_tilt_y + (uint16_t)(-R_Rocker_Y);

			if(pan_tilt_x>2500) pan_tilt_x=2500;
			else if(pan_tilt_x<500) pan_tilt_x=500;
		
			if(pan_tilt_y>2100) pan_tilt_y=2100;
			else if(pan_tilt_y<800) pan_tilt_y=800;
		}

					

		TIM_SetCompare3(TIM4,pan_tilt_y);
		TIM_SetCompare4(TIM4,pan_tilt_x);

        vTaskDelay(100); //延时100ms
    }
}



