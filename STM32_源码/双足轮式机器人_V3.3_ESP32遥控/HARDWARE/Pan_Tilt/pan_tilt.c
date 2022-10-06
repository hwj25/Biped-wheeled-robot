#include "pan_tilt.h"

void Pan_Tilt_TIM4_PWM_Init(u32 arr,u32 psc)     //���-��ʱ��4��ͨ��3��ͨ��4��PB8��PB9
{		 					 
	//�˲������ֶ��޸�IO������
	
	GPIO_InitTypeDef GPIO_InitStructure; //����һ�����ų�ʼ���Ľṹ��
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //����һ����ʱ�жϵĽṹ��	
	TIM_OCInitTypeDef TIM_OCInitTypeStrue; //����һ��PWM����Ľṹ��
//	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��GPIOBʱ�ӣ�GPIOB������APB2ʱ���£���STM32��ʹ��IO��ǰ��Ҫʹ�ܶ�Ӧʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʹ��ͨ�ö�ʱ��3ʱ��
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9;//����6  7
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; //�����������ģʽ����ʱ������ΪA6 A7���Ÿ��ù���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; //�������������ٶ�Ϊ50MHZ
	GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ������GPIOB1
	
	TIM_TimeBaseInitStrue.TIM_Period=arr; //����ģʽΪ���ϼ���ʱ����ʱ����0��ʼ����������������arrʱ������ʱ�жϷ�����
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc; //Ԥ��Ƶϵ��������ÿһ��������ʱ��
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up; //����ģʽ�����ϼ���
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1; //һ�㲻ʹ�ã�Ĭ��TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStrue); //����TIM_TimeBaseInitStrue�Ĳ�����ʼ����ʱ��TIM3
	
	TIM_OCInitTypeStrue.TIM_OCMode=TIM_OCMode_PWM1; //PWMģʽ1������ʱ������С��TIM_Pulseʱ����ʱ����ӦIO�����Ч��ƽ
	TIM_OCInitTypeStrue.TIM_OCPolarity=TIM_OCNPolarity_High; //�����Ч��ƽΪ�ߵ�ƽ
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable; //ʹ��PWM���

	TIM_OCInitTypeStrue.TIM_Pulse = 1500; //���ô�װ�벶��ȽϼĴ���������ֵ


	TIM_OC3Init(TIM4, &TIM_OCInitTypeStrue); //��TIM_OCInitTypeStrue������ʼ����ʱ��4ͨ��3
	TIM_OC4Init(TIM4, &TIM_OCInitTypeStrue); //��TIM_OCInitTypeStrue������ʼ����ʱ��4ͨ��4
	

	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable); //CH4Ԥװ��ʹ��  ʹ�ܺ�ı�TIM_Pulse(��PWM)��ֵ������Ч����ʹ�����¸�������Ч
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Disable); //CH4Ԥװ��ʹ��  ʹ�ܺ�ı�TIM_Pulse(��PWM)��ֵ������Ч����ʹ�����¸�������Ч
	
	TIM_ARRPreloadConfig(TIM4, ENABLE); //TIM3Ԥװ��ʹ��
	
	TIM_Cmd(TIM4, ENABLE); //ʹ�ܶ�ʱ��TIM3		

								
} 


uint16_t pan_tilt_x = 1400, pan_tilt_y = 1500; 
//OLED������
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

        vTaskDelay(100); //��ʱ100ms
    }
}



