#include "leg_control.h"

float Height_out=0.0;
float R_Pot_Leg_Height=6.0; 
float Leg_Med_Angle = -6.5;
float Leg_Balance = 0.0;
void TIM3_PWM_Init(u32 arr,u32 psc)     //���-��ʱ��3��ͨ��1��ͨ��2��PC6��PC7
{		 					 
	//�˲������ֶ��޸�IO������
	
	GPIO_InitTypeDef GPIO_InitStructure; //����һ�����ų�ʼ���Ľṹ��
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //����һ����ʱ�жϵĽṹ��	
	TIM_OCInitTypeDef TIM_OCInitTypeStrue; //����һ��PWM����Ľṹ��
//	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��GPIOBʱ�ӣ�GPIOB������APB2ʱ���£���STM32��ʹ��IO��ǰ��Ҫʹ�ܶ�Ӧʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʹ��ͨ�ö�ʱ��3ʱ��
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;//����6  7
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; //�����������ģʽ����ʱ������ΪA6 A7���Ÿ��ù���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; //�������������ٶ�Ϊ50MHZ
	GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ������GPIOA6  7

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_0;//����0
	GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ������GPIOB1
	
	TIM_TimeBaseInitStrue.TIM_Period=arr; //����ģʽΪ���ϼ���ʱ����ʱ����0��ʼ����������������arrʱ������ʱ�жϷ�����
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc; //Ԥ��Ƶϵ��������ÿһ��������ʱ��
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up; //����ģʽ�����ϼ���
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1; //һ�㲻ʹ�ã�Ĭ��TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStrue); //����TIM_TimeBaseInitStrue�Ĳ�����ʼ����ʱ��TIM3
	
	TIM_OCInitTypeStrue.TIM_OCMode=TIM_OCMode_PWM1; //PWMģʽ1������ʱ������С��TIM_Pulseʱ����ʱ����ӦIO�����Ч��ƽ
	TIM_OCInitTypeStrue.TIM_OCPolarity=TIM_OCNPolarity_High; //�����Ч��ƽΪ�ߵ�ƽ
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable; //ʹ��PWM���

	TIM_OCInitTypeStrue.TIM_Pulse = R_THIGH_FRONT_MIDDLE; //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OC1Init(TIM3, &TIM_OCInitTypeStrue); //��TIM_OCInitTypeStrue������ʼ����ʱ��3ͨ��1
	
	TIM_OCInitTypeStrue.TIM_Pulse = R_THIGH_BACK_MIDDLE; //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OC2Init(TIM3, &TIM_OCInitTypeStrue); //��TIM_OCInitTypeStrue������ʼ����ʱ��3ͨ��2

	TIM_OCInitTypeStrue.TIM_Pulse = L_THIGH_FRONT_MIDDLE; //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OC3Init(TIM3, &TIM_OCInitTypeStrue); //��TIM_OCInitTypeStrue������ʼ����ʱ��3ͨ��3

	TIM_OCInitTypeStrue.TIM_Pulse = L_THIGH_BACK_MIDDLE; //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OC4Init(TIM3, &TIM_OCInitTypeStrue); //��TIM_OCInitTypeStrue������ʼ����ʱ��3ͨ��4
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable); //CH4Ԥװ��ʹ��  ʹ�ܺ�ı�TIM_Pulse(��PWM)��ֵ������Ч����ʹ�����¸�������Ч
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable); //CH4Ԥװ��ʹ��  ʹ�ܺ�ı�TIM_Pulse(��PWM)��ֵ������Ч����ʹ�����¸�������Ч
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable); //CH4Ԥװ��ʹ��  ʹ�ܺ�ı�TIM_Pulse(��PWM)��ֵ������Ч����ʹ�����¸�������Ч
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable); //CH4Ԥװ��ʹ��  ʹ�ܺ�ı�TIM_Pulse(��PWM)��ֵ������Ч����ʹ�����¸�������Ч
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //TIM3Ԥװ��ʹ��
	
	TIM_Cmd(TIM3, ENABLE); //ʹ�ܶ�ʱ��TIM3		

								
}  




/**************************************************************************
�������ܣ����ȳ��ȼ��㺯��
��ڲ�����float  float
����  ֵ��
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
	
    L=sqrt(pow(x,2) + pow(y,2));      //pow Ϊ����xΪ�׵�2�η���sqrtΪ������ƽ����

    if(L<4.5) L=4.5;              //����L��ֵ
    else if(L>21.0) L=21.0;         //����L��ֵ


    N=asin(x/L)*180.0/PI;           //asinΪת�Ƕ�     PI����3.14159265358979f
    M=acos(	(pow(L,2)+pow(L1,2)-pow(L2,2))/(2*L1*L)	)*180.0/PI;     //L1=10��L2=20
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
�������ܣ����ȳ��ȼ��㺯��
��ڲ�����float  float
����  ֵ��
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
	
    L=sqrt(pow(x,2) + pow(y,2));      //pow Ϊ����xΪ�׵�2�η���sqrtΪ������ƽ����

    if(L<4.5) L=4.5;              //����L��ֵ
    else if(L>21.0) L=21.0;         //����L��ֵ


    N=asin(x/L)*180.0/PI;           //asinΪת�Ƕ�     PI����3.14159265358979f
    M=acos(	(pow(L,2)+pow(L1,2)-pow(L2,2))/(2*L1*L)	)*180.0/PI;     //L1=10��L2=20
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
�������ܣ��Ȳ�ƽ���������
��ڲ�����
����  ֵ��
**************************************************************************/
void Leg_Control_task(void *pvParameters)
{
    while(1)
    {
		if(StartUp_Flag)
		{
			if(Walking_mode==false)  //ƽ��ģʽ
			{
				Height_out = Height_pid(&pid_Height, Leg_Med_Angle+Leg_Balance, Pitch_Y);
				Leg_Left_Height(0,R_Pot_Leg_Height+Height_out);
				Leg_Right_Height(0,R_Pot_Leg_Height-Height_out);
			}
			else        //����ģʽ
			{
				Leg_Left_Height(0,4.5);
				Leg_Right_Height(0,4.5);
			}
			
		}
		else        //�����˹ر�
		{
			pid_Height.pidout = 0.0;
			Leg_Left_Height(0,4.5);
			Leg_Right_Height(0,4.5);
		}

	
        vTaskDelay(10); //��ʱ200ms
    }
}
