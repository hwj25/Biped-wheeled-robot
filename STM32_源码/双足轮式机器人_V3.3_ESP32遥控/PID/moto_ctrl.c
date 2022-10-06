#include "moto_ctrl.h"

   
int Left_Wheel_Speed= 0 ,Right_Wheel_Speed= 0;   //ƽ�����PID�����ٶ�֮��
u16 Time_Reset=0;            //ң�ضϵ����ֵ
s32 Travel_Speed=0, Turn_Speed=0;    //ǰ���ٶ�  ת���ٶ�
float Med_Angle=177.85, Med_Angle_Low=177.85;	//��е��ֵ��  181.8 - 178.8
u8 StartUp_Flag=false, Median_Flag=false, Pick_Up_Flag = true; //�������λ   ƽ������Ӧ��ȡ���λ   ������λ


float Balance_L_Out,Balance_R_Out,Velocity_out=0.0,Turn_out=0.0;//ֱ����&�ٶȻ�&ת�� ���������



//��ʱ��7�жϷ���������PID
void TIM1_UP_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET) //����ж�
	{
		pid_moto_calculation();
	}
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);  //����жϱ�־λ
}


/**************************************************************************
�������ܣ�ƽ��������PIDִ�к���
��ڲ�����
����  ֵ��
**************************************************************************/
void pid_moto_calculation(void)
{   

	Time_Reset++;
	APP_Time_Reset++;
	Start_Up();   //�ж��Ƿ��������
	Pick_Up();     //�ж��Ƿ������ʧ��
	Set_Median_Height();     //��̬����ƽ����ֵ
	//Get_Median();
	if(StartUp_Flag)    
	{
		if(Walking_mode==false)   //ƽ��ģʽ
		{
			Velocity_out = Velocity_pin(&pid_Velocity,(float)Travel_Speed,moto_chassis[0].speed_rpm,-moto_chassis[1].speed_rpm);    //�ٶȻ�PID
			Balance_L_Out = Balance_pin(&pid_Balance_L, Velocity_out+Med_Angle, Roll_X,Gyrox);         //ƽ�⻷PID
			//Balance_R_Out = Balance_pin(&pid_Balance_R, Velocity_out+Med_Angle, Roll_X,Gyrox);
			Turn_out = Turn_PID(Turn_Speed,Gyroz);         //ת�����PID      
			Left_Wheel_Speed =   Balance_L_Out + Turn_out;      //��������ٶ�
			Right_Wheel_Speed = -(Balance_L_Out - Turn_out);
		}
		else          //����ģʽ
		{
			Left_Wheel_Speed =   Travel_Speed/2 + Turn_Speed*2;      //��������ٶ�
			Right_Wheel_Speed = -(Travel_Speed/2 - Turn_Speed*2 );
		}
	}
	else     //�����˹ر�
	{
		
		pid_Balance_L.integral = 0;   //
		pid_Balance_R.integral = 0;
		Left_Wheel_Speed =  0;
		Right_Wheel_Speed =  0;
		
		
	}
	
	pid_calc(&pid_2006_speed[0],moto_chassis[0].speed_rpm,Left_Wheel_Speed);  //����ת�ٻ�PID
	pid_calc(&pid_2006_speed[1],moto_chassis[1].speed_rpm,Right_Wheel_Speed);  //����ת�ٻ�PID
	send_chassis_cur1_4(pid_2006_speed[0].pos_out,pid_2006_speed[1].pos_out, 0, 0);		//����1-2���ݸ�can�շ���
//	send_chassis_cur1_4(Left_Wheel_Speed,Right_Wheel_Speed, 0, 0);	
}

/**************************************************************************
�������ܣ��Ƿ�����������ƺ���
��ڲ�����
����  ֵ��
**************************************************************************/
void Start_Up()   
{
	if(Roll_X > Med_Angle - ERROR_MAX && Roll_X < Med_Angle + ERROR_MAX && Pick_Up_Flag == false)
		StartUp_Flag = true;
	else 
		StartUp_Flag = false;
		
}

/**************************************************************************
�������ܣ����С���Ƿ�����
��ڲ�����
����  ֵ��
**************************************************************************/
void Pick_Up()
{
	static u16 count = 0;
	if(Pick_Up_Flag == false)
	{
		if(ABS(moto_chassis[0].speed_rpm) + ABS(moto_chassis[1].speed_rpm)>12500)  		
			count++;		//����
		else
			count = 0;
		if(count > 100)
			Pick_Up_Flag = true;
	}
}

/**************************************************************************
�������ܣ������ȳ�����Ӧƽ��Ƕ�
��ڲ�����
����  ֵ��
**************************************************************************/
void Set_Median_Height()
{
	Med_Angle = Med_Angle_Low - (R_Pot_Leg_Height - 6.0)*0.08;
}


/**************************************************************************
�������ܣ�����Ӧ��ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Median()
{
	 static int  count;
     if(StartUp_Flag==true&&Median_Flag==false)
	{
		if(ABS(moto_chassis[0].speed_rpm)<1000&&ABS(moto_chassis[1].speed_rpm)<1000)  		
			count++;		//����
		else			                                	
			count=0;	
		if(count>200)  //����3�봦��ƽ��λ�ã���ȡ��ֵ
		{	
			Med_Angle_Low=Roll_X;	
			Median_Flag=true;
		}
	} 
}


/**************************************************************************
�������ܣ�PID��ʼ������
��ڲ�����
����  ֵ��
**************************************************************************/
void pid_first_param_init(void)
{
	int i;
	
    for ( i = 0; i < 2; i++)  //16384.0f��Ӧ20A	15.5f	 0.000100			15.5f,0,0 // 16.0f, 0.001f   20.0f, 0.004297f   22.0f, 0.01399f
    {
		PID_struct_init(&pid_2006_speed[i], POSITION_PID, 10000.0f, 3000.0f, 1.5f, 0.100f, 0.0f);		//�ٶȻ�PID��pid�ṹ�壬PID���ͣ����������������ƣ�P , I , D ��
	}
			//	PID_struct_init(&pid_3508_jy901b, POSITION_PID, 1000.0f, 1000.0f, 50.0f, 0.0f, 16.0f);		//λ�û�PID��&pid�ṹ�壬PID���ͣ����������������ƣ�P , I , D �� ʹ���µ�PID�����ȳ�ʼ��
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


//PID�ж�ִ���жϳ�ʼ��
void TIM1_Init_Clock(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //�����Զ���װ�ؼĴ�������ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc;//����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//�ظ���������
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //������ʼ��
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);//���жϱ�־λ

	TIM_ITConfig(      //ʹ�ܻ���ʧ��ָ����TIM�ж�
	TIM1,            //TIM1
	TIM_IT_Update  | //TIM �����ж�Դ
	TIM_IT_Trigger,  //TIM �����ж�Դ 
	ENABLE  	     //ʹ��
	);
	
	  //�������ȼ�
	  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�0��
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  	   //�����ȼ�0��
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure); 


	  TIM_Cmd(TIM1, ENABLE);  //ʹ��TIMx����
	
}
