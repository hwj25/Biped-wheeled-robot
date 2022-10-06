#include "systemcontrol.h"

u8 temperature;  	    
u8 humidity;
s16 L_Rocker_X=0, L_Rocker_Y=0, R_Rocker_X=0, R_Rocker_Y=0;
u8 L_Pot=0, R_Pot=0;
u8 Botton[8]={0};
u8 L_Pot_Med = 128, L_Pot_Med_Flag = true;
u8 LoRa_Remote_Control_Flag = false;

u8 APP_Remote_Control_Flag = false;
u32 APP_Time_Reset=0;
u8 Angle_Reset=false;

u8 Walking_mode = false;
u8 APP_Slide = false;
/**************************************************************************
�������ܣ���ȡLoRaң�ص�����
��ڲ�����
����  ֵ��
**************************************************************************/
void LORA_Read()
{
	if(USART2_LORA_date[0]>124 && USART2_LORA_date[0]<133)
	{
		L_Rocker_X = 0;
	}
	else{
		L_Rocker_X = (USART2_LORA_date[0] - 128);
	}


	if(USART2_LORA_date[1]>124 && USART2_LORA_date[1]<133)
	{
		L_Rocker_Y = 0;
	}
	else{
		L_Rocker_Y = USART2_LORA_date[1] - 128;
	}


	if(USART2_LORA_date[2]>123 && USART2_LORA_date[2]<132)
	{
		R_Rocker_X = 0;
	}
	else{
		R_Rocker_X = -(USART2_LORA_date[2] - 127);
	}


	if(USART2_LORA_date[3]>123 && USART2_LORA_date[3]<132)
	{
		R_Rocker_Y = 0;
	}
	else{
		R_Rocker_Y = USART2_LORA_date[3]-127;
	}



	L_Pot = USART2_LORA_date[4];
	R_Pot = USART2_LORA_date[5];
	
	Botton[0] = USART2_LORA_date[6] & 0x01;
	Botton[1] = USART2_LORA_date[6]>>1 & 0x01;
	Botton[2] = USART2_LORA_date[6]>>2 & 0x01;
	Botton[3] = USART2_LORA_date[6]>>3 & 0x01;
	Botton[4] = USART2_LORA_date[6]>>4 & 0x01;
	Botton[5] = USART2_LORA_date[6]>>5 & 0x01;
	Botton[6] = USART2_LORA_date[6]>>6 & 0x01;
	Botton[7] = USART2_LORA_date[6]>>7 & 0x01;
	
}

/**************************************************************************
�������ܣ���ȡAPPң�ص�����
��ڲ�����
����  ֵ��
**************************************************************************/
void APP_Read()
{
	u8 angle_reset;
	L_Rocker_X = USART1_APP_Date[2] - 125;
	L_Rocker_Y = USART1_APP_Date[3] - 125;

	R_Rocker_X = USART1_APP_Date[4] - 125;
	R_Rocker_Y = USART1_APP_Date[5] - 125;

	R_Pot = USART1_APP_Date[6];
	angle_reset = USART1_APP_Date[7]>>2&0x01;
	if(Angle_Reset==false && angle_reset==1)
		Angle_Reset = true;
	
	APP_Slide = (USART1_APP_Date[7]>>1)&0x01;
	
//	printf("L_Rocker_X=%d   L_Rocker_Y=%d  R_Pot=%d\r\n",R_Rocker_X,R_Rocker_Y,R_Pot);
}




/**************************************************************************
�������ܣ�������ǰ���ٶ��˲�����
��ڲ�����s32
����  ֵ��s32
**************************************************************************/
#define Travel_Speed_AVERAGE_N 5
s32 Travel_Speed_average_value_buf[Travel_Speed_AVERAGE_N];
s32 Travel_Speed_average_sum = 0; 
s32 Travel_Speed_average_filter(s32 new_value)
{
    
    uint8_t count;
    Travel_Speed_average_sum -= Travel_Speed_average_value_buf[0];
    for ( count = 0; count < Travel_Speed_AVERAGE_N - 1; count++)
    {
        Travel_Speed_average_value_buf[count] = Travel_Speed_average_value_buf[count + 1] ;
    }
    Travel_Speed_average_value_buf[Travel_Speed_AVERAGE_N - 1] = new_value;
    Travel_Speed_average_sum += Travel_Speed_average_value_buf[Travel_Speed_AVERAGE_N - 1];
 
    return (Travel_Speed_average_sum /Travel_Speed_AVERAGE_N );
}

/**************************************************************************
�������ܣ�������ת���ٶ��˲�����
��ڲ�����s32
����  ֵ��s32
**************************************************************************/
#define Turn_Speed_AVERAGE_N 4
s32 Turn_Speed_average_value_buf[Turn_Speed_AVERAGE_N];
s32 Turn_Speed_average_sum = 0; 
s32 Turn_Speed_average_filter(s32 new_value)
{
    
    uint8_t count;
    Turn_Speed_average_sum -= Turn_Speed_average_value_buf[0];
    for ( count = 0; count < Turn_Speed_AVERAGE_N - 1; count++)
    {
        Turn_Speed_average_value_buf[count] = Turn_Speed_average_value_buf[count + 1] ;
    }
    Turn_Speed_average_value_buf[Turn_Speed_AVERAGE_N - 1] = new_value;
    Turn_Speed_average_sum += Turn_Speed_average_value_buf[Turn_Speed_AVERAGE_N - 1];
 
    return (Turn_Speed_average_sum /Turn_Speed_AVERAGE_N );
}

/**************************************************************************
�������ܣ��������Ȳ������˲�����
��ڲ�����float
����  ֵ��float
**************************************************************************/
#define R_Pot_Leg_Height_AVERAGE_N 7
float R_Pot_Leg_Height_average_value_buf[R_Pot_Leg_Height_AVERAGE_N];
float R_Pot_Leg_Height_average_sum = 0; 
float R_Pot_Leg_Height_average_filter(float new_value)
{
    
    uint8_t count;
    R_Pot_Leg_Height_average_sum -= R_Pot_Leg_Height_average_value_buf[0];
    for ( count = 0; count < R_Pot_Leg_Height_AVERAGE_N - 1; count++)
    {
        R_Pot_Leg_Height_average_value_buf[count] = R_Pot_Leg_Height_average_value_buf[count + 1] ;
    }
    R_Pot_Leg_Height_average_value_buf[R_Pot_Leg_Height_AVERAGE_N - 1] = new_value;
    R_Pot_Leg_Height_average_sum += R_Pot_Leg_Height_average_value_buf[R_Pot_Leg_Height_AVERAGE_N - 1];
 
    return (R_Pot_Leg_Height_average_sum /(float)R_Pot_Leg_Height_AVERAGE_N );
}

/**************************************************************************
�������ܣ�������Y��ƽ��Ƕ��˲������˲�����
��ڲ�����float
����  ֵ��float
**************************************************************************/
#define Leg_Balance_AVERAGE_N 4
float Leg_Balance_average_value_buf[Leg_Balance_AVERAGE_N];
float Leg_Balance_average_sum = 0; 
float Leg_Balance_average_filter(float new_value)
{
    
    uint8_t count;
    Leg_Balance_average_sum -= Leg_Balance_average_value_buf[0];
    for ( count = 0; count < Leg_Balance_AVERAGE_N - 1; count++)
    {
        Leg_Balance_average_value_buf[count] = Leg_Balance_average_value_buf[count + 1] ;
    }
    Leg_Balance_average_value_buf[Leg_Balance_AVERAGE_N - 1] = new_value;
    Leg_Balance_average_sum += Leg_Balance_average_value_buf[Leg_Balance_AVERAGE_N - 1];
 
    return (Leg_Balance_average_sum /(float)Leg_Balance_AVERAGE_N );
}



void LoRa_Control()
{

	if(Time_Reset>100)    //ң�������жϱ���
	{
		LoRa_Remote_Control_Flag = false;
		if(Time_Reset>10000)
			Time_Reset = 200;
	}
	
	if(LoRa_Remote_Control_Flag)    //��ȡң�����ݺ������ٶȡ�ת���ٶȵȹ���
	{
		LORA_Read();   //��ȡң������

		if(Botton[1]==1)
		{
			Travel_Speed = Travel_Speed_average_filter(L_Rocker_Y * (75 - R_Pot/7.5) );     //����ǰ���ٶ�
			Turn_Speed = Turn_Speed_average_filter(R_Rocker_X * 12);                 //����ת���ٶ�
		}
		else
		{
			Travel_Speed = Travel_Speed_average_filter(L_Rocker_Y * (75 - R_Pot/7.5) );     //����ǰ���ٶ�
			Turn_Speed = Turn_Speed_average_filter(L_Rocker_X * 12);    
		}


		R_Pot_Leg_Height = R_Pot_Leg_Height_average_filter(6.0 + (float)R_Pot/20.0);  //�����Ȳ�����
		 
		if(L_Pot_Med_Flag)       //ң�ص�һ�����ӻ���Ҫ�ı�L_Pot_Medֵ
		{
			L_Pot_Med_Flag = false;
			L_Pot_Med = L_Pot;
		}
		Leg_Balance = Leg_Balance_average_filter((L_Pot - L_Pot_Med)/12.0);     //����Pitch_YĿ��Ƕ�
		if(Leg_Balance>10.0)       //����������Сֵ
			Leg_Balance = 10.0;
		else if(Leg_Balance<-10.0)
			Leg_Balance = -10.0;
		
		if(Angle_Reset==false &&  Botton[4]==1)
			Angle_Reset = true;

		if(Botton[5] == 1)
			Pick_Up_Flag = false;

		if(Botton[6] == 1)     
		{
			//L_Pot_Med = L_Pot;
			L_Pot_Med_Flag = true;
		}
		if(Botton[7]==1)
			Pick_Up_Flag = true;
		if(Botton[2]==1 && R_Pot==0)
		{
			Walking_mode = true;
		}
		else
		{
			Walking_mode = false;
		}	
		
	}
	else       //���Ӷ˿���������
	{
		R_Rocker_X = 0;
		R_Rocker_Y = 0;
		Travel_Speed = 0;
		Turn_Speed = 0;
		Leg_Balance = 0.0;
	}

}


void APP_Control()
{
	if(APP_Time_Reset>100)    //ң�������жϱ���
	{
		APP_Remote_Control_Flag = false;
		if(APP_Time_Reset>10000)
			APP_Time_Reset = 200;
	}
	
	if(APP_Remote_Control_Flag)    //��ȡң�����ݺ������ٶȡ�ת���ٶȵȹ���
	{
		APP_Read();   //��ȡң������

		Travel_Speed = Travel_Speed_average_filter(L_Rocker_Y * (75 - R_Pot/5.5) );     //����ǰ���ٶ�
		Turn_Speed = Turn_Speed_average_filter(L_Rocker_X * 12);                 //����ת���ٶ�
		R_Pot_Leg_Height = R_Pot_Leg_Height_average_filter(6.0 + (float)R_Pot/20.0);  //�����Ȳ�����
		
		if(APP_Slide==1 && R_Pot==0)
		{
			Walking_mode = true;
		}
		else
		{
			Walking_mode = false;
		} 
	}
	else       //���Ӷ˿���������
	{
		R_Rocker_X = 0;
		R_Rocker_Y = 0;
		Travel_Speed = 0;
		Turn_Speed = 0;
		Leg_Balance = 0.0;
	}

}



/**************************************************************************
�������ܣ�ң�����ݴ�������
��ڲ�����
����  ֵ��
**************************************************************************/
void Remote_Control_task(void *pvParameters)
{
	while(1)
	{
		if(Mode_KEY)     //�������ؿ���ң�ط�ʽ
		{
			LoRa_Control();
		}
		else
		{
			APP_Control();
		}

		if(PickUpClearKEY == 0)
		{
			vTaskDelay(5); 
			if(PickUpClearKEY == 0)
			{
				Pick_Up_Flag = false;
			}
		}

		vTaskDelay(80); 

	}

}



void UPDTAT_task(void *pvParameters)
{
 
	while(1)
	{	
#if APPCONTROLMODE

		DHT11_Read_Data(&temperature,&humidity);	//��ȡ��ʪ��ֵ
		Get_Electric_Quantity_24_Average(1,10);
		Get_Electric_Quantity_12_Average(4,10);
		Get_Air_Quality_Average(0,10);

		Usart1_UpdataAPP();
	

#endif		
		vTaskDelay(1000); //��ʱ1000ms
	}

}


//OLED������
void OLED_task(void *pvParameters)
{
    while(1)
    {
		//OLED_Clear();				//OLED??
		OLED_ShowString(45,1,"     ",12);
		OLED_ShowString(1,1,"Roll:",12);
		OLED_Float(1,45,Roll_X,1);

		OLED_ShowString(50,2,"         ",12);
		OLED_ShowString(1,2, "Pitch:",12);
		OLED_Float(2,50,Pitch_Y,2);

		OLED_ShowString(20,3,"   ",12);
		OLED_ShowString(1,3, "T:",12);
		OLED_ShowNumber(20,3,temperature,2,12);

		OLED_ShowString(20,4,"       ",12);
		OLED_ShowString(1,4, "H:",12);
		OLED_ShowNumber(20,4,humidity,2,12);

		OLED_ShowString(85,4,"       ",12);
		OLED_ShowString(60,4, "AQ:",12);
		OLED_ShowNumber(85,4,humidity,2,12);

		OLED_ShowString(30,5,"       ",12);
		OLED_ShowString(1,5, "V12:",12);
		OLED_ShowNumber(30,5,Electric_Quantity_12,2,12);

		OLED_ShowString(30,6,"       ",12);
		OLED_ShowString(1,6, "V24:",12);
		OLED_ShowNumber(30,6,Electric_Quantity_24,2,12);

		

        vTaskDelay(300); //��ʱ200ms
    }
}




