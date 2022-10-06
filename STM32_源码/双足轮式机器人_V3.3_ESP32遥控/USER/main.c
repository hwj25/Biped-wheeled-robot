#include "sys.h"
//任务参数--------------------------
//优先级 堆栈大小 任务句柄 任务函数
#define START_TASK_PRIO     1
#define START_STK_SIZE      128  
TaskHandle_t StartTask_Handler;
void start_task(void *pvParameters);

#define TASK1_TASK_PRIO     6
#define TASK1_STK_SIZE      128  
TaskHandle_t Task1Task_Handler;
void task1_task(void *pvParameters);

#define LEG_CONTROL_TASK_PRIO     2
#define LEG_CONTROL_STK_SIZE      256  
TaskHandle_t LEG_CONTROLTask_Handler;
void Leg_Control_task(void *pvParameters);

#define REMOTE_CONTROL_TASK_PRIO     3
#define REMOTE_CONTROL_STK_SIZE      128  
TaskHandle_t REMOTE_CONTROLTask_Handler;
void Remote_Control_task(void *pvParameters);


#define PAN_TILT_TASK_PRIO     4       //
#define PAN_TILT_STK_SIZE      128  
TaskHandle_t PAN_TILTTask_Handler;
void PAN_TILT_task(void *pvParameters);

#define UPDTAT_TASK_PRIO     5   
#define UPDTAT_STK_SIZE      128  
TaskHandle_t UPDTATTask_Handler;
void UPDTAT_task(void *pvParameters);

#define OLED_TASK_PRIO     6   
#define OLED_STK_SIZE      128  
TaskHandle_t OLEDTask_Handler;
void OLED_task(void *pvParameters);



s16 p = 1000, i = 0, d = 0;
float float_send[8] = {0};
float leg_x=0.0,leg_y=6.0;
u16 duoji[4] = {1500,1500,1500,1500};
void all_Init()					
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	delay_init();	    	 //延时函数初始化

//	LED_Init();		  		//初始化与LED连接的硬件接口
	GPIO_In_Init();

	uart_init(115200);  		//串口
	LORA_USART2_init(115200);
	JY901_USART3_init(115200);

	Pan_Tilt_TIM4_PWM_Init(20000-1,72-1);

	pid_first_param_init();   		//pid初始化

	TIM3_PWM_Init(20000-1,72-1);
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,2,CAN_Mode_Normal);//CAN初始化环回模式,波特率1Mbps    
	TIM1_Init_Clock(100-1,7200-1);
	
	DHT11_Init();
	Adc_Init();
	
	OLED_Init();				//OLED初始化

}

int main(void)
{	
	 all_Init();	           //整体初始化
	     //创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄  
    //开启任务调度                
    vTaskStartScheduler();
} 
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
    
    //创建TASK1任务     debug任务
    xTaskCreate((TaskFunction_t )task1_task,             
                (const char*    )"task1_task",           
                (uint16_t       )TASK1_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )TASK1_TASK_PRIO,        
                (TaskHandle_t*  )&Task1Task_Handler);   
	
	//创建Leg_Control任务    腿部长度控制任务
    xTaskCreate((TaskFunction_t )Leg_Control_task,             
                (const char*    )"Leg_Control_task",           
                (uint16_t       )LEG_CONTROL_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )LEG_CONTROL_TASK_PRIO,        
                (TaskHandle_t*  )&LEG_CONTROLTask_Handler); 
	
	//创建REMOTE_CONTROL任务   遥控数据处理任务
    xTaskCreate((TaskFunction_t )Remote_Control_task,             
                (const char*    )"Remote_Control_task",           
                (uint16_t       )REMOTE_CONTROL_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )REMOTE_CONTROL_TASK_PRIO,        
                (TaskHandle_t*  )&REMOTE_CONTROLTask_Handler); 

	//创建PAN_TILT任务       云台运动控制函数
    xTaskCreate((TaskFunction_t )PAN_TILT_task,     
                (const char*    )"PAN_TILT_task",   
                (uint16_t       )PAN_TILT_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )PAN_TILT_TASK_PRIO,
                (TaskHandle_t*  )&PAN_TILTTask_Handler); 

	//创建OLED任务         UPDATA显示控制任务
    xTaskCreate((TaskFunction_t )UPDTAT_task,     
                (const char*    )"UPDTAT_task",   
                (uint16_t       )UPDTAT_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )UPDTAT_TASK_PRIO,
                (TaskHandle_t*  )&UPDTATTask_Handler); 

//    //创建OLED任务         OLED显示控制任务
    xTaskCreate((TaskFunction_t )OLED_task,     
                (const char*    )"OLED_task",   
                (uint16_t       )OLED_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )OLED_TASK_PRIO,
                (TaskHandle_t*  )&OLEDTask_Handler); 
                
    vTaskDelete(StartTask_Handler); //删除开始任务
                
    taskEXIT_CRITICAL();            //退出临界区
}
//task1任务函数
void task1_task(void *pvParameters)
{
    while(1)
    {
#if DEBUGMODE
		//CAN1_Send_Msg(0x200,8,p,p,0,0);
		//send_chassis_cur1_4(p,p,0,0);
//		TIM_SetCompare1(TIM3,duoji[0]);
//		TIM_SetCompare2(TIM3,duoji[1]);
//		TIM_SetCompare3(TIM4,duoji[2]);
//		TIM_SetCompare4(TIM4,duoji[3]);

//		float_send[0]=Aacx;
//		float_send[1]=Aacy;
//		float_send[2]=Aacz;
		
//		float_send[0]=Aacz;
//		float_send[1]=Gyrox;
//		float_send[2]=Gyroz;
//		float_send[3]=Roll_X;
//		float_send[4]=Pitch_Y;
		
//		
//		float_send[0]=Med_Angle+pid_Velocity.ActualOut;
//		float_send[1]=Roll_X;
//		float_send[0]=-6.5+Leg_Balance;
//		float_send[1]=Pitch_Y;
////		//float_send[3]=(float)(moto_chassis[0].speed_rpm-moto_chassis[1].speed_rpm);
////		float_send[2]=Pitch_Y;
//		virtual_Osc_send_array_float(float_send,2);
		

//		//printf("Aacx= %f  Aacy= %f   Aacz= %f \n",Aacx,Aacy,Aacz);
//		//printf("Gyrox= %d  Gyroy= %d   Gyroz= %d \n",Gyrox,Gyroy,Gyroz);
//		//printf("Roll= %f  Pitch= %f   Yaw= %f \n",Roll,Pitch,Yaw);
//		//printf("\n Travel_Speed = %d\n",Travel_Speed);

//			Leg_Left_Height(leg_x,leg_y);
//			Leg_Right_Height(leg_x,leg_y);

		
		//printf("L_Rocker_X= %d  L_Rocker_Y= %d   R_Rocker_X= %d  R_Rocker_Y=%d   %d    %d\n",L_Pot,R_Pot,Botton[2],Botton[3],Botton[4],Botton[5]);
//		Leg_Left_Height(leg_x,leg_y);
//		Leg_Right_Height(leg_x,leg_y);

#endif
        vTaskDelay(30); //延时100ms
    }
}




