#include "jy901_usart3.h"

float Aacx,Aacy,Aacz;		//加速度
float Gyrox,Gyroy,Gyroz;	//陀螺仪--角速度
float Roll_X,Pitch_Y,Yaw_Z;		//角度

u8 uart3_recv_data[40] = {0};  // ???????
u8 uart3_recv_len = 0;         // 接收的数据长度

u8 USART3_JY901_Date[40]={0};

void JY901_USART3_init(u32 bound){
  //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef    DMA_InitStruct;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能USART3，GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  // ??DMA1???

	//USART3_TX   GPIOB10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB10
   
	//USART3_RX	  GPIOB11初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB11

	//USART3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART3, &USART_InitStructure); //初始化串口2

	DMA_DeInit(DMA1_Channel3);  // DMA1 ??3
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;    // 数据寄存器(USART_DR) 地址偏移：0x04
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)uart3_recv_data;   // 内存地址
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;                   // 外设到内存
    DMA_InitStruct.DMA_BufferSize = sizeof(uart3_recv_data)/sizeof(uart3_recv_data[0]);
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel3, &DMA_InitStruct);

	USART_DMACmd(USART3, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);// 使能DMA串口发送和接受请求
	DMA_ITConfig(DMA1_Channel3,  DMA_IT_TC, ENABLE);

	DMA_Cmd(DMA1_Channel3, ENABLE);     // 开启接收

	USART_ITConfig(USART3,  USART_IT_IDLE, ENABLE);//开启串口接受中断
	USART_Cmd(USART3, ENABLE);                    //使能串口3 

}

//void USART3_IRQHandler(void)                	//串口2中断服务程序
//{
//	u8 Res;
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断
//	{
//		Res =USART_ReceiveData(USART3);	//读取接收到的数据
//		USART3_Receive_Prepare(Res);
//	}
//		

//}

// 串口3的中断处理函数
void USART3_IRQHandler(void)
{   
    if( USART_GetITStatus(USART3, USART_IT_IDLE) != RESET )
    {    
		USART3->SR; // 清除空闲中断, 由软件序列清除该位(先读USART_SR，然后读USART_DR)
        USART3->DR; // 清除空闲中断  
        DMA_Cmd(DMA1_Channel3, DISABLE);
		uart3_recv_len = sizeof(uart3_recv_data)/sizeof(uart3_recv_data[0]) - DMA_GetCurrDataCounter(DMA1_Channel3);// 统计收到的数据的长度
		//printf("data_1 = %d   data_2 = %d\n", uart3_recv_data[0], uart3_recv_data[1]);
		if(uart3_recv_data[0] == 0x55 && uart3_recv_data[1] == 0x51)
			USART3_Receive_Data(uart3_recv_data, uart3_recv_len );//数据缓存
		//uart2_recv_flag = 1;                // 接收标志置1
		memset(uart3_recv_data, '\0', sizeof(uart3_recv_data)/sizeof(uart3_recv_data[0]));  // 清空接收缓冲区
		USART_ClearITPendingBit( USART3, USART_IT_IDLE );			//清除空闲中断标志位

		DMA_SetCurrDataCounter(DMA1_Channel3, sizeof(uart3_recv_data)/sizeof(uart3_recv_data[0])); 
		DMA_Cmd(DMA1_Channel3, ENABLE);     // DMA1 通道5, UART1_RX
    }
}

void USART3_Receive_Prepare(u8 data)
{
    /* 局部静态变量：接收缓存 */
    static u8 RxBuffer[40];
    /* 数据长度 *//* 数据数组下标 */
    static u8  _data_cnt = 0;
    /* 接收状态 */
    static u8 state = 0;
 
    /* 帧头1 */
    if(state==0&&data==0x55)
    {
        state=1;
		_data_cnt=0;
        RxBuffer[_data_cnt++]=data;
    }
    /* 帧头2 */
    else if(state==1&&data==0x51)
    {
        state=2;
        RxBuffer[_data_cnt++]=data;
    }
    /* 接收数据组 */
    else if(state==2)
    {
        RxBuffer[_data_cnt++]=data;
        if(_data_cnt>=33)
        {
			state=0;
			USART3_Receive_Data(RxBuffer,_data_cnt);
			_data_cnt=0;
        }
    }
    /* 若有错误重新等待接收帧头 */
    else
        state = 0;
}

void USART3_Receive_Data(u8 data[],u8 i)
{ 
	u8 j=0;
	for(j=0;j<i;j++)
	{
		USART3_JY901_Date[j]=(u8)data[j];
		
	}
	JY910_Set();
}



void USART3_char(u8 send_value)
{
	USART_SendData(USART3, send_value);//向串口1发送数据
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//等待发送结束
}


#define Roll_X_AVERAGE_N 3
float Roll_X_average_value_buf[Roll_X_AVERAGE_N];
float Roll_X_average_sum = 0; 
float Roll_X_average_filter(float new_value)
{
    
    uint8_t count;
    Roll_X_average_sum -= Roll_X_average_value_buf[0];
    for ( count = 0; count < Roll_X_AVERAGE_N - 1; count++)
    {
        Roll_X_average_value_buf[count] = Roll_X_average_value_buf[count + 1] ;
    }
    Roll_X_average_value_buf[Roll_X_AVERAGE_N - 1] = new_value;
    Roll_X_average_sum += Roll_X_average_value_buf[Roll_X_AVERAGE_N - 1];
 
    return (Roll_X_average_sum /(float)Roll_X_AVERAGE_N );
}



void JY910_Set()
{

//	Aacx = (float)((short)(USART3_JY901_Date[3]<<8)|USART3_JY901_Date[2])/32768 * 16;
//	Aacy = (float)((short)(USART3_JY901_Date[5]<<8)|USART3_JY901_Date[4])/32768 * 16;
	Aacz = (float)((short)(USART3_JY901_Date[7]<<8)|USART3_JY901_Date[6])/32768 * 16;   //机器人拿起检测


	Gyrox = (float)((short)(USART3_JY901_Date[14]<<8)|USART3_JY901_Date[13])/32768 * 2000;  //自立环
//	Gyroy = (float)((short)(USART3_JY901_Date[16]<<8)|USART3_JY901_Date[15])/32768 * 2000;  //
	Gyroz = (float)((short)(USART3_JY901_Date[18]<<8)|USART3_JY901_Date[17])/32768 * 2000;  //转弯

	Roll_X = (float)((USART3_JY901_Date[25]<<8)|USART3_JY901_Date[24])/32768 * 180;  //自立环
	Pitch_Y = (float)((USART3_JY901_Date[27]<<8)|USART3_JY901_Date[26])/32768 * 180;  //左右平衡环
	//Yaw_Z = (float)((USART3_JY901_Date[29]<<8)|USART3_JY901_Date[28])/32768 * 180;
	
//	Roll_X = Roll_X_average_filter(Roll_X);

	//if(Roll_X>180)  Roll_X  = Roll_X-360;
	if(Pitch_Y>180) Pitch_Y = Pitch_Y-360;		
	//if(Yaw_Z>180)  Yaw_Z  = Yaw_Z-360;
	
}




