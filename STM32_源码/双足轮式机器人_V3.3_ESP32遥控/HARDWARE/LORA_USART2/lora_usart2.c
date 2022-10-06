#include "lora_usart2.h"

//u8 uart2_recv_data[15] = {0};  // ???????
//u8 uart2_recv_len = 0;         // 接收的数据长度

/*
bluetooth_date[]数据说明
0-1    --- L_X  L_Y
2-3    --- R_X  R_Y
4-5    --- L_pot  R_pot
6      --- Button 0-6
*/
u8 USART2_LORA_date[15]={0};

void LORA_USART2_init(u32 bound){
  //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef    DMA_InitStruct;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  // ??DMA1???

	//USART2_TX   GPIOA2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA2
   
	//USART2_RX	  GPIOA3初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA3

	//Usart2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART2, &USART_InitStructure); //初始化串口2
	
//	DMA_DeInit(DMA1_Channel6);  // DMA1 ??3
//	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;    // 数据寄存器(USART_DR) 地址偏移：0x04
//    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)uart2_recv_data;   // 内存地址
//    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;                   // 外设到内存
//    DMA_InitStruct.DMA_BufferSize = sizeof(uart2_recv_data)/sizeof(uart2_recv_data[0]);
//    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
//    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
//    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
//    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
//    DMA_Init(DMA1_Channel6, &DMA_InitStruct);

//	USART_DMACmd(USART2, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);// 使能DMA串口发送和接受请求
//	DMA_ITConfig(DMA1_Channel6,  DMA_IT_TC, ENABLE);

//	DMA_Cmd(DMA1_Channel6, ENABLE);     // 开启接收

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART2, ENABLE);                    //使能串口2 

}

void USART2_IRQHandler(void)                	//串口2中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
	{
		Res =USART_ReceiveData(USART2);	//读取接收到的数据
		USART2_Receive_Prepare(Res);
	}
		

}


//// 串口2的中断处理函数
//void USART2_IRQHandler(void)
//{   
//    if( USART_GetITStatus(USART2, USART_IT_IDLE) != RESET )
//    {    
//		USART2->SR; // 清除空闲中断, 由软件序列清除该位(先读USART_SR，然后读USART_DR)
//        USART2->DR; // 清除空闲中断  
//        DMA_Cmd(DMA1_Channel6, DISABLE);
//		uart2_recv_len = sizeof(uart2_recv_data)/sizeof(uart2_recv_data[0]) - DMA_GetCurrDataCounter(DMA1_Channel6);// 统计收到的数据的长度
//		printf("data_1 = %d   data_2 = %d\n", uart2_recv_data[0], uart2_recv_data[1]);
//		if(uart2_recv_data[0] == 0xA6 )//&& uart2_recv_data[1] == 0x6A)
//			USART2_Receive_Data(uart2_recv_data, uart2_recv_len);//数据缓存
//		//uart2_recv_flag = 1;                // 接收标志置1
//		memset(uart2_recv_data, '\0', sizeof(uart2_recv_data)/sizeof(uart2_recv_data[0]));  // 清空接收缓冲区
//		USART_ClearITPendingBit(USART2, USART_IT_IDLE );			//清除空闲中断标志位

//		DMA_SetCurrDataCounter(DMA1_Channel6, sizeof(uart2_recv_data)/sizeof(uart2_recv_data[0])); 
//		DMA_Cmd(DMA1_Channel6, ENABLE);     // DMA1 通道5, UART1_RX
//    }
//}


void USART2_Receive_Prepare(u8 data)
{
    /* 局部静态变量：接收缓存 */
    static u8 RxBuffer[21];
    /* 数据长度 *//* 数据数组下标 */
    static u8  _data_cnt = 0;
    /* 接收状态 */
    static u8 state = 0;
 
    /* 帧头1 */
    if(state==0&&data==0xA6)
    {
        state=1;
        _data_cnt = 0;
		//	RxBuffer[_data_cnt++] = data;
    }
    /* 帧头2 */
    else if(state==1&&data==0x6A)
    {
        state=2;
        _data_cnt = 0;
			//RxBuffer[_data_cnt++] = data;			
    }
    /* 接收数据组 */
    else if(state==2)
    {
        RxBuffer[_data_cnt++]=data;
        if((_data_cnt>=10||data==0xAA)&&_data_cnt>=7)
        {
			state=0;
			USART2_Receive_Data(RxBuffer,_data_cnt);
        }
    }
    /* 若有错误重新等待接收帧头 */
    else
        state = 0;
}

void USART2_Receive_Data(u8 data[],u8 i)
{ 
	u8 j=0;
	for(j=0;j<i;j++)
	{
		USART2_LORA_date[j]=(u8)data[j];
		
	}
	LoRa_Remote_Control_Flag = true;
	Time_Reset = 0;
	
}



