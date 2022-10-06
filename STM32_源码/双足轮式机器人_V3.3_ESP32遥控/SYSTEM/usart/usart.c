#include "usart.h"	  


u8 uart1_recv_data[20] = {0};  // ???????
u8 uart1_recv_len = 0;         // 接收的数据长度

u8 uart1_send_data[16] = {0};
char uart1_send_flag = 1;        // 发送完成标志位
u8 uart1_send_len=8;

u8 USART1_APP_Date[20]={0};


//功能：串口值用山外虚拟示波器绘图出来，支持16位数据等
//说明：串口发送函数依照不同的单片机而定；
//函数说明：使用的前命令：{0x03,0xFC}{数据}
//					使用后的命令：{0xFC,0x03}
//      数据类型由自己宏定义，包括无符号整型和无符号字符型
///
#define uint  unsigned int
#define uchar unsigned char
void send_char(u8 send_value)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
		USART_SendData(USART1,send_value);
	//while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束	
}


#if DEBUGMODE 
/**************************************
函数功能：将数据用虚拟示波器表示出来（单通道）
函数参数：所要发送的值

**************************************/
void virtual_Osc_send_data(u16 value)
{
   u8 value_h,value_l;
   send_char(0x03);     //发送前命令
   send_char(0xFC);     //发送前命令
	
   value_h=value/256;  //高八位
   value_l=value%256;  //低八位
   send_char(value_l);      //发送低八位
   send_char(value_h);      //发送高八位
	
   send_char(0xFC);     //发送后命令
   send_char(0x03);     //发送后命令
}


void virtual_Osc_send_data_two(u16 value_1,u16 value_2)
{
   u8 value_h,value_l;
   send_char(0x03);     //发送前命令
   send_char(0xFC);     //发送前命令
	
   value_h=value_1/256;  //高八位
   value_l=value_1%256;  //低八位
   send_char(value_l);      //发送低八位
   send_char(value_h);      //发送高八位
	
	value_h=value_2/256;  //高八位
   value_l=value_2%256;  //低八位
   send_char(value_l);      //发送低八位
   send_char(value_h);      //发送高八位
	
	
   send_char(0xFC);     //发送后命令
   send_char(0x03);     //发送后命令
}


/**************************************************
函数功能：将多个数组数据同时用虚拟示波器的图形表示出来
函数参数：array：所要发送的数组，最多支持8个数组
            len: 所发数组的长度，不得大于8
**************************************************/
u16 printf_virtual_Osc_shiboqi[8]={0};
void virtual_Osc_send_array(u16 *array,u8 len)
{
   uchar value_h[8],value_l[8],i;

	 send_char(0x03);     //发送前命令
   send_char(0xFC);     //发送前命令
   for(i=0;i<len;i++)
   {
      value_h[i]=array[i]/256;   //高八位
      value_l[i]=array[i]%256;   //低八位
      send_char(value_l[i]);      //发送低八位
      send_char(value_h[i]);      //发送高八位
		 
   }
   send_char(0xFC);     //发送后命令
   send_char(0x03);     //发送后命令
}
/**************************************************
函数功能：将多个数组数据同时用虚拟示波器的图形表示出来
函数参数：array：所要发送的数组，最多支持8个数组
            len: 所发数组的长度，不得大于8
**************************************************/
void virtual_Osc_send_array_float(float *array,u8 len)
{
	uchar i;
	union result
    {
         float d;
         unsigned char data[4];
    }r1;
	send_char(0x03);     //发送前命令
   send_char(0xFC);     //发送前命令
   for(i=0;i<len;i++)
   {
	   r1.d = array[i];
       send_char(r1.data[0]);   
	   send_char(r1.data[1]); 
	   send_char(r1.data[2]); 
	   send_char(r1.data[3]); 
   
		 
   }
   send_char(0xFC);     //发送后命令
   send_char(0x03);     //发送后命令
}
#endif

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 



void uart_init(u32 bound){
  //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef    DMA_InitStruct;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
	//USART1_RX	  GPIOA.10初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器




  
	//USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1

#if APPCONTROLMODE

	// 配置串口1的中断控制器
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;   // 在 stm32f10x.h 中找 IRQn_Type 枚举   DMA1_Channel4_IRQn
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         // 子优先级
    NVIC_Init(&NVIC_InitStructure);

	//RX
//	DMA_DeInit(DMA1_Channel5);  // DMA1 5
//	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;    // 数据寄存器(USART_DR) 地址偏移：0x04
//    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)uart1_recv_data;   // 内存地址
//    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;                   // 外设到内存
//    DMA_InitStruct.DMA_BufferSize = sizeof(uart1_recv_data)/sizeof(uart1_recv_data[0]);
//    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
//    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
//    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
//    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
//    DMA_Init(DMA1_Channel5, &DMA_InitStruct);




	//TX
	DMA_DeInit(DMA1_Channel4);  // DMA1 通道7
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;   // 数据寄存器(USART_DR) 地址偏移：0x04
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)uart1_send_data;  // 内存地址
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_BufferSize = 0;      // 寄存器的内容为0时，无论通道是否开启，都不会发生任何数据传输
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStruct);
	
	// 配置 DMA1 通道4, UART1_TX 传输完成中断
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
	

	USART_DMACmd(USART1, USART_DMAReq_Tx , ENABLE);// 使能DMA串口发送和接受请求
//	USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx , ENABLE);// 使能DMA串口发送和接受请求
//	DMA_ITConfig(DMA1_Channel5,  DMA_IT_TC, ENABLE);
//	DMA_Cmd(DMA1_Channel5, ENABLE);     // 开启接收


	DMA_Cmd(DMA1_Channel4, DISABLE);    // 禁止发送
	
#endif
	USART_ITConfig(USART1,  USART_IT_RXNE, ENABLE);//开启串口接受中断
//	USART_ITConfig(USART1,  USART_IT_IDLE, ENABLE);//开启串口接受中断
	USART_Cmd(USART1, ENABLE);                    //使能串口1 

	

}


#if APPCONTROLMODE


void USART1_Receive_Data(u8 data[],u8 i)
{ 
	u8 j=0;
	for(j=0;j<i;j++)
	{
		USART1_APP_Date[j]=(u8)data[j];
		
	}
	APP_Time_Reset = 0;
	APP_Remote_Control_Flag = true;
	
}


void USART1_Receive_Prepare(u8 data)
{
    /* 局部静态变量：接收缓存 */
    static u8 RxBuffer[21];
    /* 数据长度 *//* 数据数组下标 */
    static u8  _data_cnt = 0;
    /* 接收状态 */
    static u8 state = 0;
 
    /* 帧头1 */
    if(state==0&&data==0xfb)
    {
        state=1;
        _data_cnt = 0;
		RxBuffer[_data_cnt++] = data;
    }
    /* 帧头2 */
    else if(state==1&&data==0xfc)
    {
        state=2;
       // _data_cnt = 0;
		RxBuffer[_data_cnt++] = data;			
    }
    /* 接收数据组 */
    else if(state==2)
    {
        RxBuffer[_data_cnt++]=data;
        if((_data_cnt>=10||data==0xfd)&&_data_cnt>=9)
        {
			state=0;
			USART1_Receive_Data(RxBuffer,_data_cnt);
        }
    }
    /* 若有错误重新等待接收帧头 */
    else
        state = 0;
}


void USART1_IRQHandler(void)                	//串口2中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断
	{
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
		USART1_Receive_Prepare(Res);
	}
		

}



//void USART1_IRQHandler(void)                	//串口1中断服务程序
//{
//	    if( USART_GetITStatus(USART1, USART_IT_IDLE) != RESET )
//    {    
//		USART1->SR; // 清除空闲中断, 由软件序列清除该位(先读USART_SR，然后读USART_DR)
//        USART1->DR; // 清除空闲中断  
//        DMA_Cmd(DMA1_Channel5, DISABLE);
//		uart1_recv_len = sizeof(uart1_recv_data)/sizeof(uart1_recv_data[0]) - DMA_GetCurrDataCounter(DMA1_Channel5);// 统计收到的数据的长度
//		//printf("data_1 = %d   uart1_recv_len = %d\n", uart1_recv_data[0], uart1_recv_len);
//		if(uart1_recv_data[0] == 0xfb && uart1_recv_data[1] == 0xfc && uart1_recv_len == 9)
//			USART1_Receive_Data(uart1_recv_data, uart1_recv_len );//数据缓存
//		//uart2_recv_flag = 1;                // 接收标志置1
//		memset(uart1_recv_data, '\0', sizeof(uart1_recv_data)/sizeof(uart1_recv_data[0]));  // 清空接收缓冲区
//		USART_ClearITPendingBit( USART1, USART_IT_IDLE );			//清除空闲中断标志位

//		DMA_SetCurrDataCounter(DMA1_Channel5, sizeof(uart1_recv_data)/sizeof(uart1_recv_data[0])); 
//		DMA_Cmd(DMA1_Channel5, ENABLE);     // DMA1 通道5, UART1_RX
//    }

//}



// DMA1 通道4, UART1_TX 传输完成中断
void DMA1_Channel4_IRQHandler(void)
{
    if( DMA_GetITStatus(DMA1_IT_TC4) != RESET ) // DMA1 通道7
    {
        DMA_ClearITPendingBit(DMA1_IT_TC4);     // 清除中断
		uart1_send_flag = 1;
		DMA_Cmd(DMA1_Channel4, DISABLE);    // 关闭 DMA1 通道7
    }
}

u8 a=0; 

void Usart1_UpdataAPP()
{
	uart1_send_data[0]=0xb1;
	uart1_send_data[1]=0xb2;
	

	uart1_send_data[2]=Electric_Quantity_12;
	uart1_send_data[3]=Electric_Quantity_24;
	uart1_send_data[4]=Air_Quality;
	uart1_send_data[5]=temperature;
	uart1_send_data[6]=humidity;
		
//	uart1_send_data[2]=a;
//	uart1_send_data[3]=a*2;
//	uart1_send_data[4]=a*4;
//	uart1_send_data[5]=a*5;
//	uart1_send_data[6]=a*6;
//	
//	a++;

//	if(a>40)
//		a=0;
	
	uart1_send_data[7]=0xb3;
	
	if(uart1_send_flag==1)
	{	
		uart1_send_flag = 0;
		DMA_Cmd(DMA1_Channel4, DISABLE);    // 关闭 DMA1 通道4, UART1_TX
		DMA_SetCurrDataCounter(DMA1_Channel4, uart1_send_len);  // 传输数量寄存器只能在通道不工作(DMA_CCRx的EN=0)时写入
		DMA_Cmd(DMA1_Channel4, ENABLE);    // 开启 DMA1 通道4, UART1_TX
	}
}

#endif
