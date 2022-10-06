#include "lora_usart2.h"

//u8 uart2_recv_data[15] = {0};  // ???????
//u8 uart2_recv_len = 0;         // ���յ����ݳ���

/*
bluetooth_date[]����˵��
0-1    --- L_X  L_Y
2-3    --- R_X  R_Y
4-5    --- L_pot  R_pot
6      --- Button 0-6
*/
u8 USART2_LORA_date[15]={0};

void LORA_USART2_init(u32 bound){
  //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef    DMA_InitStruct;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  // ??DMA1???

	//USART2_TX   GPIOA2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA2
   
	//USART2_RX	  GPIOA3��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA3

	//Usart2 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure); //��ʼ������2
	
//	DMA_DeInit(DMA1_Channel6);  // DMA1 ??3
//	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;    // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
//    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)uart2_recv_data;   // �ڴ��ַ
//    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;                   // ���赽�ڴ�
//    DMA_InitStruct.DMA_BufferSize = sizeof(uart2_recv_data)/sizeof(uart2_recv_data[0]);
//    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
//    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
//    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
//    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
//    DMA_Init(DMA1_Channel6, &DMA_InitStruct);

//	USART_DMACmd(USART2, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);// ʹ��DMA���ڷ��ͺͽ�������
//	DMA_ITConfig(DMA1_Channel6,  DMA_IT_TC, ENABLE);

//	DMA_Cmd(DMA1_Channel6, ENABLE);     // ��������

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���2 

}

void USART2_IRQHandler(void)                	//����2�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�
	{
		Res =USART_ReceiveData(USART2);	//��ȡ���յ�������
		USART2_Receive_Prepare(Res);
	}
		

}


//// ����2���жϴ�����
//void USART2_IRQHandler(void)
//{   
//    if( USART_GetITStatus(USART2, USART_IT_IDLE) != RESET )
//    {    
//		USART2->SR; // ��������ж�, ��������������λ(�ȶ�USART_SR��Ȼ���USART_DR)
//        USART2->DR; // ��������ж�  
//        DMA_Cmd(DMA1_Channel6, DISABLE);
//		uart2_recv_len = sizeof(uart2_recv_data)/sizeof(uart2_recv_data[0]) - DMA_GetCurrDataCounter(DMA1_Channel6);// ͳ���յ������ݵĳ���
//		printf("data_1 = %d   data_2 = %d\n", uart2_recv_data[0], uart2_recv_data[1]);
//		if(uart2_recv_data[0] == 0xA6 )//&& uart2_recv_data[1] == 0x6A)
//			USART2_Receive_Data(uart2_recv_data, uart2_recv_len);//���ݻ���
//		//uart2_recv_flag = 1;                // ���ձ�־��1
//		memset(uart2_recv_data, '\0', sizeof(uart2_recv_data)/sizeof(uart2_recv_data[0]));  // ��ս��ջ�����
//		USART_ClearITPendingBit(USART2, USART_IT_IDLE );			//��������жϱ�־λ

//		DMA_SetCurrDataCounter(DMA1_Channel6, sizeof(uart2_recv_data)/sizeof(uart2_recv_data[0])); 
//		DMA_Cmd(DMA1_Channel6, ENABLE);     // DMA1 ͨ��5, UART1_RX
//    }
//}


void USART2_Receive_Prepare(u8 data)
{
    /* �ֲ���̬���������ջ��� */
    static u8 RxBuffer[21];
    /* ���ݳ��� *//* ���������±� */
    static u8  _data_cnt = 0;
    /* ����״̬ */
    static u8 state = 0;
 
    /* ֡ͷ1 */
    if(state==0&&data==0xA6)
    {
        state=1;
        _data_cnt = 0;
		//	RxBuffer[_data_cnt++] = data;
    }
    /* ֡ͷ2 */
    else if(state==1&&data==0x6A)
    {
        state=2;
        _data_cnt = 0;
			//RxBuffer[_data_cnt++] = data;			
    }
    /* ���������� */
    else if(state==2)
    {
        RxBuffer[_data_cnt++]=data;
        if((_data_cnt>=10||data==0xAA)&&_data_cnt>=7)
        {
			state=0;
			USART2_Receive_Data(RxBuffer,_data_cnt);
        }
    }
    /* ���д������µȴ�����֡ͷ */
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



