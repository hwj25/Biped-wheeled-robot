#include "usart.h"	  


u8 uart1_recv_data[20] = {0};  // ???????
u8 uart1_recv_len = 0;         // ���յ����ݳ���

u8 uart1_send_data[16] = {0};
char uart1_send_flag = 1;        // ������ɱ�־λ
u8 uart1_send_len=8;

u8 USART1_APP_Date[20]={0};


//���ܣ�����ֵ��ɽ������ʾ������ͼ������֧��16λ���ݵ�
//˵�������ڷ��ͺ������ղ�ͬ�ĵ�Ƭ��������
//����˵����ʹ�õ�ǰ���{0x03,0xFC}{����}
//					ʹ�ú�����{0xFC,0x03}
//      �����������Լ��궨�壬�����޷������ͺ��޷����ַ���
///
#define uint  unsigned int
#define uchar unsigned char
void send_char(u8 send_value)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
		USART_SendData(USART1,send_value);
	//while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���	
}


#if DEBUGMODE 
/**************************************
�������ܣ�������������ʾ������ʾ��������ͨ����
������������Ҫ���͵�ֵ

**************************************/
void virtual_Osc_send_data(u16 value)
{
   u8 value_h,value_l;
   send_char(0x03);     //����ǰ����
   send_char(0xFC);     //����ǰ����
	
   value_h=value/256;  //�߰�λ
   value_l=value%256;  //�Ͱ�λ
   send_char(value_l);      //���͵Ͱ�λ
   send_char(value_h);      //���͸߰�λ
	
   send_char(0xFC);     //���ͺ�����
   send_char(0x03);     //���ͺ�����
}


void virtual_Osc_send_data_two(u16 value_1,u16 value_2)
{
   u8 value_h,value_l;
   send_char(0x03);     //����ǰ����
   send_char(0xFC);     //����ǰ����
	
   value_h=value_1/256;  //�߰�λ
   value_l=value_1%256;  //�Ͱ�λ
   send_char(value_l);      //���͵Ͱ�λ
   send_char(value_h);      //���͸߰�λ
	
	value_h=value_2/256;  //�߰�λ
   value_l=value_2%256;  //�Ͱ�λ
   send_char(value_l);      //���͵Ͱ�λ
   send_char(value_h);      //���͸߰�λ
	
	
   send_char(0xFC);     //���ͺ�����
   send_char(0x03);     //���ͺ�����
}


/**************************************************
�������ܣ��������������ͬʱ������ʾ������ͼ�α�ʾ����
����������array����Ҫ���͵����飬���֧��8������
            len: ��������ĳ��ȣ����ô���8
**************************************************/
u16 printf_virtual_Osc_shiboqi[8]={0};
void virtual_Osc_send_array(u16 *array,u8 len)
{
   uchar value_h[8],value_l[8],i;

	 send_char(0x03);     //����ǰ����
   send_char(0xFC);     //����ǰ����
   for(i=0;i<len;i++)
   {
      value_h[i]=array[i]/256;   //�߰�λ
      value_l[i]=array[i]%256;   //�Ͱ�λ
      send_char(value_l[i]);      //���͵Ͱ�λ
      send_char(value_h[i]);      //���͸߰�λ
		 
   }
   send_char(0xFC);     //���ͺ�����
   send_char(0x03);     //���ͺ�����
}
/**************************************************
�������ܣ��������������ͬʱ������ʾ������ͼ�α�ʾ����
����������array����Ҫ���͵����飬���֧��8������
            len: ��������ĳ��ȣ����ô���8
**************************************************/
void virtual_Osc_send_array_float(float *array,u8 len)
{
	uchar i;
	union result
    {
         float d;
         unsigned char data[4];
    }r1;
	send_char(0x03);     //����ǰ����
   send_char(0xFC);     //����ǰ����
   for(i=0;i<len;i++)
   {
	   r1.d = array[i];
       send_char(r1.data[0]);   
	   send_char(r1.data[1]); 
	   send_char(r1.data[2]); 
	   send_char(r1.data[3]); 
   
		 
   }
   send_char(0xFC);     //���ͺ�����
   send_char(0x03);     //���ͺ�����
}
#endif

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 



void uart_init(u32 bound){
  //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef    DMA_InitStruct;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
	//USART1_RX	  GPIOA.10��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���




  
	//USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //��ʼ������1

#if APPCONTROLMODE

	// ���ô���1���жϿ�����
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;   // �� stm32f10x.h ���� IRQn_Type ö��   DMA1_Channel4_IRQn
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  // ��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         // �����ȼ�
    NVIC_Init(&NVIC_InitStructure);

	//RX
//	DMA_DeInit(DMA1_Channel5);  // DMA1 5
//	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;    // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
//    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)uart1_recv_data;   // �ڴ��ַ
//    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;                   // ���赽�ڴ�
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
	DMA_DeInit(DMA1_Channel4);  // DMA1 ͨ��7
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;   // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)uart1_send_data;  // �ڴ��ַ
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_BufferSize = 0;      // �Ĵ���������Ϊ0ʱ������ͨ���Ƿ����������ᷢ���κ����ݴ���
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStruct);
	
	// ���� DMA1 ͨ��4, UART1_TX ��������ж�
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
	

	USART_DMACmd(USART1, USART_DMAReq_Tx , ENABLE);// ʹ��DMA���ڷ��ͺͽ�������
//	USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx , ENABLE);// ʹ��DMA���ڷ��ͺͽ�������
//	DMA_ITConfig(DMA1_Channel5,  DMA_IT_TC, ENABLE);
//	DMA_Cmd(DMA1_Channel5, ENABLE);     // ��������


	DMA_Cmd(DMA1_Channel4, DISABLE);    // ��ֹ����
	
#endif
	USART_ITConfig(USART1,  USART_IT_RXNE, ENABLE);//�������ڽ����ж�
//	USART_ITConfig(USART1,  USART_IT_IDLE, ENABLE);//�������ڽ����ж�
	USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 

	

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
    /* �ֲ���̬���������ջ��� */
    static u8 RxBuffer[21];
    /* ���ݳ��� *//* ���������±� */
    static u8  _data_cnt = 0;
    /* ����״̬ */
    static u8 state = 0;
 
    /* ֡ͷ1 */
    if(state==0&&data==0xfb)
    {
        state=1;
        _data_cnt = 0;
		RxBuffer[_data_cnt++] = data;
    }
    /* ֡ͷ2 */
    else if(state==1&&data==0xfc)
    {
        state=2;
       // _data_cnt = 0;
		RxBuffer[_data_cnt++] = data;			
    }
    /* ���������� */
    else if(state==2)
    {
        RxBuffer[_data_cnt++]=data;
        if((_data_cnt>=10||data==0xfd)&&_data_cnt>=9)
        {
			state=0;
			USART1_Receive_Data(RxBuffer,_data_cnt);
        }
    }
    /* ���д������µȴ�����֡ͷ */
    else
        state = 0;
}


void USART1_IRQHandler(void)                	//����2�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�
	{
		Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
		USART1_Receive_Prepare(Res);
	}
		

}



//void USART1_IRQHandler(void)                	//����1�жϷ������
//{
//	    if( USART_GetITStatus(USART1, USART_IT_IDLE) != RESET )
//    {    
//		USART1->SR; // ��������ж�, ��������������λ(�ȶ�USART_SR��Ȼ���USART_DR)
//        USART1->DR; // ��������ж�  
//        DMA_Cmd(DMA1_Channel5, DISABLE);
//		uart1_recv_len = sizeof(uart1_recv_data)/sizeof(uart1_recv_data[0]) - DMA_GetCurrDataCounter(DMA1_Channel5);// ͳ���յ������ݵĳ���
//		//printf("data_1 = %d   uart1_recv_len = %d\n", uart1_recv_data[0], uart1_recv_len);
//		if(uart1_recv_data[0] == 0xfb && uart1_recv_data[1] == 0xfc && uart1_recv_len == 9)
//			USART1_Receive_Data(uart1_recv_data, uart1_recv_len );//���ݻ���
//		//uart2_recv_flag = 1;                // ���ձ�־��1
//		memset(uart1_recv_data, '\0', sizeof(uart1_recv_data)/sizeof(uart1_recv_data[0]));  // ��ս��ջ�����
//		USART_ClearITPendingBit( USART1, USART_IT_IDLE );			//��������жϱ�־λ

//		DMA_SetCurrDataCounter(DMA1_Channel5, sizeof(uart1_recv_data)/sizeof(uart1_recv_data[0])); 
//		DMA_Cmd(DMA1_Channel5, ENABLE);     // DMA1 ͨ��5, UART1_RX
//    }

//}



// DMA1 ͨ��4, UART1_TX ��������ж�
void DMA1_Channel4_IRQHandler(void)
{
    if( DMA_GetITStatus(DMA1_IT_TC4) != RESET ) // DMA1 ͨ��7
    {
        DMA_ClearITPendingBit(DMA1_IT_TC4);     // ����ж�
		uart1_send_flag = 1;
		DMA_Cmd(DMA1_Channel4, DISABLE);    // �ر� DMA1 ͨ��7
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
		DMA_Cmd(DMA1_Channel4, DISABLE);    // �ر� DMA1 ͨ��4, UART1_TX
		DMA_SetCurrDataCounter(DMA1_Channel4, uart1_send_len);  // ���������Ĵ���ֻ����ͨ��������(DMA_CCRx��EN=0)ʱд��
		DMA_Cmd(DMA1_Channel4, ENABLE);    // ���� DMA1 ͨ��4, UART1_TX
	}
}

#endif
