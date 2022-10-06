 #include "bsp_can.h"

/*
 * 函数名：CAN_GPIO_Config
 * 描述  ：CAN的GPIO 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_GPIO_Config(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;   	

  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(CAN_TX_GPIO_CLK|CAN_RX_GPIO_CLK, ENABLE);
	
	//重映射引脚
//  GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

	 /* Configure CAN TX pins */
  GPIO_InitStructure.GPIO_Pin = CAN_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		         // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CAN_TX_GPIO_PORT, &GPIO_InitStructure);
	
	/* Configure CAN RX  pins */
  GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	             // 上拉输入
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CAN_RX_GPIO_PORT, &GPIO_InitStructure);

	
}



/*
 * 函数名：CAN_Mode_Config
 * 描述  ：CAN的模式 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Mode_Config(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	/************************CAN通信参数设置**********************************/
	/* Enable CAN clock */
  RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);

	/*CAN寄存器初始化*/
//	CAN_DeInit(CANx);
//	CAN_StructInit(&CAN_InitStructure);

	/*CAN单元初始化*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  关闭时间触发通信模式使能
	CAN_InitStructure.CAN_ABOM=DISABLE;			   //MCR-ABOM  自动离线管理 
	CAN_InitStructure.CAN_AWUM=DISABLE;			   //MCR-AWUM  使用自动唤醒模式
	CAN_InitStructure.CAN_NART=ENABLE;			   //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
	CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
	CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符 
	

	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;   //普通工作模式

//	CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;  //回环工作模式

	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW 重新同步跳跃宽度 2个时间单元  1
	 
	/* ss=1 bs1=5 bs2=3 位时间宽度为(1+5+3) 波特率即为时钟周期tq*(1+3+5)  */
	CAN_InitStructure.CAN_BS1=CAN_BS1_1tq;		   //BTR-TS1 时间段1 占用了5个时间单元
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;		   //BTR-TS1 时间段2 占用了3个时间单元	
	
	/* CAN Baudrate = 1 MBps (1MBps已为stm32的CAN最高速率) (CAN 时钟频率为 APB1 = 36 MHz) */
	CAN_InitStructure.CAN_Prescaler =6;		   ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 36/(1+5+3)/4=1 Mbps
	CAN_Init(CANx, &CAN_InitStructure);
}

/*
 * 函数名：CAN_Filter_Config
 * 描述  ：CAN的过滤器 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Filter_Config(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CAN筛选器初始化*/
	CAN_FilterInitStructure.CAN_FilterNumber=0;						//筛选器组0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在掩码模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//筛选器位宽为单个32位。
	/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

	CAN_FilterInitStructure.CAN_FilterIdHigh= 0x0000;		//要筛选的ID高位 
	CAN_FilterInitStructure.CAN_FilterIdLow= 0x0000; //要筛选的ID低位 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000;			//筛选器高16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;			//筛选器低16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//筛选器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能筛选器
	CAN_FilterInit(&CAN_FilterInitStructure);

}

/*
 * 函数名：CAN_NVIC_Config
 * 描述  ：CAN的NVIC 配置,第1优先级组，0，0优先级
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_NVIC_Config(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
//	/* Configure one bit for preemption priority */
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	/*CAN通信中断使能*/
//	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
//    CAN_ITConfig(CANx,CAN_IT_TME,ENABLE);
//	CAN_ITConfig(CANx,CAN_IT_BOF,ENABLE);
	CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE); //??FIFO0??????

	/*中断设置*/
	NVIC_InitStructure.NVIC_IRQChannel = CAN_RX_IRQ;	   //CAN1 RX0中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		   //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			   //子优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
 * 函数名：CAN_Config
 * 描述  ：完整配置CAN的功能
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
u8 CAN_Config(void)
{
  CAN_GPIO_Config();  
  CAN_Mode_Config(); 
  CAN_Filter_Config(); 
  CAN_NVIC_Config();
  return 0;	
}




u8 CAN1_Send_Msg(u16 StdId_620,u8 len,s16 iq1, s16 iq2, s16 iq3, s16 iq4)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=StdId_620;	 // 标准标识符为0
//  TxMessage.ExtId=0x12;	 	// 设置扩展标示符（29位）
  TxMessage.IDE=CAN_ID_STD;		 /*标准标识符*/ 
  TxMessage.RTR=CAN_RTR_DATA;	// 消息类型为数据帧，一帧8位；RTR，远程请求位。0，数据帧；1， 远程帧；
  TxMessage.DLC=len;			// 发送数据帧的长度
  TxMessage.Data[0] = (uint8_t)(iq1>>8);
  TxMessage.Data[1] = (uint8_t) iq1;
  TxMessage.Data[2] = (uint8_t)(iq2>>8);
  TxMessage.Data[3] = (uint8_t) iq2;
  TxMessage.Data[4] = (uint8_t)(iq3>>8);
  TxMessage.Data[5] = (uint8_t) iq3;
  TxMessage.Data[6] = (uint8_t)(iq4>>8);
  TxMessage.Data[7] = (uint8_t) iq4; 
	
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		

}

//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN1_Receive_Msg(uint32_t *RxStdId,u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//查询CAN_FIFO0里是否有数据，没有接收到数据,直接退出该函数 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
	*RxStdId=RxMessage.StdId; //读取接收到的标识符 ！！！
//	printf("\r\nID:%d\r\n",RxMessage.StdId);
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	//返回接收到的数据长度
}



/**************************END OF FILE************************************/

