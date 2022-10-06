#include "can.h"



moto_measure_t moto_chassis[8] = {0};//4 chassis moto
moto_measure_t moto_info;


void get_total_angle(moto_measure_t *p);
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);

CAN_HandleTypeDef* _hcan;
CAN_HandleTypeDef  hcan1;


void HAL_CAN_Rx_deal(void)
{
	//ignore can1 or can2.
	switch(_hcan->pRxMsg->StdId){
	case CAN_3508Moto1_ID:
	case CAN_3508Moto2_ID:
	case CAN_3508Moto3_ID:
	case CAN_3508Moto4_ID:
    case CAN_3508Moto5_ID:
    case CAN_3508Moto6_ID:
    case CAN_3508Moto7_ID:
    case CAN_3508Moto8_ID:			
			{
				static u8 i;
				i = _hcan->pRxMsg->StdId - CAN_3508Moto1_ID;
				
				moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], _hcan) : get_moto_measure(&moto_chassis[i], _hcan);
				get_moto_measure(&moto_info, _hcan);
				//get_moto_measure(&moto_chassis[i], _hcan);
			}
			break;
		
		
	}
		
}

//�жϷ�����			    
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	CAN_Receive(CAN1, 0, &RxMessage);
	hcan1.pRxMsg=&RxMessage;
	_hcan=&hcan1;
	HAL_CAN_Rx_deal();
//	printf("\r\n   ID:%x\r\n",_hcan->pRxMsg->StdId);
}



/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    ������̨���,3510���ͨ��CAN����������Ϣ
  * @Param		
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
//	u32  sum=0;
//	u8	 i = FILTER_BUF_LEN;
	
	/*BUG!!! dont use this para code*/
//	ptr->angle_buf[ptr->buf_idx] = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
//	ptr->buf_idx = ptr->buf_idx++ > FILTER_BUF_LEN ? 0 : ptr->buf_idx;
//	while(i){
//		sum += ptr->angle_buf[--i];
//	}
//	ptr->fited_angle = sum / FILTER_BUF_LEN;
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	ptr->real_current  = (int16_t)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
	ptr->speed_rpm = ptr->real_current;	//��������Ϊ���ֵ����Ӧλ��һ������Ϣ
	ptr->given_current = (int16_t)(hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5])/-5;
	ptr->hall = hcan->pRxMsg->Data[6];
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
*@bref ����ϵ�Ƕ�=0�� ֮���������������3510�������Կ�����Ϊ0������ԽǶȡ�
	*/
void get_total_angle(moto_measure_t *p){
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//���ܵ����
		res1 = p->angle + 8192 - p->last_angle;	//��ת��delta=+
		res2 = p->angle - p->last_angle;				//��ת	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//��ת	delta -
		res2 = p->angle - p->last_angle;				//��ת	delta +
	}
	//��������ת���϶���ת�ĽǶ�С���Ǹ������
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}



void send_chassis_cur1_4(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x200;	 // ������?������?��??a0
  //TxMessage.ExtId=0x12;	 	// ����??��??1������?��?�ꡧ29??��?
  TxMessage.IDE=CAN_Id_Standard;		 /*������?������?��?*/ 
  TxMessage.RTR=CAN_RTR_DATA;	// ???�騤��D��?a��y?Y??��?��???8??��?RTR��???3��???��???��0��?��y?Y??��?1��? ??3��??��?
  TxMessage.DLC=0x04;			// ����?����y?Y??��?3��?��
  TxMessage.Data[0] = (uint8_t)(motor1>>8);
  TxMessage.Data[1] = (uint8_t) motor1;
  TxMessage.Data[2] = (uint8_t)(motor2>>8);
  TxMessage.Data[3] = (uint8_t) motor2;
//  TxMessage.Data[4] = (uint8_t)(motor3>>8);
//  TxMessage.Data[5] = (uint8_t) motor3;
//  TxMessage.Data[6] = (uint8_t)(motor4>>8);
//  TxMessage.Data[7] = (uint8_t) motor4; 
	
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�̨���y����?��?����?
//  if(i>=0XFFF)return 1;
//  return 0;	
}


//����?���̡�?����??��?????����?
void send_chassis_cur5_8(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{

  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x1ff;	 // ������?������?��??a0
//  TxMessage.ExtId=0x12;	 	// ����??��??1������?��?�ꡧ29??��?
  TxMessage.IDE=CAN_ID_STD;		 /*������?������?��?*/ 
  TxMessage.RTR=CAN_RTR_DATA;	// ???�騤��D��?a��y?Y??��?��???8??��?RTR��???3��???��???��0��?��y?Y??��?1��? ??3��??��?
  TxMessage.DLC=0x08;			// ����?����y?Y??��?3��?��
  TxMessage.Data[0] = (uint8_t)(motor5>>8);
  TxMessage.Data[1] = (uint8_t) motor5;
  TxMessage.Data[2] = (uint8_t)(motor6>>8);
  TxMessage.Data[3] = (uint8_t) motor6;
  TxMessage.Data[4] = (uint8_t)(motor7>>8);
  TxMessage.Data[5] = (uint8_t) motor7;
  TxMessage.Data[6] = (uint8_t)(motor8>>8);
  TxMessage.Data[7] = (uint8_t) motor8; 
	
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�̨���y����?��?����?
//  if(i>=0XFFF)return 1;
//  return 0;	
}
 



u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{ 
	GPIO_InitTypeDef 		GPIO_InitStructure; 
	CAN_InitTypeDef        	CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure;
#if 1 
	NVIC_InitTypeDef  		NVIC_InitStructure;
#endif

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO

	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE;			//��ʱ�䴥��ͨ��ģʽ  
	CAN_InitStructure.CAN_ABOM=ENABLE;			//����Զ����߹���	 
	CAN_InitStructure.CAN_AWUM=ENABLE;			//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=DISABLE;			//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;		 	//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=DISABLE;			//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= mode;	        //ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; 
	//���ò�����
	CAN_InitStructure.CAN_SJW=tsjw;				//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;        //��Ƶϵ��(Fdiv)Ϊbrp+1	
	CAN_Init(CAN1, &CAN_InitStructure);        	//��ʼ��CAN1 

	CAN_FilterInitStructure.CAN_FilterNumber=0;	//������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//����λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 	//32λ�� 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;	//32λID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0

	CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��
	
#if 1 
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0��Ϣ�Һ��ж�����.		    

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
/**************************END OF FILE************************************/ 
 




















