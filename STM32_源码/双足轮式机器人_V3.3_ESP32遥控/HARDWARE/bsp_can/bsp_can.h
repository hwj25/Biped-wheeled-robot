#ifndef __CAN_H
#define	__CAN_H
#include "stm32f10x.h"
#include "sys.h"


#define CAN_MODE                  		1  //CAN通讯模式选择1：普通模式 0：回环模式
#define CANx                       		CAN1
#define CAN_CLK                    		RCC_APB1Periph_CAN1
#define CAN_RX_IRQ				  	 	USB_LP_CAN1_RX0_IRQn
#define CAN_RX_IRQHandler		   		USB_LP_CAN1_RX0_IRQHandler

#define CAN_RX_PIN                 		GPIO_Pin_11
#define CAN_TX_PIN                 		GPIO_Pin_12
#define CAN_TX_GPIO_PORT         	 	GPIOA
#define CAN_RX_GPIO_PORT          		GPIOA
#define CAN_TX_GPIO_CLK           		RCC_APB2Periph_GPIOA
#define CAN_RX_GPIO_CLK           		RCC_APB2Periph_GPIOA

static void CAN_GPIO_Config(void);
static void CAN_NVIC_Config(void);
static void CAN_Mode_Config(void);
static void CAN_Filter_Config(void);
u8 CAN_Config(void);
void CAN_SetMsg(CanTxMsg *TxMessage);
void Init_RxMes(CanRxMsg *RxMessage);

u8 CAN1_Send_Msg(u16 StdId_620,u8 len,s16 iq1, s16 iq2, s16 iq3, s16 iq4);//发送数据

u8 CAN1_Receive_Msg(uint32_t *RxStdId,u8 *buf);							//接收标准标识符和数据帧


#endif


