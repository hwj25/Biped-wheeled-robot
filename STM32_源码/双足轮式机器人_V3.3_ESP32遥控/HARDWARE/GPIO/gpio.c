#include "gpio.h"

void GPIO_In_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); //���������˿�PA��ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //�˿�����Ϊ��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);	//��ʼ���˿�
}


