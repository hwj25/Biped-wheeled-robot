#include "gpio.h"

void GPIO_In_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); //开启按键端口PA的时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //端口配置为上拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);	//初始化端口
}


