#include "adc.h"

 
u16 AD_After_Filter[Channel_Num]; 
 
u8 Electric_Quantity_12, Electric_Quantity_24, Air_Quality;


 
void Adc_Init(void)
{
    GPIO_InitTypeDef      GPIO_InitStructure;                             //定义一个设置GPIO的变量
    ADC_InitTypeDef       ADC_InitStructure;                              //定义一个设置ADC的变量

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );	              //使能GPIOA通道时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );	              //使能GPIOB通道时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE );	              //使能ADC1通道时钟

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);                                      //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4;                   //准备设置PA6 7
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		                   //模拟输入引脚
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                 //设置PA6 7

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;                              //准备设置PB0
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		                   //模拟输入引脚
//    GPIO_Init(GPIOB, &GPIO_InitStructure);                                 //设置PB0

    ADC_DeInit(ADC1);                                                      //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	                   //ADC工作模式:ADC1和ADC2工作在独立模式
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;	                       //模数转换工作在单通道模式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	                   //模数转换工作在单次转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	   //转换由软件而不是外部触发启动
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	               //ADC数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 3;	                               //顺序进行规则转换的ADC通道的数目
    ADC_Init(ADC1, &ADC_InitStructure);                                    //根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器



    ADC_Cmd(ADC1, ENABLE);	                                               //使能指定的ADC1
    ADC_ResetCalibration(ADC1);	                                           //使能复位校准
    while(ADC_GetResetCalibrationStatus(ADC1));                     	   //等待复位校准结束
    ADC_StartCalibration(ADC1);	                                           //开启AD校准
    while(ADC_GetCalibrationStatus(ADC1));	                               //等待校准结束


}

/*-------------------------------------------------*/
/*函数名：获得ADC结果                              */
/*参  数：ch: 通道数                               */
/*返回值：ADC的结果                                */
/*-------------------------------------------------*/
int Get_Adc(int ch)
{
	ADC_RegularChannelConfig(ADC1, ch, 0, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ch, 2, ADC_SampleTime_239Cycles5 );
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);		                        //使能指定的ADC1的软件转换启动功能
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));                     //等待转换结束
    return ADC_GetConversionValue(ADC1);	                            //返回最近一次ADC1规则组的转换结果
}


/*-------------------------------------------------*/
/*函数名：平均多次ADC结果, 读取空气质量传感器模拟量   */
/*参  数：ch: 通道数                               */
/*参  数：times: 平均次数                          */
/*返回值：平均后的结果                             */
/*-------------------------------------------------*/
void Get_Air_Quality_Average(u8 ch,u8 times)
{
    int temp_val=0;
    char t;

    for(t=0; t<times; t++)           //循环读取times次
    {
        temp_val+=Get_Adc(ch);       //计算总值
        vTaskDelay(5);                 //延时
    }
	AD_After_Filter[0] =  temp_val/times;
	//printf("AD_After_Filter[0]=%d\r\n",AD_After_Filter[0]);
	Air_Quality = (u8)(((float)AD_After_Filter[0]/4096.0f)*100);

    return;           //返回平均值
}


/*-------------------------------------------------*/
/*函数名：平均多次ADC结果，读取12V电池电压           */
/*参  数：ch: 通道数                               */
/*参  数：times: 平均次数                          */
/*返回值：平均后的结果                             */
/*-------------------------------------------------*/
void Get_Electric_Quantity_12_Average(u8 ch,u8 times)
{
    int temp_val=0;
    char t;

    for(t=0; t<times; t++)           //循环读取times次
    {
        temp_val+=Get_Adc(ch);       //计算总值
        vTaskDelay(5);                 //延时
    }
	AD_After_Filter[2] =  temp_val/times-30;
	printf("AD_After_Filter=%d\r\n",AD_After_Filter[2]);
	if(AD_After_Filter[2]>2514)
		Electric_Quantity_12 = 100;
	else if(AD_After_Filter[2]<2215)
		Electric_Quantity_12 = 0;
	else
		Electric_Quantity_12 = (((float)(AD_After_Filter[2]-2215)/299.0)*100);

    return;           //返回平均值
}

/*-------------------------------------------------*/
/*函数名：平均多次ADC结果，读取24V电池电压           */
/*参  数：ch: 通道数                               */
/*参  数：times: 平均次数                          */
/*返回值：平均后的结果                             */
/*-------------------------------------------------*/

void Get_Electric_Quantity_24_Average(u8 ch,u8 times)
{
    int temp_val=0;
    char t;

    for(t=0; t<times; t++)           //循环读取times次
    {
        temp_val+=Get_Adc(ch);       //计算总值
        vTaskDelay(5);                 //延时
    }
	AD_After_Filter[1] =  temp_val/times +220;
	//printf("AD_After_Filter[1]=%d\r\n",AD_After_Filter[1]);
	if(AD_After_Filter[1]>2803)
		Electric_Quantity_24 = 100;
	else if(AD_After_Filter[1]<2470)
		Electric_Quantity_24 = 0;
	else
		Electric_Quantity_24 = ((float)(AD_After_Filter[1]-2470)/333.0f)*100;

    return;           //返回平均值
}


 
 