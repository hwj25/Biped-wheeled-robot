#include "adc.h"

 
u16 AD_After_Filter[Channel_Num]; 
 
u8 Electric_Quantity_12, Electric_Quantity_24, Air_Quality;


 
void Adc_Init(void)
{
    GPIO_InitTypeDef      GPIO_InitStructure;                             //����һ������GPIO�ı���
    ADC_InitTypeDef       ADC_InitStructure;                              //����һ������ADC�ı���

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );	              //ʹ��GPIOAͨ��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );	              //ʹ��GPIOBͨ��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE );	              //ʹ��ADC1ͨ��ʱ��

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);                                      //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4;                   //׼������PA6 7
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		                   //ģ����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                 //����PA6 7

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;                              //׼������PB0
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		                   //ģ����������
//    GPIO_Init(GPIOB, &GPIO_InitStructure);                                 //����PB0

    ADC_DeInit(ADC1);                                                      //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	                   //ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;	                       //ģ��ת�������ڵ�ͨ��ģʽ
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	                   //ģ��ת�������ڵ���ת��ģʽ
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	   //ת��������������ⲿ��������
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	               //ADC�����Ҷ���
    ADC_InitStructure.ADC_NbrOfChannel = 3;	                               //˳����й���ת����ADCͨ������Ŀ
    ADC_Init(ADC1, &ADC_InitStructure);                                    //����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���



    ADC_Cmd(ADC1, ENABLE);	                                               //ʹ��ָ����ADC1
    ADC_ResetCalibration(ADC1);	                                           //ʹ�ܸ�λУ׼
    while(ADC_GetResetCalibrationStatus(ADC1));                     	   //�ȴ���λУ׼����
    ADC_StartCalibration(ADC1);	                                           //����ADУ׼
    while(ADC_GetCalibrationStatus(ADC1));	                               //�ȴ�У׼����


}

/*-------------------------------------------------*/
/*�����������ADC���                              */
/*��  ����ch: ͨ����                               */
/*����ֵ��ADC�Ľ��                                */
/*-------------------------------------------------*/
int Get_Adc(int ch)
{
	ADC_RegularChannelConfig(ADC1, ch, 0, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ch, 2, ADC_SampleTime_239Cycles5 );
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);		                        //ʹ��ָ����ADC1�����ת����������
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));                     //�ȴ�ת������
    return ADC_GetConversionValue(ADC1);	                            //�������һ��ADC1�������ת�����
}


/*-------------------------------------------------*/
/*��������ƽ�����ADC���, ��ȡ��������������ģ����   */
/*��  ����ch: ͨ����                               */
/*��  ����times: ƽ������                          */
/*����ֵ��ƽ����Ľ��                             */
/*-------------------------------------------------*/
void Get_Air_Quality_Average(u8 ch,u8 times)
{
    int temp_val=0;
    char t;

    for(t=0; t<times; t++)           //ѭ����ȡtimes��
    {
        temp_val+=Get_Adc(ch);       //������ֵ
        vTaskDelay(5);                 //��ʱ
    }
	AD_After_Filter[0] =  temp_val/times;
	//printf("AD_After_Filter[0]=%d\r\n",AD_After_Filter[0]);
	Air_Quality = (u8)(((float)AD_After_Filter[0]/4096.0f)*100);

    return;           //����ƽ��ֵ
}


/*-------------------------------------------------*/
/*��������ƽ�����ADC�������ȡ12V��ص�ѹ           */
/*��  ����ch: ͨ����                               */
/*��  ����times: ƽ������                          */
/*����ֵ��ƽ����Ľ��                             */
/*-------------------------------------------------*/
void Get_Electric_Quantity_12_Average(u8 ch,u8 times)
{
    int temp_val=0;
    char t;

    for(t=0; t<times; t++)           //ѭ����ȡtimes��
    {
        temp_val+=Get_Adc(ch);       //������ֵ
        vTaskDelay(5);                 //��ʱ
    }
	AD_After_Filter[2] =  temp_val/times-30;
	printf("AD_After_Filter=%d\r\n",AD_After_Filter[2]);
	if(AD_After_Filter[2]>2514)
		Electric_Quantity_12 = 100;
	else if(AD_After_Filter[2]<2215)
		Electric_Quantity_12 = 0;
	else
		Electric_Quantity_12 = (((float)(AD_After_Filter[2]-2215)/299.0)*100);

    return;           //����ƽ��ֵ
}

/*-------------------------------------------------*/
/*��������ƽ�����ADC�������ȡ24V��ص�ѹ           */
/*��  ����ch: ͨ����                               */
/*��  ����times: ƽ������                          */
/*����ֵ��ƽ����Ľ��                             */
/*-------------------------------------------------*/

void Get_Electric_Quantity_24_Average(u8 ch,u8 times)
{
    int temp_val=0;
    char t;

    for(t=0; t<times; t++)           //ѭ����ȡtimes��
    {
        temp_val+=Get_Adc(ch);       //������ֵ
        vTaskDelay(5);                 //��ʱ
    }
	AD_After_Filter[1] =  temp_val/times +220;
	//printf("AD_After_Filter[1]=%d\r\n",AD_After_Filter[1]);
	if(AD_After_Filter[1]>2803)
		Electric_Quantity_24 = 100;
	else if(AD_After_Filter[1]<2470)
		Electric_Quantity_24 = 0;
	else
		Electric_Quantity_24 = ((float)(AD_After_Filter[1]-2470)/333.0f)*100;

    return;           //����ƽ��ֵ
}


 
 