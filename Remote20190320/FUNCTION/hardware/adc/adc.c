
/**
	��adc�ļ�ԭ��RM��׼�����ļ��������ʿ�������adc���ö�ֱ���������޸�

	�������ǵ���ʧ�ܣ�ԭtemperature����ע��
**/


#include "adc.h"
#include "delay.h"
#include "stm32f4xx.h"


#define Rheostat_ADC_IRQ ADC_IRQn
//#define Rheostat_ADC_INT_FUNCTION ADC_IRQHandler


#define RHEOSTAT_ADC_GPIO_PORT GPIOC

#define RHEOSTAT_ADC_GPIO_PIN GPIO_Pin_3

#define RHEOSTAT_ADC_GPIO_CLK RCC_AHB1Periph_GPIOC



#define RHEOSTAT_ADC ADC1

#define RHEOSTAT_ADC_CLK RCC_APB2Periph_ADC1

#define RHEOSTAT_ADC_CHANNEL ADC_Channel_13


int ADC_ConvertedValue = 0;
int COUNTING = 0;


void voltage_ADC_init(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;


			// ʹ�� GPIO ʱ��

					RCC_AHB1PeriphClockCmd(RHEOSTAT_ADC_GPIO_CLK, ENABLE);

			// ���� IO

					GPIO_InitStructure.GPIO_Pin = RHEOSTAT_ADC_GPIO_PIN;

			// ����Ϊģ������

					GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;

			// ������������

					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;

					GPIO_Init(RHEOSTAT_ADC_GPIO_PORT, &GPIO_InitStructure);




    ADC_InitTypeDef 							ADC_InitStructure;

    ADC_CommonInitTypeDef 				ADC_CommonInitStructure;

//����ADCʱ��

    RCC_APB2PeriphClockCmd(RHEOSTAT_ADC_CLK,ENABLE);

//-------------------ADCCommon�ṹ�������ʼ��--------------------

//����ADCģʽ

    ADC_CommonInitStructure.ADC_Mode=ADC_Mode_Independent;

//ʱ��Ϊfpclkx��Ƶ

    ADC_CommonInitStructure.ADC_Prescaler=ADC_Prescaler_Div4;

//��ֹDMAֱ�ӷ���ģʽ

    ADC_CommonInitStructure.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;

//����ʱ����

    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;

    ADC_CommonInit(&ADC_CommonInitStructure);

//-------------------ADCInit�ṹ�������ʼ��---------------------

//ADC�ֱ���

    ADC_InitStructure.ADC_Resolution=ADC_Resolution_12b;

//��ֹɨ��ģʽ����ͨ���ɼ�����Ҫ

    ADC_InitStructure.ADC_ScanConvMode=DISABLE;

//����ת��

    ADC_InitStructure.ADC_ContinuousConvMode=ENABLE;

//��ֹ�ⲿ���ش���

    ADC_InitStructure.ADC_ExternalTrigConvEdge= ADC_ExternalTrigConvEdge_None;

//ʹ������������ⲿ�����������ã�ע�͵�����

//ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T1_CC1;

//�����Ҷ���

    ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;

//ת��ͨ��1��

    ADC_InitStructure.ADC_NbrOfConversion=1;

    ADC_Init(RHEOSTAT_ADC,&ADC_InitStructure);
		
		//ʹ��ADC

		ADC_Cmd(RHEOSTAT_ADC,ENABLE);
//------------------------------------------------------------------


//ADCת�����������жϣ����жϷ�������ж�ȡת��ֵ

    ADC_ITConfig(RHEOSTAT_ADC,ADC_IT_EOC,ENABLE);



    NVIC_InitTypeDef 				NVIC_InitStructure;

    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_InitStructure.NVIC_IRQChannel=Rheostat_ADC_IRQ;

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;

    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;

    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;

    NVIC_Init(&NVIC_InitStructure);
//		
//		
//		
//		
//		DMA_InitTypeDef DMA_InitStructure;
//		DMA_DeInit(DMA2_Stream0);
//		DMA_InitStructure.DMA_BufferSize = 4;  //����λСDMA_InitStructure.DMA_Channel = DMA_Channel_0; //DMAͨ��0

//		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;  //�ⲿ���ݵ��ڴ�

//		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  //��ʹ��FIFO

//		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_ConvertedValue;  //��ȡ���ݵĵ�ַ

//		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;

//		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;

//		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;

//		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;

//		DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_BASE+0x4C;  //ADC->DR��ַ

//		DMA_InitStructure.DMA_PeripheralBurst =DMA_PeripheralBurst_Single;

//		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;

//		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

//		DMA_InitStructure.DMA_Priority = DMA_Priority_High;

//		DMA_Init(DMA2_Stream0,&DMA_InitStructure);

//		DMA_Cmd(DMA2_Stream0,ENABLE);

//		ADC_DMARequestAfterLastTransferCmd(ADC1,ENABLE);//Դ���ݱ仯ʱ����DMA

//		ADC_DMACmd(ADC1,ENABLE);//ʹ��ADC��DMA����
}


void ADC_IRQHandler(void)

{

    if (ADC_GetFlagStatus(RHEOSTAT_ADC,ADC_FLAG_EOC) != RESET)  {

//��ȡADC��ת��ֵ


				ADC_ClearITPendingBit(RHEOSTAT_ADC,ADC_IT_EOC);
				ADC_ConvertedValue=ADC_GetConversionValue(RHEOSTAT_ADC);
				COUNTING++;
    }

}

int get_ADC_ConvertedValue()
{
    return ADC_ConvertedValue;
}


/****static uint16_t get_ADC(uint8_t ch);
static void temperature_ADC_Reset(void);
void temperature_ADC_init(void)
{
    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);

    ADC_TempSensorVrefintCmd(ENABLE);

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInit(&ADC_CommonInitStructure);
    temperature_ADC_Reset();

    ADC_RegularChannelConfig(ADC1, ADC_Channel_18, 1, ADC_SampleTime_15Cycles);
    ADC_Cmd(ADC1, ENABLE);
}

static void temperature_ADC_Reset(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

}

fp32 get_temprate(void)
{
    uint16_t adcx = 0;
    fp32 temperate;
    temperature_ADC_Reset();
    adcx = get_ADC(ADC_Channel_18);
    temperate = (fp32)adcx * (3.3f / 4096.0f);
    temperate = (temperate - 0.76f) / 0.0025f + 25.0f;
    return temperate;
}

static uint16_t get_ADC(uint8_t ch)
{

    ADC_ClearFlag(ADC1,ADC_FLAG_STRT|ADC_FLAG_OVR|ADC_FLAG_EOC);
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_15Cycles);

    ADC_SoftwareStartConv(ADC1);

    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
    {
        ;
    }
    return ADC_GetConversionValue(ADC1);
}
*****/
