#ifndef ADC_H
#define ADC_H
/**
	��adc�ļ�ԭ��RM��׼�����ļ��������ʿ�������adc���ö�ֱ���������޸�
	
	�������ǵ���ʧ�ܣ�ԭtemperature����ע��
**/
#include "main.h"

/****extern void temperature_ADC_init(void);
extern fp32 get_temprate(void);****/

void voltage_ADC_init(void);
int get_ADC_ConvertedValue(void);

#endif