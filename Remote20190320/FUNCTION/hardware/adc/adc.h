#ifndef ADC_H
#define ADC_H
/**
	此adc文件原是RM标准库下文件，现因功率控制增加adc配置而直接在上面修改
	
	因陀螺仪调试失败，原temperature代码注释
**/
#include "main.h"

/****extern void temperature_ADC_init(void);
extern fp32 get_temprate(void);****/

void voltage_ADC_init(void);
int get_ADC_ConvertedValue(void);

#endif
