#ifndef HAL_ADC_H_
#define HAL_ADC_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"

void ADC1Init();
void ADC2Init();
void ADC1Start();
uint16_t ADC1Read(uint8_t channel);
uint16_t ADC2Read(uint8_t channel);

#ifdef __cplusplus
}
#endif


#endif /* HAL_ADC_H_ */
