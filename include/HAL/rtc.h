#ifndef HAL_RTC_H_
#define HAL_RTC_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_rtc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"

void RTCInit();
void RTCGetTime(uint8_t* hour, uint8_t* min, uint8_t* sec);
void RTCSetTime(uint8_t hour, uint8_t min, uint8_t sec);

#ifdef __cplusplus
}
#endif

#endif /* HAL_RTC_H_ */
