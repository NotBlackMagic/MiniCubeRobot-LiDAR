#ifndef HAL_SPI_H_
#define HAL_SPI_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"

#define SPI2_BUFFER_LENGTH						64

void SPI1Init();
uint8_t SPI1ReadWrite(uint8_t txByte);
void SPI1Write(uint8_t byte);
uint8_t SPI1Read();

void SPI2Init();
uint8_t SPI2ReadWrite(uint8_t txByte);
uint8_t SPI2Read(uint8_t* data);
uint8_t SPI2Write(uint8_t* data, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* HAL_SPI_H_ */
