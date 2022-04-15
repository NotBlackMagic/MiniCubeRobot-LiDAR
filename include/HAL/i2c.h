#ifndef I2C_H_
#define I2C_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_rcc.h"

typedef enum {
	I2CMode_SM,		//I2C Standard Mode (100kHz)
	I2CMode_FM,		//I2C Fast Mode (400kHz)
	I2CMode_FMP,	//I2C Fast Mode Plus (1MHz)
	I2CMode_HSM,	//I2C High Speed Mode (3.4MHz)
	I2CMode_UFM		//I2C Ultra Fast Mode (5MHz)
} I2CMode;

void I2C1Init(I2CMode mode);
void I2C2Init(I2CMode mode);
uint8_t I2C1Write(uint8_t address, uint8_t* data, uint16_t length);
uint8_t I2C2Write(uint8_t address, uint8_t* data, uint16_t length);
uint8_t I2C1Read(uint8_t address, uint8_t data[], uint16_t length);
uint8_t I2C2Read(uint8_t address, uint8_t data[], uint16_t length);
uint8_t I2C1WriteRegister(uint8_t address, uint16_t regsiter, uint8_t* data, uint16_t length);
uint8_t I2C2WriteRegister(uint8_t address, uint16_t regsiter, uint8_t* data, uint16_t length);

void I2C1StartRead(uint8_t address, uint8_t reg, uint8_t length);
uint8_t I2C1ReadBuffer(uint8_t data[]);

#ifdef __cplusplus
}
#endif

#endif /* I2C_H_ */
