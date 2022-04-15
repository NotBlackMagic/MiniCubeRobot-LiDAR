#include "gpio.h"

#include "pinMaping.h"

/**
  * @brief	This function initializes the GPIO that don't use a peripheral (ADC/UART/SPI etc)
  * @param	None
  * @return	None
  */
void GPIOInit() {
	//Enable Port Clocks
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);

	//Enable EXTI/AFIO Clocks and Power
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	//Enable PC14 and PC15 as general IO: JTAG-DP Disabled and SW-DP Enabled
	LL_GPIO_AF_Remap_SWJ_NOJTAG();
//	while((AFIO->MAPR & AFIO_MAPR_SWJ_CFG_JTAGDISABLE) != AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

	//Set ADC Input GPIO's
	GPIOSetPinMode(GPIO_ADC_I_VCC, GPIO_Mode_Analog);
	GPIOSetPinMode(GPIO_ADC_I_VCC_S, GPIO_Mode_Analog);

	//Set Input GPIO's
	GPIOSetPinMode(GPIO_IN_INT_FL, GPIO_Mode_Input);
	GPIOSetPinMode(GPIO_IN_INT_FC, GPIO_Mode_Input);
	GPIOSetPinMode(GPIO_IN_INT_FR, GPIO_Mode_Input);
	GPIOSetPinMode(GPIO_IN_INT_BR, GPIO_Mode_Input);
	GPIOSetPinMode(GPIO_IN_INT_BC, GPIO_Mode_Input);
	GPIOSetPinMode(GPIO_IN_INT_BL, GPIO_Mode_Input);

	//Set Input Pins Interrupts
	//Enable Interrupt Sensor Front Left (PB8)
	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE8);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_8);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_8);
//	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_8);
	//Enable Interrupt Sensor Front Center (PB3)
	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE3);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_3);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_3);
//	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_3);
	//Enable Interrupt Sensor Front Right (PC10)
	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTC, LL_GPIO_AF_EXTI_LINE10);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_10);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_10);
//	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_10);
	//Enable Interrupt Sensor Back Right (PC7)
	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTC, LL_GPIO_AF_EXTI_LINE7);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_7);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_7);
//	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_7);
	//Enable Interrupt Sensor Back Center (PC5)
	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTC, LL_GPIO_AF_EXTI_LINE5);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_5);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_5);
//	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_5);
	//Enable Interrupt Sensor Back Left (PC2)
	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTC, LL_GPIO_AF_EXTI_LINE2);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_2);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_2);
//	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_2);

	//Set Output GPIO's
	GPIOSetPinMode(GPIO_OUT_LP_FL, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_LP_FC, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_LP_FR, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_LP_BR, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_LP_BC, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_LP_BL, GPIO_Mode_Output);

	GPIOSetPinMode(GPIO_OUT_RST_FL, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_RST_FC, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_RST_FR, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_RST_BR, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_RST_BC, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_RST_BL, GPIO_Mode_Output);

	GPIOSetPinMode(GPIO_OUT_IO0, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_IO1, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_IO2, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_IO3, GPIO_Mode_Output);

	GPIOSetPinMode(GPIO_OUT_LED0, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_LED1, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_LED2, GPIO_Mode_Output);

	GPIOSetPinMode(GPIO_EN_VCC_S, GPIO_Mode_Output);

	//Set Outputs of GPIOs
	GPIOWrite(GPIO_OUT_LP_FL, 0x00);		//Disable I2C comms of LIDAR FL
	GPIOWrite(GPIO_OUT_LP_FC, 0x00);		//Disable I2C comms of LIDAR FC
	GPIOWrite(GPIO_OUT_LP_FR, 0x00);		//Disable I2C comms of LIDAR FR
	GPIOWrite(GPIO_OUT_LP_BR, 0x00);		//Disable I2C comms of LIDAR BR
	GPIOWrite(GPIO_OUT_LP_BC, 0x00);		//Disable I2C comms of LIDAR BC
	GPIOWrite(GPIO_OUT_LP_BL, 0x00);		//Disable I2C comms of LIDAR BL

	GPIOWrite(GPIO_OUT_RST_FL, 0x00);		//No reset of I2C comms interface of LIDAR FL
	GPIOWrite(GPIO_OUT_RST_FC, 0x00);		//No reset of I2C comms interface of LIDAR FC
	GPIOWrite(GPIO_OUT_RST_FR, 0x00);		//No reset of I2C comms interface of LIDAR FR
	GPIOWrite(GPIO_OUT_RST_BR, 0x00);		//No reset of I2C comms interface of LIDAR BR
	GPIOWrite(GPIO_OUT_RST_BC, 0x00);		//No reset of I2C comms interface of LIDAR BC
	GPIOWrite(GPIO_OUT_RST_BL, 0x00);		//No reset of I2C comms interface of LIDAR BL

	GPIOWrite(GPIO_OUT_IO0, 0x00);			//Set backbone IO0 low
	GPIOWrite(GPIO_OUT_IO1, 0x00);			//Set backbone IO1 low
	GPIOWrite(GPIO_OUT_IO2, 0x00);			//Set backbone IO2 low
	GPIOWrite(GPIO_OUT_IO3, 0x00);			//Set backbone IO3 low

	GPIOWrite(GPIO_OUT_LED0, 0x00);			//Disable LED0
	GPIOWrite(GPIO_OUT_LED1, 0x00);			//Disable LED1
	GPIOWrite(GPIO_OUT_LED2, 0x00);			//Disable LED2

	GPIOWrite(GPIO_EN_VCC_S, 0x00);			//Disable LDO for LIDAR Sensors

	//Enable EXTI Interrupts
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_SetPriority(EXTI2_IRQn, 0);
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_SetPriority(EXTI3_IRQn, 0);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_SetPriority(EXTI9_5_IRQn, 0);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_SetPriority(EXTI15_10_IRQn, 0);
}

/**
  * @brief	This function sets the output mode type of a pin
  * @param	gpio: Pin to define output mode
  * @param	mode: Output mode of this pin
  * @return	None
  */
void GPIOSetPinMode(uint8_t gpio, GPIOOutputMode mode) {
	uint8_t port = (gpio >> 4);
	uint8_t pin = gpio & 0x0F;
	LL_GPIO_SetPinMode(gpioPorts[port], gpioPins[pin], gpioOutputMode[mode]);
}

/**
  * @brief	This function sets the output of a pin
  * @param	gpio: Pin to set output
  * @param	on: 1 output is set high, 0 output is set low
  * @return	None
  */
void GPIOWrite(uint8_t gpio, uint8_t on) {
	uint8_t port = (gpio >> 4);
	uint8_t pin = gpio & 0x0F;
	if(on == 1) {
		LL_GPIO_SetOutputPin(gpioPorts[port], gpioPins[pin]);
	}
	else {
		LL_GPIO_ResetOutputPin(gpioPorts[port], gpioPins[pin]);
	}
}

/**
  * @brief	This function gets the input state of a pin
  * @param	gpio: Pin to set output
  * @return	1 input is set high, 0 input is set low
  */
uint8_t GPIORead(uint8_t gpio) {
	uint8_t port = (gpio >> 4);
	uint8_t pin = gpio & 0x0F;
	return LL_GPIO_IsInputPinSet(gpioPorts[port], gpioPins[pin]);
}

__attribute__((weak)) void EXTI0Callback() {}

/**
  * @brief	This function is the Handler for GPIO0s
  * @param	None
  * @return	None
  */
void EXTI0_IRQHandler(void) {
	//EXTI0 Triggered, call callback function
	EXTI0Callback();

	//Clear Interrupt Flag
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
}

__attribute__((weak)) void EXTI1Callback() {}

/**
  * @brief	This function is the Handler for GPIO1s
  * @param	None
  * @return	None
  */
void EXTI1_IRQHandler(void) {
	//EXTI1 Triggered, call callback function
	EXTI1Callback();

	//Clear Interrupt Flag
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
}

__attribute__((weak)) void EXTI2Callback() {}

/**
  * @brief	This function is the Handler for GPIO2s
  * @param	None
  * @return	None
  */
void EXTI2_IRQHandler(void) {
	//EXTI2 Triggered, call callback function
	EXTI2Callback();

	//Clear Interrupt Flag
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
}

__attribute__((weak)) void EXTI3Callback() {}

/**
  * @brief	This function is the Handler for GPIO3s
  * @param	None
  * @return	None
  */
void EXTI3_IRQHandler(void) {
	//EXTI3 Triggered, call callback function
	EXTI3Callback();

	//Clear Interrupt Flag
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);
}

__attribute__((weak)) void EXTI4Callback() {}

/**
  * @brief	This function is the Handler for GPIO4s
  * @param	None
  * @return	None
  */
void EXTI4_IRQHandler(void) {
	//EXTI4 Triggered, call callback function
	EXTI4Callback();

	//Clear Interrupt Flag
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
}

__attribute__((weak)) void EXTI5Callback() {}
__attribute__((weak)) void EXTI6Callback() {}
__attribute__((weak)) void EXTI7Callback() {}
__attribute__((weak)) void EXTI8Callback() {}
__attribute__((weak)) void EXTI9Callback() {}

/**
  * @brief	This function is the Handler for GPIO5s to GPIO9s
  * @param	None
  * @return	None
  */
void EXTI9_5_IRQHandler(void) {
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_5) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_5) == 0x01) {
		//EXTI5 Triggered, call callback function
		EXTI5Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_6) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_6) == 0x01) {
		//EXTI6 Triggered, call callback function
		EXTI6Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_7) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_7) == 0x01) {
		//EXTI7 Triggered, call callback function
		EXTI7Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_8) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8) == 0x01) {
		//EXTI8 Triggered, call callback function
		EXTI8Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_9) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9) == 0x01) {
		//EXTI9 Triggered, call callback function
		EXTI9Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);
	}
}

__attribute__((weak)) void EXTI10Callback() {}
__attribute__((weak)) void EXTI11Callback() {}
__attribute__((weak)) void EXTI12Callback() {}
__attribute__((weak)) void EXTI13Callback() {}
__attribute__((weak)) void EXTI14Callback() {}
__attribute__((weak)) void EXTI15Callback() {}

/**
  * @brief	This function is the Handler for GPIO10s to GPIO15s
  * @param	None
  * @return	None
  */
void EXTI15_10_IRQHandler(void) {
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_10) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10) == 0x01) {
		//EXTI10 Triggered, call callback function
		EXTI10Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_11) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_11) == 0x01) {
		//EXTI11 Triggered, call callback function
		EXTI11Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_11);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_12) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12) == 0x01) {
		//EXTI12 Triggered, call callback function
		EXTI12Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_13) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13) == 0x01) {
		//EXTI13 Triggered, call callback function
		EXTI13Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_14) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_14) == 0x01) {
		//EXTI14 Triggered, call callback function
		EXTI14Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_14);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_15) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_15) == 0x01) {
		//EXTI15 Triggered, call callback function
		EXTI15Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
	}
}
