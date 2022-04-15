#include "uart.h"

#define UART_WRITE_TIMEOUT									1000	//Timeout to wait for TX Buffer empty in ms

#define UART_RX_BUFFER_SIZE									256
#define UART_TX_BUFFER_SIZE									2048

uint16_t uart1RXCRC;
uint16_t uart1RXBufferIndex;
uint16_t uart1RXBufferLength;
uint8_t uart1RXBuffer[UART_RX_BUFFER_SIZE];
uint16_t uart1TXBufferIndex;
uint16_t uart1TXBufferLength;
uint8_t uart1TXBuffer[UART_TX_BUFFER_SIZE];

uint16_t uart2RXCRC;
uint16_t uart2RXBufferIndex;
uint16_t uart2RXBufferLength;
uint8_t uart2RXBuffer[UART_RX_BUFFER_SIZE];

uint8_t uart2TXEmptyBuffer = 3;		//Buffer empty (available): 0: None; 1: buffer[0]; 2: buffer[1]; 3: Both
uint8_t uart2TXActiveBuffer = 0;	//Buffer used in TX: 0: None; 1: buffer[0]; 2: buffer[1]
uint16_t uart2TXBufferIndex = 0;
uint16_t uart2TXBufferLength[2];
uint8_t uart2TXBuffer[2][UART_TX_BUFFER_SIZE];

/**
  * @brief	This function initializes the UART1 interface, also sets the respective GPIO pins
  * @param	baud: UART Baud rate to use
  * @return	None
  */
void UART1Init(uint32_t baud) {
	//Enable bus clocks
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);

	//Configure GPIOs
//	LL_GPIO_AF_EnableRemap_USART1();	//Remap UART1 GPIOs to Alternative GPIOs
	//Set UART1 TX (PA9) as AF push-pull
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
	//Set UART1 RX (PA10) as input floating
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_FLOATING);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

	//Configure UART Interface
	LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
	LL_USART_ConfigCharacter(USART1, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
	LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);
	LL_USART_SetBaudRate(USART1, SystemCoreClock, baud);

	//Configure UART Interrupts
	NVIC_SetPriority(USART1_IRQn, 0);
	NVIC_EnableIRQ(USART1_IRQn);
	LL_USART_EnableIT_RXNE(USART1);
	LL_USART_EnableIT_ERROR(USART1);
//	LL_USART_EnableIT_IDLE(USART1);
//	LL_USART_EnableIT_TXE(USART1);

	//Enable UART
	LL_USART_Enable(USART1);
}

/**
  * @brief	This function initializes the UART2 interface, also sets the respective GPIO pins
  * @param	baud: UART Baud rate to use
  * @return	None
  */
void UART2Init(uint32_t baud) {
	//Enable bus clocks
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);

	//Configure GPIOs
//	LL_GPIO_AF_EnableRemap_USART2();	//Remap UART2 GPIOs to Alternative GPIOs
	//Set UART2 TX (PA2) as AF push-pull
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);
	//Set UART2 RX (PA3) as input floating
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_FLOATING);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_UP);

	//Configure UART Interface
	LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX_RX);
	LL_USART_ConfigCharacter(USART2, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
	LL_USART_SetHWFlowCtrl(USART2, LL_USART_HWCONTROL_NONE);
	LL_USART_SetBaudRate(USART2, (SystemCoreClock / 2), baud);

	//Configure UART Interrupts
	NVIC_SetPriority(USART2_IRQn, 0);
	NVIC_EnableIRQ(USART2_IRQn);
	LL_USART_EnableIT_RXNE(USART2);
	LL_USART_EnableIT_ERROR(USART2);
//	LL_USART_EnableIT_IDLE(USART2);
//	LL_USART_EnableIT_TXE(USART2);

	//Enable UART
	LL_USART_Enable(USART2);
}

/**
  * @brief	This function sets the UART1 Baudrate
  * @param	baudrate: The baudrate to use/set
  * @return	None
  */
void UART1SetBaudrate(uint32_t baudrate) {
	LL_USART_SetBaudRate(USART1, SystemCoreClock, baudrate);
}

/**
  * @brief	This function sets the UART2 Baudrate
  * @param	baudrate: The baudrate to use/set
  * @return	None
  */
void UART2SetBaudrate(uint32_t baudrate) {
	LL_USART_SetBaudRate(USART2, SystemCoreClock, baudrate);
}

/**
  * @brief	This function sends data over UART1
  * @param	data: data array to transmit
  * @param	length: length of the transmit data array
  * @return	0 -> if all good, no errors; 1 -> TX Buffer is full
  */
uint8_t UART1Write(uint8_t* data, uint16_t length) {
	//Wait for buffer empty, if is not
	uint32_t timestamp = GetSysTick();
	while(uart1TXBufferLength != 0x00 && (timestamp + UART_WRITE_TIMEOUT) > GetSysTick());

	if(uart1TXBufferLength == 0x00) {
		//**********************//
		//		UART Frame		//
		//**********************//
		//|  u16   | n*u8 | u16 |
		//|--------|------|-----|
		//| Length | Data | CRC |
		uint16_t index = 0;

		//Add length field
		uart1TXBuffer[index++] = (uint8_t)((length >> 8) & 0xFF);
		uart1TXBuffer[index++] = (uint8_t)((length) & 0xFF);

		//Add payload/data
		memcpy(&uart1TXBuffer[index], data, length);
		index = index + length;

		//Calculate and add CRC
		uint16_t crc;
		CRCReset(&crc);
		uint16_t i;
		for(i = 0; i < (length + 2); i++) {
			CRCWrite(uart1TXBuffer[i], &crc);
		}
		crc = CRCRead(&crc);
		uart1TXBuffer[index++] = (uint8_t)((crc >> 8) & 0xFF);
		uart1TXBuffer[index++] = (uint8_t)((crc) & 0xFF);

		uart1TXBufferLength = index;
		uart1TXBufferIndex = 0;

		LL_USART_TransmitData8(USART1, uart1TXBuffer[uart1TXBufferIndex++]);

		LL_USART_EnableIT_TXE(USART1);
	}
	else {
		//Buffer full, return error
		return 1;
	}

	return 0;
}

/**
  * @brief	This function sends data over UART2
  * @param	data: data array to transmit
  * @param	length: length of the transmit data array
  * @return	0 -> if all good, no errors; 1 -> TX Buffer is full
  */
uint8_t UART2Write(uint8_t* data, uint16_t length) {
	//Wait for buffer empty, if is not
	uint32_t timestamp = GetSysTick();
	while(uart2TXEmptyBuffer == 0x00 && (timestamp + UART_WRITE_TIMEOUT) > GetSysTick());

	uint8_t bufferSelection = 0;
	if(uart2TXEmptyBuffer == 0x01) {
		//Only buffer 0 empty
		bufferSelection = 0;
		uart2TXEmptyBuffer = 0x00;
	}
	else if(uart2TXEmptyBuffer == 0x02) {
		//Only buffer 1 empty
		bufferSelection = 1;
		uart2TXEmptyBuffer = 0x00;
	}
	else if(uart2TXEmptyBuffer == 0x03) {
		//Both buffer empty (use 0)
		bufferSelection = 0;
		uart2TXActiveBuffer = 0x01;
		uart2TXEmptyBuffer = 0x02;
	}
	else {
		//Buffers full, return error
		return 1;
	}

	//**********************//
	//		UART Frame		//
	//**********************//
	//|  u16   | n*u8 | u16 |
	//|--------|------|-----|
	//| Length | Data | CRC |
	uint16_t index = 0;

	//Add length field
	uart2TXBuffer[bufferSelection][index++] = (uint8_t)((length >> 8) & 0xFF);
	uart2TXBuffer[bufferSelection][index++] = (uint8_t)((length) & 0xFF);

	//Add payload/data
	memcpy(&uart2TXBuffer[bufferSelection][index], data, length);
	index = index + length;

	//Calculate and add CRC
	uint16_t crc;
	CRCReset(&crc);
	uint16_t i;
	for(i = 0; i < (length + 2); i++) {
		CRCWrite(uart2TXBuffer[bufferSelection][i], &crc);
	}
	crc = CRCRead(&crc);
	uart2TXBuffer[bufferSelection][index++] = (uint8_t)((crc >> 8) & 0xFF);
	uart2TXBuffer[bufferSelection][index++] = (uint8_t)((crc) & 0xFF);

	uart2TXBufferLength[bufferSelection] = index;

	if(uart2TXEmptyBuffer == 0x02) {
		//Both buffer where empty so TX has to be initiated again
		LL_USART_TransmitData8(USART2, uart2TXBuffer[0][uart2TXBufferIndex++]);

		LL_USART_EnableIT_TXE(USART2);
	}

	return 0;
}

/**
  * @brief	This function reads the UART1 RX buffer
  * @param	data: data array to where the received data should be copied to
  * @param	length: length of the received array, is set in this function
  * @return	Returns 0 if no new data, 1 if new data
  */
uint8_t UART1Read(uint8_t* data, uint16_t* length) {
	if(uart1RXBufferLength > 0) {
		uint16_t i;
		for(i = 0; i < (uart1RXBufferLength - 4); i++) {
			data[i] = uart1RXBuffer[2 + i];
		}

		*length = (uart1RXBufferLength - 4);

		uart1RXBufferLength = 0;
		uart1RXBufferIndex = 0;

		return 1;
	}
	else {
		return 0;
	}
}

/**
  * @brief	This function reads the UART2 RX buffer
  * @param	data: data array to where the received data should be copied to
  * @param	length: length of the received array, is set in this function
  * @return	Returns 0 if no new data, 1 if new data
  */
uint8_t UART2Read(uint8_t* data, uint16_t* length) {
	if(uart2RXBufferLength > 0) {
		uint16_t i;
		for(i = 0; i < (uart2RXBufferLength - 4); i++) {
			data[i] = uart2RXBuffer[2 + i];
		}

		*length = (uart2RXBufferLength - 4);

		uart2RXBufferLength = 0;
		uart2RXBufferIndex = 0;

		return 1;
	}
	else {
		return 0;
	}
}


/**
  * @brief	UART1 IRQ Handler
  * @param	None
  * @return	None
  */
void USART1_IRQHandler(void) {
	//**********************//
	//		UART Frame		//
	//**********************//
	//|  u16   | n*u8 | u16 |
	//|--------|------|-----|
	//| Length | Data | CRC |
	if(LL_USART_IsEnabledIT_TXE(USART1) == 0x01 && LL_USART_IsActiveFlag_TXE(USART1) == 0x01) {
		if(uart1TXBufferIndex >= uart1TXBufferLength) {
			//Transmission complete
			uart1TXBufferLength = 0;
			uart1TXBufferIndex = 0;

			//Disable TX done interrupt
			LL_USART_DisableIT_TXE(USART1);
		}
		else {
			//Transmit another byte
			LL_USART_TransmitData8(USART1, uart1TXBuffer[uart1TXBufferIndex++]);
		}
	}

	if(LL_USART_IsActiveFlag_RXNE(USART1) == 0x01) {
		if(uart1RXBufferLength != 0x00) {
			//RX Buffer full, has a complete frame in it
			uint8_t dummy = LL_USART_ReceiveData8(USART1);

			uart1RXBufferIndex = 0;
		}
		else if(uart1RXBufferIndex >= UART_RX_BUFFER_SIZE) {
			//RX Buffer overflow
			uint8_t dummy = LL_USART_ReceiveData8(USART1);

			uart1RXBufferIndex = 0;
		}
		else {
			//All good, read received byte to RX buffer
			uint8_t rxByte = LL_USART_ReceiveData8(USART1);
			uart1RXBuffer[uart1RXBufferIndex++] = rxByte;

			//Payload length byte received
			uint16_t payloadLength = (uart1RXBuffer[0] << 8) + uart1RXBuffer[1];

			//Update CRC
			if(uart1RXBufferIndex == 1) {
				CRCReset(&uart1RXCRC);
				CRCWrite(rxByte, &uart1RXCRC);
			}
			else if(uart1RXBufferIndex <= (payloadLength + 2)) {
				CRCWrite(rxByte, &uart1RXCRC);
			}

			//End of transmission detection based on UART frame length, in bytes 0 and 1
			if(uart1RXBufferIndex >= 2) {
				//Check for end of packet
				if(payloadLength < 1) {
					//Invalid payload length
					uart1RXBufferIndex = 0;
				}
				else if(payloadLength > UART_RX_BUFFER_SIZE) {
					//Buffer overflow
					uart1RXBufferIndex = 0;
				}
				else if(uart1RXBufferIndex >= (payloadLength + 4)) {
					//Full packet received, get CRC from package (last two bytes)
					//CRC Check
					uint16_t crc = (uart1RXBuffer[uart1RXBufferIndex - 2] << 8) + uart1RXBuffer[uart1RXBufferIndex - 1];
					if (crc != CRCRead(&uart1RXCRC)) {
						//Packet CRC failed
						uart1RXBufferIndex = 0;
					}
					else {
						uart1RXBufferLength = uart1RXBufferIndex;
					}
				}
			}
		}
	}

	if(LL_USART_IsActiveFlag_IDLE(USART1) == 0x01) {
		//End of frame transmission, detected by receiver timeout
//		uart1RXBufferLength = uart1RXBufferIndex;

		//Clear IDLE Flag
		LL_USART_ClearFlag_IDLE(USART1);
	}

	if(LL_USART_IsActiveFlag_FE(USART1) == 0x01 || LL_USART_IsActiveFlag_NE(USART1) == 0x01 || LL_USART_IsActiveFlag_ORE(USART1) == 0x01) {
		//Some Errors Interrupt

		//Clear all Error Flags
		LL_USART_ClearFlag_ORE(USART1);
	}
}

/**
  * @brief	UART2 IRQ Handler
  * @param	None
  * @return	None
  */
void USART2_IRQHandler(void) {
	//**********************//
	//		UART Frame		//
	//**********************//
	//|  u16   | n*u8 | u16 |
	//|--------|------|-----|
	//| Length | Data | CRC |
	if(LL_USART_IsEnabledIT_TXE(USART2) == 0x01 && LL_USART_IsActiveFlag_TXE(USART2) == 0x01) {
		if(uart2TXActiveBuffer == 0x01) {
			//Buffer 0 currently being transmitted
			if(uart2TXBufferIndex >= uart2TXBufferLength[0]) {
				//Transmission complete
				uart2TXBufferIndex = 0;

				if(uart2TXEmptyBuffer == 0x00) {
					uart2TXActiveBuffer = 0x02;
					uart2TXEmptyBuffer = 0x01;

					LL_USART_TransmitData8(USART2, uart2TXBuffer[1][uart2TXBufferIndex++]);
				}
				else {
					//Disable TX done interrupt
					uart2TXActiveBuffer = 0x00;
					uart2TXEmptyBuffer = 0x03;

					LL_USART_DisableIT_TXE(USART2);
				}
			}
			else {
				//Transmit another byte
				LL_USART_TransmitData8(USART2, uart2TXBuffer[0][uart2TXBufferIndex++]);
			}
		}
		else if(uart2TXActiveBuffer == 0x02) {
			//Buffer 1 currently being transmitted
			if(uart2TXBufferIndex >= uart2TXBufferLength[1]) {
				//Transmission complete
				uart2TXBufferIndex = 0;

				if(uart2TXEmptyBuffer == 0x00) {
					uart2TXActiveBuffer = 0x01;
					uart2TXEmptyBuffer = 0x02;

					LL_USART_TransmitData8(USART2, uart2TXBuffer[0][uart2TXBufferIndex++]);
				}
				else {
					//Disable TX done interrupt
					uart2TXActiveBuffer = 0x00;
					uart2TXEmptyBuffer = 0x03;

					LL_USART_DisableIT_TXE(USART2);
				}
			}
			else {
				//Transmit another byte
				LL_USART_TransmitData8(USART2, uart2TXBuffer[1][uart2TXBufferIndex++]);
			}
		}
	}

	if(LL_USART_IsActiveFlag_RXNE(USART2) == 0x01) {
		if(uart2RXBufferLength != 0x00) {
			//RX Buffer full, has a complete frame in it
			uint8_t dummy = LL_USART_ReceiveData8(USART2);
		}
		else if(uart2RXBufferIndex >= UART_RX_BUFFER_SIZE) {
			//RX Buffer overflow
			uint8_t dummy = LL_USART_ReceiveData8(USART2);

			uart2RXBufferIndex = 0;
		}
		else {
			//All good, read received byte to RX buffer
			uint8_t rxByte = LL_USART_ReceiveData8(USART2);
			uart2RXBuffer[uart2RXBufferIndex++] = rxByte;

			//Payload length byte received
			uint16_t payloadLength = (uart2RXBuffer[0] << 8) + uart2RXBuffer[1];

			//Update CRC
			if(uart2RXBufferIndex == 1) {
				CRCReset(&uart2RXCRC);
				CRCWrite(rxByte, &uart2RXCRC);
			}
			else if(uart2RXBufferIndex <= (payloadLength + 2)) {
				CRCWrite(rxByte, &uart2RXCRC);
			}

			//End of transmission detecteion based on UART frame length, in bytes 0 and 1
			if(uart2RXBufferIndex >= 2) {
				//Check for end of packet
				if(payloadLength < 1) {
					//Invalid payload length
					uart2RXBufferIndex = 0;
				}
				else if(payloadLength > UART_RX_BUFFER_SIZE) {
					//Buffer overflow
					uart2RXBufferIndex = 0;
				}
				else if(uart2RXBufferIndex >= (payloadLength + 4)) {
					//Full packet received, get crc from packege (last two bytes)
					//CRC Check
					uint16_t crc = (uart2RXBuffer[uart2RXBufferIndex - 2] << 8) + uart2RXBuffer[uart2RXBufferIndex - 1];
					if (crc != CRCRead(&uart2RXCRC)) {
						//Packet CRC failed
						uart2RXBufferIndex = 0;
					}
					else {
						uart2RXBufferLength = uart2RXBufferIndex;
					}
				}
			}
		}
	}

	if(LL_USART_IsActiveFlag_IDLE(USART2) == 0x01) {
		//End of frame transmission, detected by receiver timeout
//		uart2RXBufferLength = uart2RXBufferIndex;

		//Clear IDLE Flag
		LL_USART_ClearFlag_IDLE(USART2);
	}

	if(LL_USART_IsActiveFlag_FE(USART2) == 0x01 || LL_USART_IsActiveFlag_NE(USART2) == 0x01 || LL_USART_IsActiveFlag_ORE(USART2) == 0x01) {
		//Some Errors Interrupt

		//Clear all Error Flags
		LL_USART_ClearFlag_ORE(USART2);
	}
}

