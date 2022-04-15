#include "spi.h"

#define SPI_TX_BUFFER_SIZE						128

uint16_t spi1TXBufferIndex;
uint16_t spi1TXBufferLength;
uint8_t spi1TXBuffer[SPI_TX_BUFFER_SIZE];

//SPI2 Buffer variables
volatile uint8_t spi2RXBufferLength;
volatile uint8_t spi2TXBufferLength;
volatile uint8_t spi2RXBufferIndex;
volatile uint8_t spi2TXBufferIndex;
volatile uint8_t spi2RTXBuffer[SPI2_BUFFER_LENGTH];

/**
  * @brief	This function initializes the SPI1 interface, also sets the respective GPIO pins
  * @param	None
  * @return	None
  */
void SPI1Init() {
	//Enable bus clocks
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);		//Enable Clock to SPI1
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);

	//Configure GPIOs
//	LL_GPIO_AF_EnableRemap_SPI1();		//Remap SPI1 GPIOs to Alternative GPIOs
	//Set SCLK, on PA5
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
	//Set MISO, on PA6
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_FLOATING);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
	//Set MOSI, on PA7
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
	//Set CS, on PA4
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);

	//Configure SPI Interface
	LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV2);
	LL_SPI_SetTransferDirection(SPI1, LL_SPI_FULL_DUPLEX);
	LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_1EDGE);
	LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_LOW);
	LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
	LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
	LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
	LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);

	//Configure SPI Interrupts
//	NVIC_SetPriority(SPI1_IRQn, 0);		// NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0)
//	NVIC_EnableIRQ(SPI1_IRQn);
//	LL_SPI_EnableIT_TXE(SPI1);
//	LL_SPI_EnableIT_RXNE(SPI1);
//	LL_SPI_EnableIT_ERR(SPI1);

	//Enable SPI
	LL_SPI_Enable(SPI1);
}

/**
  * @brief	This function initializes the SPI2 interface, also sets the respective GPIO pins
  * @param	None
  * @return	None
  */
void SPI2Init() {
	//Enable bus clocks
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);		//Enable Clock to SPI2
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);

	//Configure GPIOs
	//Set SCLK, on PB13
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);
	//Set MISO, on PB14
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_FLOATING);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_14, LL_GPIO_SPEED_FREQ_HIGH);
	//Set MOSI, on PB15
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_15, LL_GPIO_OUTPUT_PUSHPULL);
	//Set CS, on PB12
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_12, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_12, LL_GPIO_OUTPUT_PUSHPULL);

	//Configure SPI Interface
	LL_SPI_SetBaudRatePrescaler(SPI2, LL_SPI_BAUDRATEPRESCALER_DIV8);		//Set to: SPI_CLK = PCLK1 / DIV = 32MHz / 8 = 4MHz
	LL_SPI_SetTransferDirection(SPI2,LL_SPI_FULL_DUPLEX);
	LL_SPI_SetClockPhase(SPI2, LL_SPI_PHASE_1EDGE);
	LL_SPI_SetClockPolarity(SPI2, LL_SPI_POLARITY_LOW);
	LL_SPI_SetTransferBitOrder(SPI2, LL_SPI_MSB_FIRST);
	LL_SPI_SetDataWidth(SPI2, LL_SPI_DATAWIDTH_8BIT);
	LL_SPI_SetNSSMode(SPI2, LL_SPI_NSS_HARD_OUTPUT);
	LL_SPI_SetMode(SPI2, LL_SPI_MODE_MASTER);

	//Configure SPI Interrupts
	NVIC_SetPriority(SPI2_IRQn, 1);
	NVIC_EnableIRQ(SPI2_IRQn);
//	LL_SPI_EnableIT_TXE(SPI2);
	LL_SPI_EnableIT_RXNE(SPI2);
	LL_SPI_EnableIT_ERR(SPI2);

	LL_SPI_Enable(SPI2);
}

/**
  * @brief	This function performs a byte transfer on SPI1 (read and write 1 byte)
  * @param	txByte: Single Byte to be transmitted
  * @return	received single byte
  */
uint8_t SPI1ReadWrite(uint8_t txByte) {
	uint8_t rxByte;

	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
	LL_SPI_TransmitData8(SPI1, txByte);
	while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
	rxByte = LL_SPI_ReceiveData8(SPI1);

	return rxByte;
}

/**
  * @brief	This function performs a single byte transfer on SPI1 (ONLY write 1 byte)
  * @param	byte: Byte to be sent
  * @return	None
  */
void SPI1Write(uint8_t byte) {
//	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
	LL_SPI_TransmitData8(SPI1, byte);
}

/**
  * @brief	This function reads a received byte on SPI1, needs a call to SPI1Write before
  * @param	None
  * @return	Received byte
  */
uint8_t SPI1Read() {
	while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
	return LL_SPI_ReceiveData8(SPI1);
}

/**
  * @brief	This function performs a byte transfer on SPI2 (read and write 1 byte)
  * @param	txByte: Single Byte to be transmitted
  * @return	received single byte
  */
uint8_t SPI2ReadWrite(uint8_t txByte) {
	uint8_t rxByte;

	while(!LL_SPI_IsActiveFlag_TXE(SPI2));
	LL_SPI_TransmitData8(SPI2, txByte);
	while(!LL_SPI_IsActiveFlag_RXNE(SPI2));
	rxByte = LL_SPI_ReceiveData8(SPI2);

	return rxByte;
}

/**
  * @brief	This function reads the data received during last data transfer
  * @param	data: data array to where the received data should be copied to
  * @return	Returns the number of bytes read/received
  */
uint8_t SPI2Read(uint8_t* data) {
	if(spi2RXBufferLength == 0x00) {
		//RX Buffer not full
		return 0x00;
	}

	uint8_t i;
	for(i = 0; i < spi2RXBufferLength; i++) {
		data[i] = spi2RTXBuffer[i];
	}
	spi2RXBufferLength = 0x00;
	spi2RXBufferIndex = 0x00;

	return i;
}

/**
  * @brief	This function starts a new data full duplex transfer
  * @param	data: data array to transmit
  * @param	length: length of the transmit data array
  * @return	0 -> No Errors; 1 -> Error
  */
uint8_t SPI2Write(uint8_t* data, uint8_t length) {
	if(spi2TXBufferIndex != 0x00) {
		//TX Buffer not empty
		return 0x01;
	}

	if(length > SPI2_BUFFER_LENGTH) {
		//Buffer overflow
		return 0x01;
	}

	uint8_t i;
	for(i = 0; i < length; i++) {
		spi2RTXBuffer[i] = data[i];
	}
	spi2TXBufferLength = length;
	spi2TXBufferIndex = 0x00;

	//Enable SPI
//	LL_SPI_Enable(SPI2);

	//Write data to the SPI out register
	LL_SPI_TransmitData8(SPI2, spi2RTXBuffer[spi2TXBufferIndex++]);

//	LL_SPI_EnableIT_TXE(SPI2);

	return 0x00;
}

__attribute__((weak)) void SPI2RXCompleteCallback(uint8_t* data, uint16_t dataLength) {}

/**
  * @brief  This function handles SPI2 interrupt request.
  * @param  None
  * @return None
  */
void SPI2_IRQHandler(void) {
	//Check RXNE flag value in ISR register
	if(LL_SPI_IsActiveFlag_RXNE(SPI2) && LL_SPI_IsEnabledIT_RXNE(SPI2)) {
		spi2RTXBuffer[spi2RXBufferIndex++] = LL_SPI_ReceiveData8(SPI2);

		if(spi2RXBufferIndex >= spi2TXBufferLength) {
			//RX complete
			spi2RXBufferLength = spi2RXBufferIndex;
			spi2RXBufferIndex = 0x00;

			spi2TXBufferIndex = 0x00;

			//Call RX complete callback
			SPI2RXCompleteCallback(spi2RTXBuffer, spi2RXBufferLength);
		}
		else {
			//Get next TX ready
			LL_SPI_TransmitData8(SPI2, spi2RTXBuffer[spi2TXBufferIndex++]);
		}
	}

	//Check TXE flag value in ISR register
//	if(LL_SPI_IsActiveFlag_TXE(SPI2) && LL_SPI_IsEnabledIT_TXE(SPI2)) {
//		LL_SPI_TransmitData8(SPI2, spi2RTXBuffer[spi2TXBufferIndex++]);
//
//		if(spi2TXBufferIndex >= spi2TXBufferLength) {
//			//TX complete
////			spi2TXBufferLength = 0x00;		//Needed for RX ISR
////			spi2TXBufferIndex = 0x00;
//
//			LL_SPI_DisableIT_TXE(SPI2);
//		}
//	}

	//Check OVR flag value in ISR register
	if(LL_SPI_IsActiveFlag_OVR(SPI2)) {

	}
}
