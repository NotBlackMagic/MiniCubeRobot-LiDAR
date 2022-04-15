#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "rcc.h"
#include "rtc.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"
#include "usb_vcp.h"

#include "pinMaping.h"

#include "robotMsgs_defines.h"
#include "uRMSMessage.h"
#include "uRMSLaserScan.h"

#include "laserScanner.h"

uint8_t robotMsgPayload[1024];
RobotMsgStruct robotMsg = {	.frameID = 0,
							.payload = robotMsgPayload};

int main(void) {
	//Configure the system clock
	SystemClockInit();
	SystemTickInit();
	RTCInit();

	//Initialize all configured peripherals
	GPIOInit();
	UART1Init(115200);
	UART2Init(115200);
	I2C1Init(I2CMode_FM);		//Fast mode (400 kHz) is maximum speed for STM32F103
	I2C2Init(I2CMode_FM);
	SPI1Init();
	ADC1Init();
	USBVCPInit();

	//Start ADC
	ADC1Start();

	//Set Time
	RTCSetTime(14, 25, 00);

	//Init Laser Scanner
	LaserScannerInit();

	GPIOWrite(GPIO_OUT_LED0, 0x01);

	uint32_t laserScanMsgTimestamp = GetSysTick();

	uint8_t txData[1050];
	uint16_t txLength;
	uint8_t rxData[512];
	uint16_t rxLength;
	while(1) {
		//USB Communication Check
		if(USBVCPRead(rxData, &rxLength) == 0x01) {

		}

		//UART1 Communication Check
		if(UART1Read(rxData, &rxLength) == 0x01) {
			//From Board bellow, Motor Drive Board, forward to other UART, connected to Bluetooth
			if(UART2Write(rxData, rxLength) != 0x00) {
				//Retransmit data failed
			}
		}

		//UART2 Communication Check
		if(UART2Read(rxData, &rxLength) == 0x01) {
			//New command received, decode byte array into packet
			if(UART1Write(rxData, rxLength) != 0x00) {
				//Retransmit data failed
			}
//			uint8_t pktPayload[100];
//			RobotMsgStruct packet = {.payload = pktPayload};
//			uint8_t error = RobotMessageDecode(&packet, rxData, rxLength);
		}

		//Update Calls
		LaserScannerUpdate();

		//Send Laser Scan Message
		if((laserScanMsgTimestamp + 1000) < GetSysTick()) {
			//Set Robot Message ID
			robotMsg.topicID = ROBOT_MSGS_LASER_SCAN;

			//Get uptime in ms
			robotMsg.nsec = GetSysTick();

			//Get Laser Scan Info and assemble into packet payload
			robotMsg.payloadLength = 0;
			robotMsg.payloadLength += RobotSerialize_RobotLaserScanPacket(laserScanPacket, robotMsg.payload);

			//Encode packet into byte array
			RobotMessageEncode(robotMsg, txData, &txLength);

			//Write Data over UART
			UART2Write(txData, txLength);

			laserScanMsgTimestamp = GetSysTick();
		}
	}
}

void Error_Handler(void) {

}
