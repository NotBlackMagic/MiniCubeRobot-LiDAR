#include "vd6283.h"

uint8_t VD6283ReadRegister(uint8_t reg) {
	uint8_t dataBuffer[2];
	dataBuffer[0] = reg;
	I2C2Write(VD6283_I2C_ADD, dataBuffer, 1);
	I2C2Read(VD6283_I2C_ADD, dataBuffer, 1);
	return dataBuffer[0];
}

void VD6283WriteRegister(uint8_t reg, uint8_t data) {
	uint8_t dataBuffer[2];
	dataBuffer[0] = reg;
	dataBuffer[1] = data;
	I2C2Write(VD6283_I2C_ADD, dataBuffer, 2);
}

void VD6283ModifyRegister(uint8_t reg, uint8_t data, uint8_t mask) {
	uint8_t regData = VD6283ReadRegister(reg);
	regData &= ~mask;
	regData |= data;
	VD6283WriteRegister(reg, regData);
}

void VD6283Init() {
	//Configure Base configs like interfaces, interrupts, etc
	VD6283WriteRegister(VD6283_ALS_CTRL, 0x00);			//ALS Operation Stopped (Idle)
	VD6283WriteRegister(VD6283_CH6_EN, 0x01);			//Enable Channel 6
	VD6283WriteRegister(VD6283_ALS_CH_EN, 0x1F);		//Enable ALL ALS Channels
	VD6283WriteRegister(VD6283_AC_MODE_CTRL, 0x13);		//AC Mode: On GPIO2 + from channel 6 + Enable flicker frequency extraction
	VD6283WriteRegister(VD6283_SDA_DRV_CFG, 0x09);		//SDA Drive: 95pF + Drive strength 2 (8 mA) (defaults)
	VD6283WriteRegister(VD6283_GPIO1_DRV_CFG, 0x00);	//GPIO1 Drive: Open Drain Output (default)
}

uint8_t VD6283GetID() {
	return VD6283ReadRegister(VD6283_DEVICE_ID);
}

uint8_t VD6283GetRevision() {
	return VD6283ReadRegister(VD6283_REVISION_ID);
}

uint8_t VD6283GetInterruptStatus() {
	return ((VD6283ReadRegister(VD6283_INT_CTRL) >> 1) & 0x01);
}

void VD6283ClearInterruptFlag() {
	VD6283WriteRegister(VD6283_INT_CTRL, 0x01);
	VD6283WriteRegister(VD6283_INT_CTRL, 0x00);
}

void VD6283StartMeasurment() {
	//Single ALS measurement request
	VD6283WriteRegister(VD6283_ALS_CTRL, 0x01);
}

void VD6283SetALSPeriod(uint16_t period) {
	//LSB = 20.5 ms
	if(period > 5120) {
		return;
	}
	uint8_t value = ((uint32_t)period / 20);
	VD6283WriteRegister(VD6283_ALS_PERIOD, value);
}

void VD6283SetALSExposure(uint16_t exposure) {
	//Exposure time = (EXTIME[9:0] + 1) x 16384/Fosc (Fosc = 10.24MHz)
	if(exposure > 1600) {
		return;
	}
	uint16_t value = (exposure * 5) >> 3;
	VD6283WriteRegister(VD6283_ALS_EXPOSURE_M, ((value >> 2) & 0x03));
	VD6283WriteRegister(VD6283_ALS_EXPOSURE_L, ((value) & 0xFF));
}

void VD6283SetALSGain(uint8_t channel, VD6283ALSGain alsGain) {
	switch(channel) {
		case 0x01:
			VD6283WriteRegister(VD6283_ALS_GAIN_CH1, alsGain);
			break;
		case 0x02:
			VD6283WriteRegister(VD6283_ALS_GAIN_CH2, alsGain);
			break;
		case 0x03:
			VD6283WriteRegister(VD6283_ALS_GAIN_CH3, alsGain);
			break;
		case 0x04:
			VD6283WriteRegister(VD6283_ALS_GAIN_CH4, alsGain);
			break;
		case 0x05:
			VD6283WriteRegister(VD6283_ALS_GAIN_CH5, alsGain);
			break;
		case 0x06:
			VD6283WriteRegister(VD6283_ALS_GAIN_CH6, alsGain);
			break;
		default:
			break;
	}
}

//Channel 1 (Red): [13.1; 14.6; 16.0] Counts/(uW/cm2) @ AGAIN = 10x EXTIME = 50ms
//Channel 2 (Visible): [44.7; 49.7; 54.6] Counts/(uW/cm2) @ AGAIN = 10x EXTIME = 50ms
//Channel 3 (Blue): [13.1; 14.6; 16.0] Counts/(uW/cm2) @ AGAIN = 10x EXTIME = 50ms
//Channel 4 (Green): [26.5; 29.5; 32.4] Counts/(uW/cm2) @ AGAIN = 10x EXTIME = 50ms
//Channel 5 (IR): [77.4; 86.0; 94.6] Counts/(uW/cm2) @ AGAIN = 10x EXTIME = 50ms
//Channel 6 (Clear): [56.1; 62.3; 68.6] Counts/(uW/cm2) @ AGAIN = 5x EXTIME = 50ms
void VD6283GetALSChannelData(uint8_t channel, uint32_t* alsData) {
	switch(channel) {
		case 0x01:
			*alsData = (VD6283ReadRegister(VD6283_ALS_CH1_DATA_H) << 16);
			*alsData += (VD6283ReadRegister(VD6283_ALS_CH1_DATA_M) << 8);
			*alsData += VD6283ReadRegister(VD6283_ALS_CH1_DATA_L);
			break;
		case 0x02:
			*alsData = (VD6283ReadRegister(VD6283_ALS_CH2_DATA_H) << 16);
			*alsData += (VD6283ReadRegister(VD6283_ALS_CH2_DATA_M) << 8);
			*alsData += VD6283ReadRegister(VD6283_ALS_CH2_DATA_L);
			break;
		case 0x03:
			*alsData = (VD6283ReadRegister(VD6283_ALS_CH3_DATA_H) << 16);
			*alsData += (VD6283ReadRegister(VD6283_ALS_CH3_DATA_M) << 8);
			*alsData += VD6283ReadRegister(VD6283_ALS_CH3_DATA_L);
			break;
		case 0x04:
			*alsData = (VD6283ReadRegister(VD6283_ALS_CH4_DATA_H) << 16);
			*alsData += (VD6283ReadRegister(VD6283_ALS_CH4_DATA_M) << 8);
			*alsData += VD6283ReadRegister(VD6283_ALS_CH4_DATA_L);
			break;
		case 0x05:
			*alsData = (VD6283ReadRegister(VD6283_ALS_CH5_DATA_H) << 16);
			*alsData += (VD6283ReadRegister(VD6283_ALS_CH5_DATA_M) << 8);
			*alsData += VD6283ReadRegister(VD6283_ALS_CH5_DATA_L);
			break;
		case 0x06:
			*alsData = (VD6283ReadRegister(VD6283_ALS_CH6_DATA_H) << 16);
			*alsData += (VD6283ReadRegister(VD6283_ALS_CH6_DATA_M) << 8);
			*alsData += VD6283ReadRegister(VD6283_ALS_CH6_DATA_L);
			break;
		default:
			break;
	}
}
