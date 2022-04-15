/*******************************************************************************
* Copyright (c) 2020, STMicroelectronics - All Rights Reserved
*
* This file is part of the VL53L5CX Ultra Lite Driver and is dual licensed,
* either 'STMicroelectronics Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, the VL53L5CX Ultra Lite Driver may be distributed under the
* terms of 'BSD 3-clause "New" or "Revised" License', in which case the
* following provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
*******************************************************************************/

#include "platform.h"
#include "gpio.h"
#include "i2c.h"
#include "rcc.h"

uint8_t RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value) {
	uint8_t status = 0x00;
	uint8_t i2cBuffer[2];

	i2cBuffer[0] = (RegisterAdress >> 8) & 0xFF;
	i2cBuffer[1] = RegisterAdress & 0xFF;
	uint8_t status_i2c = I2C1Write(p_platform->address, i2cBuffer, 2);
	if(status_i2c != 0) {
		return 0x01;
	}

	status_i2c = I2C1Read(p_platform->address, p_value, 1);
	if(status_i2c != 0) {
		return 0x01;
	}

	return status;
}

uint8_t WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value) {
	uint8_t status = 0x00;
	uint8_t i2cBuffer[3];

	i2cBuffer[0] = (RegisterAdress >> 8) & 0xFF;
	i2cBuffer[1] = RegisterAdress & 0xFF;
	i2cBuffer[2] = value;
	uint8_t status_i2c = I2C1Write(p_platform->address, i2cBuffer, 3);
	if(status_i2c != 0) {
		return 0x01;
	}

	return status;
}

uint8_t WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
	uint8_t status = 0x00;
//	uint8_t i2cBuffer[256];

//	if (size > sizeof(i2cBuffer) - 1) {
//		return 0x01;
//	}
//
//	i2cBuffer[0] = (RegisterAdress >> 8) & 0xFF;
//	i2cBuffer[1] = RegisterAdress & 0xFF;
//	memcpy(&i2cBuffer[2], p_values, size);
//	uint8_t status_i2c = I2C1Write(p_platform->address, i2cBuffer, size + 2);
//	if(status_i2c != 0) {
//		return 0x01;
//	}

	uint8_t status_i2c = I2C1WriteRegister(p_platform->address, RegisterAdress, p_values, size);
	if(status_i2c != 0) {
		return 0x01;
	}

	return status;
}

uint8_t RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
	uint8_t status = 0x00;
	uint8_t i2cBuffer[2];

	i2cBuffer[0] = (RegisterAdress >> 8) & 0xFF;
	i2cBuffer[1] = RegisterAdress & 0xFF;
	uint8_t status_i2c = I2C1Write(p_platform->address, i2cBuffer, 2);
	if(status_i2c != 0) {
		return 0x01;
	}

	status_i2c = I2C1Read(p_platform->address, p_values, size);
	if(status_i2c != 0) {
		return 0x01;
	}

	return status;
}

uint8_t Reset_Sensor(VL53L5CX_Platform *p_platform) {
	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */

	/* Set pin LPN to LOW */
	GPIOWrite(p_platform->gpio_lp, 0);
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	WaitMs(p_platform, 100);

	/* Set pin LPN of to HIGH */
	GPIOWrite(p_platform->gpio_lp, 1);
	/* Set pin AVDD of to HIGH */
	/* Set pin VDDIO of  to HIGH */
	WaitMs(p_platform, 100);
  
	return 0;
}

void SwapBuffer(uint8_t *buffer, uint16_t size) {
	uint32_t i, tmp;

	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4)	{
		tmp = (	buffer[i] << 24)
				| (buffer[i+1] << 16)
				| (buffer[i+2] << 8)
				| (buffer[i+3]);

		memcpy(&(buffer[i]), &tmp, 4);
	}
}

uint8_t WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs) {
	Delay(TimeMs);
	return 0;
}
