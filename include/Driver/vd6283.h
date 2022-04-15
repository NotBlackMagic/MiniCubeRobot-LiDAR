#ifndef DRIVER_VD6283_H_
#define DRIVER_VD6283_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "gpio.h"
#include "i2c.h"
#include "rcc.h"

#include "vd6283_regs.h"

typedef enum {
	ALSGain_66x6 = 1,
	ALSGain_50x,
	ALSGain_33x,
	ALSGain_25x,
	ALSGain_16x,
	ALSGain_10x,
	ALSGain_7x1,
	ALSGain_5x,
	ALSGain_3x33,
	ALSGain_2x5,
	ALSGain_1x67,
	ALSGain_1x25,
	ALSGain_1x,
	ALSGain_0x83,
	ALSGain_0x71
} VD6283ALSGain;

void VD6283Init();
uint8_t VD6283GetID();
uint8_t VD6283GetRevision();
uint8_t VD6283GetInterruptStatus();
void VD6283ClearInterruptFlag();
void VD6283StartMeasurment();
void VD6283SetALSPeriod(uint16_t period);
void VD6283SetALSExposure(uint16_t exposure);
void VD6283SetALSGain(uint8_t channel, VD6283ALSGain alsGain);
void VD6283GetALSChannelData(uint8_t channel, uint32_t* alsData);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_VD6283_H_ */
