#ifndef URMS_URMSBATTERY_H_
#define URMS_URMSBATTERY_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "uRMSStdTypes.h"

typedef struct RobotBatteryPacket {
	uint16_t current;
	uint16_t voltage;
	uint16_t charge;
	uint8_t percentage;
	uint8_t status;
} RobotBatteryPacket;

uint16_t RobotSerialize_RobotBatteryPacket(RobotBatteryPacket src, uint8_t* dst);
uint16_t RobotDeserialize_RobotBatteryPacket(uint8_t* src, RobotBatteryPacket* dst);

#ifdef __cplusplus
}
#endif

#endif /* URMS_URMSBATTERY_H_ */
