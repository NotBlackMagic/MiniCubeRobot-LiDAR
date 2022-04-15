#ifndef URMS_URMSCONTACT_H_
#define URMS_URMSCONTACT_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "uRMSStdTypes.h"

typedef struct RobotContactPacket {
	uint8_t count;
	uint8_t contact[4];
} RobotContactPacket;

uint16_t RobotSerialize_RobotContactPacket(RobotContactPacket src, uint8_t* dst);
uint16_t RobotDeserialize_RobotContactPacket(uint8_t* src, RobotContactPacket* dst);

#ifdef __cplusplus
}
#endif

#endif /* URMS_URMSCONTACT_H_ */
