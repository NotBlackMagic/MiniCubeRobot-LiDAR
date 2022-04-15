#ifndef URMS_URMSDRIVE_H_
#define URMS_URMSDRIVE_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "uRMSStdTypes.h"

typedef struct RobotDrivePacket {
	int32_t wheelRPMLeft;
	int32_t wheelRPMRight;
	uint16_t motorCurrentLeft;
	uint16_t motorCurrentRight;
	uint8_t motorDriveStatus;
} RobotDrivePacket;

uint16_t RobotSerialize_RobotDrivePacket(RobotDrivePacket src, uint8_t* dst);
uint16_t RobotDeserialize_RobotDrivePacket(uint8_t* src, RobotDrivePacket* dst);

#ifdef __cplusplus
}
#endif

#endif /* URMS_URMSDRIVE_H_ */
