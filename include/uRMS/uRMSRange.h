#ifndef URMS_URMSRANGE_H_
#define URMS_URMSRANGE_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "uRMSStdTypes.h"

typedef struct RobotRangePacket {
	uint8_t count;
	uint16_t ranges[6];
} RobotRangePacket;

uint16_t RobotSerialize_RobotRangePacket(RobotRangePacket src, uint8_t* dst);
uint16_t RobotDeserialize_RobotRangePacket(uint8_t* src, RobotRangePacket* dst);

#ifdef __cplusplus
}
#endif

#endif /* URMS_URMSRANGE_H_ */
