#ifndef URMS_URMSTWIST_H_
#define URMS_URMSTWIST_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "uRMSStdTypes.h"

typedef struct RobotTwistPacket {
	Vector3_q31 velocityLinear;
	Vector3_q31 velocityAngular;
} RobotTwistPacket;

uint16_t RobotSerialize_RobotTwistPacket(RobotTwistPacket src, uint8_t* dst);
uint16_t RobotDeserialize_RobotTwistPacket(uint8_t* src, RobotTwistPacket* dst);

#ifdef __cplusplus
}
#endif

#endif /* URMS_URMSTWIST_H_ */
