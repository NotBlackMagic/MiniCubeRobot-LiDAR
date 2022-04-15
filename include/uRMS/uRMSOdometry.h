#ifndef URMS_URMSODOMETRY_H_
#define URMS_URMSODOMETRY_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "uRMSStdTypes.h"

typedef struct RobotOdometryPacket {
	Vector3_q31 velocityLinear;
	Vector3_q31 velocityAngular;
	Vector3_q31 posePoint;
	Vector3_q31 poseOrientation;
} RobotOdometryPacket;

uint16_t RobotSerialize_RobotOdometryPacket(RobotOdometryPacket src, uint8_t* dst);
uint16_t RobotDeserialize_RobotOdometryPacket(uint8_t* src, RobotOdometryPacket* dst);

#ifdef __cplusplus
}
#endif

#endif /* URMS_URMSODOMETRY_H_ */
