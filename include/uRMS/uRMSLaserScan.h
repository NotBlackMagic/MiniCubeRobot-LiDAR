#ifndef URMS_URMSLASERSCAN_H_
#define URMS_URMSLASERSCAN_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "uRMSStdTypes.h"

typedef struct RobotLaserScanPacket {
	uint16_t angleStart;		//Start angle of laser scan, radians Q4.12 [-2*pi; 2*pi]
	uint16_t angleStop;			//End angle of laser scan, radians Q4.12 [-2*pi; 2*pi]
	uint16_t angleStep;			//Step angle between two range points, radians Q4.12 [-2*pi; 2*pi]
	uint16_t scanTime;			//Time between two complete scans, in ms

	uint16_t rangeMin;			//Minimum range threshold, in mm
	uint16_t rangeMax;			//Maximum range threshold, in mm

	uint8_t rangeCountV;		//Number of vertical range data points, should be: (angleStop - angleStart) / angleStep
	uint8_t rangeCountH;		//Number of horizontal range data points, should be: (angleStop - angleStart) / angleStep
	uint16_t ranges[8][64];		//Range data points, in mm
} RobotLaserScanPacket;

uint16_t RobotSerialize_RobotLaserScanPacket(RobotLaserScanPacket src, uint8_t* dst);
uint16_t RobotDeserialize_RobotLaserScanPacket(uint8_t* src, RobotLaserScanPacket* dst);

#ifdef __cplusplus
}
#endif

#endif /* URMS_URMSLASERSCAN_H_ */
