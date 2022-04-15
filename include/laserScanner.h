#ifndef LASERSCANNER_H_
#define LASERSCANNER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "gpio.h"
#include "i2c.h"
#include "rcc.h"

#include "pinMaping.h"

#include "uRMSLaserScan.h"

#include "vl53l5cx_api.h"

extern RobotLaserScanPacket laserScanPacket;

void LaserScannerInit();
void LaserScannerUpdate();

#ifdef __cplusplus
}
#endif

#endif /* LASERSCANNER_H_ */
