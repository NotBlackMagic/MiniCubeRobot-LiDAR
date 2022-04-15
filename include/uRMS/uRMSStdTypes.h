#ifndef URMS_URMSSTDTYPES_H_
#define URMS_URMSSTDTYPES_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"
#include "string.h"

typedef struct Vector3_q31 {
	int32_t x;
	int32_t y;
	int32_t z;
} Vector3_q31;

uint16_t RobotSerialize_uint16(uint16_t src, uint8_t* dst);
uint16_t RobotDeserialize_uint16(uint8_t* src, uint16_t* dst);
uint16_t RobotSerialize_uint32(uint32_t src, uint8_t* dst);
uint16_t RobotDeserialize_uint32(uint8_t* src, uint32_t* dst);
uint16_t RobotSerialize_Vector3(Vector3_q31 src, uint8_t* dst);
uint16_t RobotDeserialize_Vector3(uint8_t* src, Vector3_q31* dst);

#ifdef __cplusplus
}
#endif

#endif /* URMS_URMSSTDTYPES_H_ */
