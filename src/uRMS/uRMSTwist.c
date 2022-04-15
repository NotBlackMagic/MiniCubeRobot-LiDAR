#include "uRMSTwist.h"

/**
  * @brief	This function serializes a RobotTwistPacket into a byte array
  * @param	src: RobotTwistPacket to serialize
  * @param	dst: Pointer to byte array to write to
  * @return	Number of bytes written to byte array
  */
uint16_t RobotSerialize_RobotTwistPacket(RobotTwistPacket src, uint8_t* dst) {
	uint8_t i = 0;
	i += RobotSerialize_Vector3(src.velocityLinear, &dst[i]);
	i += RobotSerialize_Vector3(src.velocityAngular, &dst[i]);
	return i;
}

/**
  * @brief	This function de-serializess a byte array to a RobotTwistPacket
  * @param	src: pointer to byte array to de-serializes
  * @param	dst: pointer to RobotTwistPacket to write to
  * @return	Number of bytes read/converted from the byte array
  */
uint16_t RobotDeserialize_RobotTwistPacket(uint8_t* src, RobotTwistPacket* dst) {
	uint8_t i = 0;
	i += RobotDeserialize_Vector3(&src[i], &dst->velocityLinear);
	i += RobotDeserialize_Vector3(&src[i], &dst->velocityAngular);
	return i;
}
