#include "uRMSOdometry.h"

/**
  * @brief	This function serializes a RobotOdometryPacket into a byte array
  * @param	src: RobotOdometryPacket to serialize
  * @param	dst: Pointer to byte array to write to
  * @return	Number of bytes written to byte array
  */
uint16_t RobotSerialize_RobotOdometryPacket(RobotOdometryPacket src, uint8_t* dst) {
	uint8_t i = 0;
	i += RobotSerialize_Vector3(src.velocityLinear, &dst[i]);
	i += RobotSerialize_Vector3(src.velocityAngular, &dst[i]);
	i += RobotSerialize_Vector3(src.posePoint, &dst[i]);
	i += RobotSerialize_Vector3(src.poseOrientation, &dst[i]);
	return i;
}

/**
  * @brief	This function de-serializess a byte array to a RobotOdometryPacket
  * @param	src: pointer to byte array to de-serializes
  * @param	dst: pointer to RobotOdometryPacket to write to
  * @return	Number of bytes read/converted from the byte array
  */
uint16_t RobotDeserialize_RobotOdometryPacket(uint8_t* src, RobotOdometryPacket* dst) {
	uint8_t i = 0;
	i += RobotDeserialize_Vector3(&src[i], &dst->velocityLinear);
	i += RobotDeserialize_Vector3(&src[i], &dst->velocityAngular);
	i += RobotDeserialize_Vector3(&src[i], &dst->posePoint);
	i += RobotDeserialize_Vector3(&src[i], &dst->poseOrientation);
	return i;
}
