#include "uRMSDrive.h"

/**
  * @brief	This function serializes a RobotDrivePacket into a byte array
  * @param	src: RobotDrivePacket to serialize
  * @param	dst: Pointer to byte array to write to
  * @return	Number of bytes written to byte array
  */
uint16_t RobotSerialize_RobotDrivePacket(RobotDrivePacket src, uint8_t* dst) {
	uint8_t i = 0;
	i += RobotSerialize_uint32(src.wheelRPMLeft, &dst[i]);
	i += RobotSerialize_uint32(src.wheelRPMRight, &dst[i]);
	i += RobotSerialize_uint16(src.motorCurrentLeft, &dst[i]);
	i += RobotSerialize_uint16(src.motorCurrentRight, &dst[i]);
	dst[i++] = src.motorDriveStatus;
	return i;
}

/**
  * @brief	This function de-serializess a byte array to a RobotDrivePacket
  * @param	src: pointer to byte array to de-serializes
  * @param	dst: pointer to RobotDrivePacket to write to
  * @return	Number of bytes read/converted from the byte array
  */
uint16_t RobotDeserialize_RobotDrivePacket(uint8_t* src, RobotDrivePacket* dst) {
	uint8_t i = 0;
	i += RobotDeserialize_uint32(&src[i], &dst->wheelRPMLeft);
	i += RobotDeserialize_uint32(&src[i], &dst->wheelRPMRight);
	i += RobotDeserialize_uint16(&src[i], &dst->motorCurrentLeft);
	i += RobotDeserialize_uint16(&src[i], &dst->motorCurrentRight);
	dst->motorDriveStatus = src[i++];
	return i;
}
