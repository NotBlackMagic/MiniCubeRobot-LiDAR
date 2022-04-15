#include "uRMSContact.h"

/**
  * @brief	This function serializes a RobotContactPacket into a byte array
  * @param	src: RobotContactPacket to serialize
  * @param	dst: Pointer to byte array to write to
  * @return	Number of bytes written to byte array
  */
uint16_t RobotSerialize_RobotContactPacket(RobotContactPacket src, uint8_t* dst) {
	uint8_t i = 0;
	dst[i++] = src.count;
	uint8_t j;
	for(j = 0; j < src.count; j++) {
		dst[i++] = src.contact[j];
	}
	return i;
}

/**
  * @brief	This function de-serializess a byte array to a RobotContactPacket
  * @param	src: pointer to byte array to de-serializes
  * @param	dst: pointer to RobotContactPacket to write to
  * @return	Number of bytes read/converted from the byte array
  */
uint16_t RobotDeserialize_RobotContactPacket(uint8_t* src, RobotContactPacket* dst) {
	uint8_t i = 0;
	dst->count = src[i++];
	uint8_t j;
	for(j = 0; j < dst->count; j++) {
		dst->contact[j] = src[i++];
	}
	return i;
}
