#include "uRMSStdTypes.h"

/**
  * @brief	This function serializes a uint16 into a byte array
  * @param	src: uint16 to serialize
  * @param	dst: pointer to byte array to write to
  * @return	Number of bytes written to byte array
  */
uint16_t RobotSerialize_uint16(uint16_t src, uint8_t* dst) {
	uint8_t i = 0;
	dst[i++] = (uint8_t)(src >> 8);
	dst[i++] = (uint8_t)(src);
	return i;
}

/**
  * @brief	This function de-serializess a byte array to a uint16
  * @param	src: pointer to byte array to de-serializes
  * @param	dst: pointer to uint16 to write to
  * @return	Number of bytes read/converted from the byte array
  */
uint16_t RobotDeserialize_uint16(uint8_t* src, uint16_t* dst) {
	uint8_t i = 0;
	*dst = (src[i++] << 8);
	*dst += (src[i++]);
	return i;
}

/**
  * @brief	This function serializes a uint32 into a byte array
  * @param	src: uint32 to serialize
  * @param	dst: pointer to byte array to write to
  * @return	Number of bytes written to byte array
  */
uint16_t RobotSerialize_uint32(uint32_t src, uint8_t* dst) {
	uint8_t i = 0;
	dst[i++] = (uint8_t)(src >> 24);
	dst[i++] = (uint8_t)(src >> 16);
	dst[i++] = (uint8_t)(src >> 8);
	dst[i++] = (uint8_t)(src);
	return i;
}

/**
  * @brief	This function de-serializess a byte array to a int32
  * @param	src: pointer to byte array to de-serializes
  * @param	dst: pointer to uint32 to write to
  * @return	Number of bytes read/converted from the byte array
  */
uint16_t RobotDeserialize_uint32(uint8_t* src, uint32_t* dst) {
	uint16_t i = 0;
	*dst = (src[i++] << 24);
	*dst += (src[i++] << 16);
	*dst += (src[i++] << 8);
	*dst += (src[i++]);
	return i;
}

/**
  * @brief	This function serializes a Vector3_q31 into a byte array
  * @param	src: Vector3_q31 to serialize
  * @param	dst: pointer to byte array to write to
  * @return	Number of bytes written to byte array
  */
uint16_t RobotSerialize_Vector3(Vector3_q31 src, uint8_t* dst) {
	uint8_t i = 0;
	i += RobotSerialize_uint32(src.x, &dst[i]);
	i += RobotSerialize_uint32(src.y, &dst[i]);
	i += RobotSerialize_uint32(src.z, &dst[i]);
	return i;
}

/**
  * @brief	This function de-serializes a byte array to a Vector3_q31
  * @param	src: pointer to byte array to de-serializes
  * @param	dst: pointer to Vector3_q31 to write to
  * @return	Number of bytes read/converted from the byte array
  */
uint16_t RobotDeserialize_Vector3(uint8_t* src, Vector3_q31* dst) {
	uint8_t i = 0;
	i += RobotDeserialize_uint32(&src[i], &dst->x);
	i += RobotDeserialize_uint32(&src[i], &dst->y);
	i += RobotDeserialize_uint32(&src[i], &dst->z);
	return i;
}
