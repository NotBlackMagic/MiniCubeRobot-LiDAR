#include "uRMSLaserScan.h"

/**
  * @brief	This function serializes a RobotLaserScanPacket into a byte array
  * @param	src: RobotLaserScanPacket to serialize
  * @param	dst: Pointer to byte array to write to
  * @return	Number of bytes written to byte array
  */
uint16_t RobotSerialize_RobotLaserScanPacket(RobotLaserScanPacket src, uint8_t* dst) {
	uint16_t i = 0;
	dst[i++] = src.rangeCountV;
	dst[i++] = src.rangeCountH;
	uint8_t v, h;
	for(v = 0; v < src.rangeCountV; v++) {
		for(h = 0; h < src.rangeCountH; h++) {
			i += RobotSerialize_uint16(src.ranges[v][h], &dst[i]);
		}
	}
	return i;
}

/**
  * @brief	This function de-serializess a byte array to a RobotLaserScanPacket
  * @param	src: pointer to byte array to de-serializes
  * @param	dst: pointer to RobotLaserScanPacket to write to
  * @return	Number of bytes read/converted from the byte array
  */
uint16_t RobotDeserialize_RobotLaserScanPacket(uint8_t* src, RobotLaserScanPacket* dst) {
	uint16_t i = 0;
	dst->rangeCountV = src[i++];
	dst->rangeCountH = src[i++];
	uint8_t v, h;
	for(v = 0; v < dst->rangeCountV; v++) {
		for(h = 0; h < dst->rangeCountH; h++) {
			i += RobotSerialize_uint16(&src[i], &dst->ranges[v][h]);
		}
	}
	return i;
}
