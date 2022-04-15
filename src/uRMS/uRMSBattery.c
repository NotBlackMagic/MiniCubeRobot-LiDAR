#include "uRMSBattery.h"

/**
  * @brief	This function serializes a RobotBatteryPacket into a byte array
  * @param	src: RobotBatteryPacket to serialize
  * @param	dst: Pointer to byte array to write to
  * @return	Number of bytes written to byte array
  */
uint16_t RobotSerialize_RobotBatteryPacket(RobotBatteryPacket src, uint8_t* dst) {
	uint8_t i = 0;
	i += RobotSerialize_uint16(src.voltage, &dst[i]);
	i += RobotSerialize_uint16(src.current, &dst[i]);
	i += RobotSerialize_uint16(src.charge, &dst[i]);
	dst[i++] = src.percentage;
	dst[i++] = src.status;
	return i;
}

/**
  * @brief	This function de-serializess a byte array to a RobotBatteryPacket
  * @param	src: pointer to byte array to de-serializes
  * @param	dst: pointer to RobotBatteryPacket to write to
  * @return	Number of bytes read/converted from the byte array
  */
uint16_t RobotDeserialize_RobotBatteryPacket(uint8_t* src, RobotBatteryPacket* dst) {
	uint8_t i = 0;
	i += RobotDeserialize_uint16(&src[i], &dst->voltage);
	i += RobotDeserialize_uint16(&src[i], &dst->current);
	i += RobotDeserialize_uint16(&src[i], &dst->charge);
	dst->percentage = src[i++];
	dst->status = src[i++];
	return i;
}
