#include "uRMSMessage.h"

/**
  * @brief	This function encodes a robot message from the packet struct to a byte array
  * @param	packet: Robot message to encode
  * @param	data: Pointer, byte array, to write the encoded message to
  * @param	dataLength: Pointer to write the encoded message length to
  * @return	0: Success, 1: Error/fail
  */
uint8_t RobotMessageEncode(RobotMsgStruct packet, uint8_t* data, uint16_t* dataLength) {
	//Add Header
	uint16_t i = 0;
	i += RobotSerialize_uint16(packet.topicID, &data[i]);
	i += RobotSerialize_uint32(packet.nsec, &data[i]);
	i += RobotSerialize_uint16(packet.frameID, &data[i]);

	//Add payload
	memcpy(&data[i], packet.payload, packet.payloadLength);
	i = i + packet.payloadLength;

	*dataLength = i;

	return 0;
}

/**
  * @brief	This function decodes a robot message from a byte array to a packet struct
  * @param	packet: Pointer, packet, to write the decoded message to
  * @param	data: Byte array to decode into a robot message packet
  * @param	dataLength: Length of the byte array to decode
  * @return	0: Success, 1: Error/fail
  */
uint8_t RobotMessageDecode(RobotMsgStruct* packet, uint8_t* data, uint16_t dataLength) {
	//Get Header
	uint16_t i = 0;
	i += RobotDeserialize_uint16(&data[i], &packet->topicID);
	i += RobotDeserialize_uint32(&data[i], &packet->nsec);
	i += RobotDeserialize_uint16(&data[i], &packet->frameID);

	packet->payloadLength = dataLength - ROBOT_MSG_PKT_HEADER_SIZE;

	//Get payload
	memcpy(packet->payload, &data[i], packet->payloadLength);
	i = i + packet->payloadLength;

	return 0;
}

