#ifndef URMS_URMSMESSAGE_H_
#define URMS_URMSMESSAGE_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "uRMSStdTypes.h"

/********************************************************/
/*			ROBOT MESSAGES PACKET FORMAT				*/
/********************************************************/
//|   u16    | u32  |   u16    | n*u8 |
//|----------|------|----------|------|
//| Topic ID | nsec | Frame ID | Data |

 #define ROBOT_MSG_PKT_HEADER_SIZE				0x08

typedef struct RobotMsgStruct {
	uint16_t topicID;
	uint32_t nsec;
	uint16_t frameID;
	uint16_t payloadLength;
	uint8_t* payload;
} RobotMsgStruct;

uint8_t RobotMessageEncode(RobotMsgStruct packet, uint8_t* data, uint16_t* dataLength);
uint8_t RobotMessageDecode(RobotMsgStruct* packet, uint8_t* data, uint16_t dataLength);

#ifdef __cplusplus
}
#endif

#endif /* URMS_URMSMESSAGE_H_ */
