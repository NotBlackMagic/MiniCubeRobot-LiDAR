#ifndef DRIVER_ROBOTMSGS_DEFINES_H_
#define DRIVER_ROBOTMSGS_DEFINES_H_

/********************************************************/
/*			ROBOT MESSAGES PACKET FORMAT				*/
/********************************************************/
//|   u16    | u32  |   u16    | n*u8 |
//|----------|------|----------|------|
//| Topic ID | nsec | Frame ID | Data |

#define ROBOT_MSG_PKT_HEADER_SIZE				0x08

/********************************************************/
/*					COMMAND MESSAGES					*/
/********************************************************/
#define ROBOT_CMD_REBOOT						0x00
#define ROBOT_CMD_ABORT							0x01
#define ROBOT_CMD_TWIST							0x02
#define ROBOT_CMD_TRANSFORM						0x03

/********************************************************/
/*					INFO MESSAGES						*/
/********************************************************/
#define ROBOT_MSGS_BATTERY						0x00
#define ROBOT_MSGS_ODOM							0x01
#define ROBOT_MSGS_RANGE						0x02
#define ROBOT_MSGS_CONTACT						0x03
#define ROBOT_MSGS_DRIVE						0x04
#define ROBOT_MSGS_TIME							0x05
#define ROBOT_MSGS_LASER_SCAN					0x06

/********************************************************/
/*					DEBUG MESSAGES						*/
/********************************************************/
#define ROBOT_DBG_DRIVE							0x44

typedef enum {
	RobotMsgs_Reboot = 0x00,
	RobotMsgs_Abort = 0x01,
	RobotMsgs_Twist = 0x02,
	RobotMsgs_Transform = 0x03
} RobotCmdMsgs;

typedef enum {
	RobotMsgs_Battery = 0x00,
	RobotMsgs_Odom = 0x01,
	RobotMsgs_Range = 0x02,
	RobotMsgs_Contact = 0x03,
	RobotMsgs_Drive = 0x04,
	RobotMsgs_Time = 0x05,
	RobotMsgs_LaserScan = 0x06
} RobotInfoMsgs;

typedef enum {
	RobotMsgs_DbgDrive = 0x44
} RobotDbgMsg;

#endif /* DRIVER_ROBOTMSGS_DEFINES_H_ */
