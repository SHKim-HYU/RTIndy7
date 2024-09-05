/*! 
 *  @file CiA402_Object_Dictionary.h
 *  @brief CiA402_Object_Dictionary
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Oct. 26. 2023
 *  @Comm
 */

#ifndef CIA402_OBJECT_DICTIONARY_H_
#define CIA402_OBJECT_DICTIONARY_H_

// Definition
#define OBJ_READ 	1
#define OBJ_WRITE 	2
#define PDO_READ	3

#define READ_REQUEST 			0x40
#define READ_1BYTE_RESPONSE 	0x4F
#define READ_2BYTE_RESPONSE 	0x4B
#define READ_4BYTE_RESPONSE 	0x43

#define WRITE_REQUEST_1BYTE 	0x2F
#define WRITE_REQUEST_2BYTE 	0x2B
#define WRITE_REQUEST_4BYTE 	0x23
#define WRITE_REQUEST_RESPONSE 	0x60


// COB Identifier --> COB-ID: Identifier + Node ID
#define COB_NMT 0x000
#define COB_SYNC 0x80
#define COB_TxPDO1 0x180
#define COB_RxPDO1 0x200
#define COB_TxPDO2 0x280
#define COB_RxPDO2 0x300
#define COB_TxPDO3 0x380
#define COB_RxPDO3 0x400
#define COB_TxPDO4 0x480
#define COB_RxPDO4 0x500
#define COB_TxSDO 0x580
#define COB_RxSDO 0x600

// Network Management (NMT)
#define NMT_START_NODE 0x01
#define NMT_STOP_NODE 0x02
#define NMT_PREOP_MODE 0x80
#define NMT_RESET_NODE 0x81
#define NMT_RESET_COMMU 0x82

// Statusword bits
#define STATUSWORD_READY_TO_SWITCH_ON_BIT 		0
#define STATUSWORD_SWITCHED_ON_BIT 				1
#define STATUSWORD_OPERATION_ENABLE_BIT 		2
#define STATUSWORD_FAULT_BIT 					3
#define STATUSWORD_VOLTAGE_ENABLE_BIT 			4
#define STATUSWORD_QUICK_STOP_BIT 				5
#define STATUSWORD_SWITCH_ON_DISABLE_BIT 		6
#define STATUSWORD_NO_USED_WARNING_BIT 			7
#define STATUSWORD_ELMO_NOT_USED_BIT 			8
#define STATUSWORD_REMOTE_BIT 					9
#define STATUSWORD_TARGET_REACHED_BIT 			10
#define STATUSWORD_INTERNAL_LIMIT_ACTIVE_BIT	11

// Object Dictionary
#define OBJ_MOTOR_DIRECTION			0x2047		// RW,	UINT8
#define OBJ_TORQUE_CONSTANT			0x2067		// RO,	UINT32
#define OBJ_CONTROLWORD				0x6040		// RW,	UINT16
#define OBJ_STATUSWORD				0x6041		// R,	UINT16
#define OBJ_OPERATIONMODE			0x6060		// RW,	INT8
#define OBJ_OPERATIONMODE_MONITOR	0x6061		// R,	INT8
#define OBJ_POSITION_ACTUAL 		0x6064		// R,	INT32
#define OBJ_VELOCITY_ACTUAL 		0x606C		// R,	INT32
#define OBJ_TARGET_TORQUE			0x6071		// RW,	INT16
#define OBJ_RATE_CURRENT			0x6075		// RW,	UINT32
#define OBJ_CURRENT_ACTUAL 			0x6078		// R,	INT16
#define OBJ_ENCODER_RESOLUTION		0x608f		// RW,	UINT32

#define OBJ_SUBINDEX_NULL 			0x00U

// Status
#define STATUS_NOT_SPECIFIED					0xffff
#define STATUS_NOT_READY_TO_SWITCH_ON 			0x0000
#define STATUS_SWITCH_ON_DISABLED 				0x0040
#define STATUS_READY_TO_SWITCH_ON 				0x0021
#define STATUS_SWITCHED_ON						0x0233
#define STATUS_OPERATION_ENABLED 				0x0237
#define STATUS_QUICK_STOP_ACTIVE 				0x0007
#define STATUS_FAULT_REACTION_ACTIVE 			0x000f
#define STATUS_FAULT 							0x0008

// CiA402 - Control State Machine							// MSB   LSB
#define CONTROL_COMMAND_DISABLE_VOLTAGE				0x01	// 0000 0001
#define CONTROL_COMMAND_QUICK_STOP					0x02	// 0000	0010	
#define CONTROL_COMMAND_SHUTDOWN					0x06	// 0000	0110
#define CONTROL_COMMAND_SWITCH_ON					0x07	// 0000	0111
#define CONTROL_COMMAND_ENABLE_OPERATION			0x0f	// 0000	1111
#define CONTROL_COMMAND_DISABLE_OPERATION			0x07	// 0000 0111
#define CONTROL_COMMAND_FAULT_RESET					0x80	// 1000 0000

// Operation mode
#define OP_MODE_NO_MODE					0x00
#define OP_MODE_PROFILE_POSITION		0x01
#define OP_MODE_VELOCITY				0x02
#define OP_MODE_PROFILE_VELOCITY		0x03
#define OP_MODE_TORQUE_PROFILE			0x04
#define OP_MODE_HOMING					0x06
#define OP_MODE_INTERPOLATED_POSITION	0x07
#define OP_MODE_CYCLIC_SYNC_POSITION	0x08
#define OP_MODE_CYCLIC_SYNC_VELOCITY	0x09
#define OP_MODE_CYCLIC_SYNC_TORQUE		0x0a

// Process Data Objects (PDO)
#define PDO_4BYTE 0x20
#define PDO_2BYTE 0x10
#define PDO_1BYTE 0x08
// RxPDO
#define RxPDO1_COMM 	0x1400
#define RxPDO1_MAP 		0x1600
#define RxPDO2_COMM 	0x1401
#define RxPDO2_MAP 		0x1601
#define RxPDO3_COMM 	0x1402
#define RxPDO3_MAP 		0x1602
#define RxPDO4_COMM 	0x1403
#define RxPDO4_MAP 		0x1603
// TxPDO
#define TxPDO1_COMM 	0x1800
#define TxPDO1_MAP 		0x1A00
#define TxPDO2_COMM 	0x1801
#define TxPDO2_MAP 		0x1A01
#define TxPDO3_COMM 	0x1802
#define TxPDO3_MAP 		0x1A02
#define TxPDO4_COMM 	0x1803
#define TxPDO4_MAP 		0x1A03


// SDO Packet
typedef union{
	struct{
		unsigned char 	type;
		unsigned char 	index_low;
		unsigned char 	index_high;
		unsigned char 	subindex;
		unsigned char 	data[4];
	}info;
	unsigned char value[8];
}SDO_PACKET;

// Object Data 4Byte
typedef union{
	unsigned char uint8Value[4];
	unsigned short uint16Value[2];
	unsigned long uint32Value;
}DATA_OBJECT;

typedef uint64_t 		UINT64;
typedef int64_t 		INT64;
typedef unsigned int 	UINT32;
typedef int32_t 		INT32;
typedef int16_t 		INT16;
typedef uint16_t 		UINT16;
typedef uint8_t 		UINT8;
typedef int8_t 			INT8;

#endif /* CIA402_OBJECT_DICTIONARY_H_ */
