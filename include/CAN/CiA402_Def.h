/*
 * CiA402_Def.h
 *
 *  Created on: Dec 6, 2017
 *      Author: spec
 */

#ifndef CIA402_DEF_H_
#define CIA402_DEF_H_

#define OBJ_READ 	1
#define OBJ_WRITE 	2

#define READ_REQUEST 			0x40
#define READ_1BYTE_RESPONSE 	0x4F
#define READ_2BYTE_RESPONSE 	0x4B
#define READ_4BYTE_RESPONSE 	0x43

#define WRITE_REQUEST_1BYTE 	0x2F
#define WRITE_REQUEST_2BYTE 	0x2B
#define WRITE_REQUEST_4BYTE 	0x23
#define WRITE_REQUEST_RESPONSE 	0x60

#define COB_NMT 0x000
#define COB_SYNC 0x80
#define COB_TPDO1 0x180
#define COB_RPDO1 0x200
#define COB_TPDO2 0x280
#define COB_RPDO2 0x300
#define COB_TPDO3 0x380
#define COB_RPDO3 0x400
#define COB_TPDO4 0x480
#define COB_RPDO4 0x500
#define COB_SDO 0x600

#define OBJ_CONTROLWORD 0x6040U
#define SUB_OBJ_SHUTDOWN 	0x06
#define SUB_OBJ_SWITCHON 	0x07
#define SUB_OBJ_ENABLE_OPERATION 0x0F
#define SUB_OBJ_DISABLE_VOLTAGE 0x02

#define OBJ_STATUSWORD 	0x6041U

#define OBJ_TARGET_TORQUE 0x6071U
//#define OBJ_TARGET_TORQUE 0x6074U

#define NMT_START_NODE 0x01
#define NMT_STOP_NODE 0x02
#define NMT_PREOP_MODE 0x80
#define NMT_RESET_NODE 0x81
#define NMT_RESET_COMMU 0x82

#define OBJ_MODES_OPERATION 0x6060U
#define OBJ_MODES_OPERATION_DISPLAY 0x6061U
#define NO_MODE 		0xFF
#define	POSITION_MODE 	0x02
#define VELOCITY_MODE 	0x03
#define TORQUE_MODE 	0x04
#define HOMING_MODE 	0x06

#define OBJ_RATE_CURRENT 0x6075U
#define OBJ_MODES_DISPLAY	0x6061U
#define OBJ_POSITION_ACTUAL 0x6063U
#define OBJ_VELOCITY_ACTUAL 0x6069U
#define OBJ_CURRENT_ACTUAL 0x6078U

#define OBJ_SUBINDEX_NULL 0x00U

#define PDO_4BYTE 0x20
#define PDO_2BYTE 0x10

#define TPDO1_INDEX 0x1A00U
#define TPDO2_INDEX 0x1A01U
#define TPDO2_INDEX2 0x1801U
#define TPDO3_INDEX 0x1A02U
#define TPDO4_INDEX 0x1A03U

#define RPDO1_INDEX 0x1600U
#define RPDO2_INDEX 0x1601U

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


#endif /* CIA402_DEF_H_ */
