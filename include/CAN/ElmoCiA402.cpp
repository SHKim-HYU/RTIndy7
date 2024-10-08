// /*
//  * [WIP]ElmoCiA402.cpp
//  *
//  *  Created on: Dec 6, 2017
//  *      Author: spec
//  */

// #include "ElmoCiA402.h"

// Elmo_CiA402::Elmo_CiA402() {
// 	cob = 0x000;
// 	elmo_id = 0;
// 	res = 0;
// }

// Elmo_CiA402::~Elmo_CiA402() {
// }

// void Elmo_CiA402::SDO_CONTROLWORD(int NodeID, int RW, unsigned char data)
// {
// 	memset(&s_obj, 0, sizeof(DATA_OBJECT));
// 	memset(&s_packet, 0, sizeof(SDO_PACKET));
// 	memset(&rx_frame, 0, sizeof(can_frame));

// 	switch(RW)
// 	{
// 	case OBJ_READ:

// 		s_obj.uint16Value[0] = OBJ_STATUSWORD;

// 		memset(&s_packet, 0, sizeof(SDO_PACKET));
// 		s_packet.info.type = READ_REQUEST;
// 		s_packet.info.index_low = s_obj.uint8Value[0];
// 		s_packet.info.index_high = s_obj.uint8Value[1];
// 		s_packet.info.subindex = OBJ_SUBINDEX_NULL;
// 		cob = COB_SDO+NodeID;
// 		res = transmitter(cob, s_packet.value, 4);
// 		Print_CAN_FRAME(OBJ_WRITE);
// 		res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 		Print_CAN_FRAME(OBJ_READ);
// 		break;
// 	case OBJ_WRITE:

// 		s_obj.uint16Value[0] = OBJ_CONTROLWORD;

// 		if( data == SUB_OBJ_SHUTDOWN
// 				|| data == SUB_OBJ_SWITCHON
// 				|| data == SUB_OBJ_ENABLE_OPERATION
// 				|| data == SUB_OBJ_DISABLE_VOLTAGE )
// 		{
// 			s_packet.info.type = WRITE_REQUEST_2BYTE;
// 			s_packet.info.index_low = s_obj.uint8Value[0];
// 			s_packet.info.index_high = s_obj.uint8Value[1];
// 			s_packet.info.subindex = OBJ_SUBINDEX_NULL;
// 			s_packet.info.data[0] = data;
// 			cob = COB_SDO+NodeID;
// 			res = transmitter(cob, s_packet.value, 5);
// 			Print_CAN_FRAME(OBJ_WRITE);
// 			res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 			Print_CAN_FRAME(OBJ_READ);

// 		}
// 		break;
// 	default:
// 		break;

// 	}
// }

// void Elmo_CiA402::SDO_MODES_OPERTAION(unsigned char NodeID, int RW, unsigned char data)
// {
// 	memset(&s_obj, 0, sizeof(DATA_OBJECT));
// 	memset(&s_packet, 0, sizeof(SDO_PACKET));
// 	memset(&rx_frame, 0, sizeof(can_frame));

// 	switch(RW)
// 	{
// 	case OBJ_READ:

// 		s_obj.uint16Value[0] = OBJ_MODES_OPERATION_DISPLAY;

// 		s_packet.info.type = READ_REQUEST;
// 		s_packet.info.index_low = s_obj.uint8Value[0];
// 		s_packet.info.index_high = s_obj.uint8Value[1];
// 		s_packet.info.subindex = OBJ_SUBINDEX_NULL;
// 		cob = COB_SDO+NodeID;
// 		res = transmitter(cob, s_packet.value, 4);
// 		Print_CAN_FRAME(OBJ_WRITE);
// 		res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 		Print_CAN_FRAME(OBJ_READ);
// 		break;
// 	case OBJ_WRITE:

// 		s_obj.uint16Value[0] = OBJ_MODES_OPERATION;
// 		if(data == NO_MODE || data == POSITION_MODE || data == TORQUE_MODE)
// 		{
// 			s_packet.info.type = WRITE_REQUEST_2BYTE;
// 			s_packet.info.index_low = s_obj.uint8Value[0];
// 			s_packet.info.index_high = s_obj.uint8Value[1];
// 			s_packet.info.subindex = OBJ_SUBINDEX_NULL;
// 			s_packet.info.data[0] = data;
// 			cob = COB_SDO+NodeID;
// 			res = transmitter(cob, s_packet.value, 5);
// 			Print_CAN_FRAME(OBJ_WRITE);
// 			res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 			Print_CAN_FRAME(OBJ_READ);
// 		}

// 		break;
// 	default:
// 		break;

// 	}
// }

// void Elmo_CiA402::SDO_TARGET_TORQUE(unsigned char NodeID, int val)
// {
// 	memset(&s_obj, 0, sizeof(DATA_OBJECT));
// 	memset(&s_packet, 0, sizeof(SDO_PACKET));
// 	memset(&rx_frame, 0, sizeof(can_frame));

// 	cob = COB_SDO+NodeID;

// 	s_obj.uint16Value[0] = 0x6074;

// 	s_packet.info.type = WRITE_REQUEST_2BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = OBJ_SUBINDEX_NULL;

// 	unsigned short tmp = (unsigned short)val;

// 	s_packet.info.data[0] = tmp & 0x00FF;
// 	s_packet.info.data[1] = (tmp & 0xFF00) >> 8;

// 	res = transmitter(cob, s_packet.value, 6);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// }

// int Elmo_CiA402::SDO_RATE_CURRENT(unsigned char NodeID)
// {

// 	memset(&s_obj, 0, sizeof(DATA_OBJECT));
// 	memset(&s_packet, 0, sizeof(SDO_PACKET));
// 	memset(&rx_frame, 0, sizeof(can_frame));

// 	s_obj.uint16Value[0] = OBJ_RATE_CURRENT;
// 	s_packet.info.type = READ_REQUEST;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = OBJ_SUBINDEX_NULL;
// 	cob = COB_SDO+NodeID;
// 	res = transmitter(cob, s_packet.value, 6);
// 	Print_CAN_FRAME(OBJ_WRITE);

// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// 	int res = (int)(rx_frame.data[4] + (rx_frame.data[5]<<8));
// 	return res;
// }

// int Elmo_CiA402::SDO_RECIEVE(void)
// {
// 	memset(&can_elmo, 0, sizeof(can_frame));
// 	res = receiver(can_elmo.can_id, can_elmo.data, can_elmo.can_dlc);
// 	if(can_elmo.data[0] == 0x60)
// 		return (can_elmo.can_id & 0x00F);
// 	else
// 		return 0;
// }

// void Elmo_CiA402::NMT_STATE(unsigned char NodeID, unsigned char data)
// {
// 	memset(&s_packet, 0, sizeof(SDO_PACKET));

// 	cob = COB_NMT;
// 	s_packet.info.type = data;
// 	s_packet.info.index_low = NodeID;
// 	res = transmitter(cob, s_packet.value, 2);
// 	Print_CAN_FRAME(OBJ_WRITE);
// }

// void Elmo_CiA402::PDO_STOP(unsigned char NodeID, unsigned char TPDO_VAL)
// {
// 	memset(&s_obj, 0, sizeof(DATA_OBJECT));
// 	memset(&s_packet, 0, sizeof(SDO_PACKET));
// 	memset(&rx_frame, 0, sizeof(can_frame));

// 	cob = COB_SDO+NodeID;

// 	switch(TPDO_VAL)
// 	{
// 	case 1:
// 		s_obj.uint16Value[0] = TPDO1_INDEX;
// 		s_packet.info.type = WRITE_REQUEST_1BYTE;
// 		s_packet.info.index_low = s_obj.uint8Value[0];
// 		s_packet.info.index_high = s_obj.uint8Value[1];
// 		s_packet.info.subindex = OBJ_SUBINDEX_NULL;
// 		s_packet.info.data[0] = 0x00;
// 		break;
// 	case 2:
// 		s_obj.uint16Value[0] = TPDO2_INDEX;
// 		s_packet.info.type = WRITE_REQUEST_1BYTE;
// 		s_packet.info.index_low = s_obj.uint8Value[0];
// 		s_packet.info.index_high = s_obj.uint8Value[1];
// 		s_packet.info.subindex = OBJ_SUBINDEX_NULL;
// 		s_packet.info.data[0] = 0x00;
// 		break;
// 	case 3:
// 		s_obj.uint16Value[0] = TPDO3_INDEX;
// 		s_packet.info.type = WRITE_REQUEST_1BYTE;
// 		s_packet.info.index_low = s_obj.uint8Value[0];
// 		s_packet.info.index_high = s_obj.uint8Value[1];
// 		s_packet.info.subindex = OBJ_SUBINDEX_NULL;
// 		s_packet.info.data[0] = 0x00;
// 		break;
// 	case 4:
// 		s_obj.uint16Value[0] = TPDO4_INDEX;
// 		s_packet.info.type = WRITE_REQUEST_1BYTE;
// 		s_packet.info.index_low = s_obj.uint8Value[0];
// 		s_packet.info.index_high = s_obj.uint8Value[1];
// 		s_packet.info.subindex = OBJ_SUBINDEX_NULL;
// 		s_packet.info.data[0] = 0x00;
// 		break;
// 	default:
// 		break;
// 	}
// 	transmitter(cob, s_packet.value, 5);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// }

// void Elmo_CiA402::PDO_MAPPING_2(unsigned char NodeID)
// {
// 	NMT_STATE(NodeID, NMT_PREOP_MODE);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	PDO_STOP(NodeID, 2);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);
// 	// mapping objects
// 	// 2byte status word, 4byte actual position
// 	cob = COB_SDO+NodeID;

// 	s_obj.uint16Value[0] = TPDO2_INDEX;
// 	s_obj.uint16Value[1] = OBJ_STATUSWORD;

// 	s_packet.info.type = WRITE_REQUEST_4BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = 0x01;

// 	s_packet.info.data[0] = PDO_2BYTE;
// 	s_packet.info.data[1] = 0x00;
// 	s_packet.info.data[2] = s_obj.uint8Value[2];
// 	s_packet.info.data[3] = s_obj.uint8Value[3];
// 	res = transmitter(cob, s_packet.value, 8);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// 	s_obj.uint16Value[0] = TPDO2_INDEX;
// 	s_obj.uint16Value[1] = OBJ_POSITION_ACTUAL;

// 	s_packet.info.type = WRITE_REQUEST_4BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = 0x02;

// 	s_packet.info.data[0] = PDO_4BYTE;
// 	s_packet.info.data[1] = 0x00;
// 	s_packet.info.data[2] = s_obj.uint8Value[2];
// 	s_packet.info.data[3] = s_obj.uint8Value[3];
// 	transmitter(cob, s_packet.value, 8);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// 	// set sync(broadcast)
// 	s_obj.uint16Value[0] = TPDO2_INDEX2;

// 	s_packet.info.type = WRITE_REQUEST_1BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = 0x02;

// 	s_packet.info.data[0] = 0x01;
// 	transmitter(cob, s_packet.value, 5);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// 	// regist the object
// 	s_obj.uint16Value[0] = TPDO2_INDEX;
// 	s_packet.info.type = WRITE_REQUEST_1BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = OBJ_SUBINDEX_NULL;

// 	s_packet.info.data[0] = 0x02;
// 	transmitter(cob, s_packet.value, 5);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// }

// void Elmo_CiA402::PDO_MAPPING_2_1(unsigned char NodeID)
// {
// 	NMT_STATE(NodeID, NMT_PREOP_MODE);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	PDO_STOP(NodeID, 2);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);
// 	// mapping objects
// 	// 4byte actual position, 4byte actual velocity
// 	cob = COB_SDO+NodeID;

// 	s_obj.uint16Value[0] = TPDO2_INDEX;
// 	s_obj.uint16Value[1] = OBJ_POSITION_ACTUAL;

// 	s_packet.info.type = WRITE_REQUEST_4BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = 0x01;

// 	s_packet.info.data[0] = PDO_4BYTE;
// 	s_packet.info.data[1] = 0x00;
// 	s_packet.info.data[2] = s_obj.uint8Value[2];
// 	s_packet.info.data[3] = s_obj.uint8Value[3];
// 	res = transmitter(cob, s_packet.value, 8);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// 	s_obj.uint16Value[0] = TPDO2_INDEX;
// 	s_obj.uint16Value[1] = OBJ_VELOCITY_ACTUAL;

// 	s_packet.info.type = WRITE_REQUEST_4BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = 0x02;

// 	s_packet.info.data[0] = PDO_4BYTE;
// 	s_packet.info.data[1] = 0x00;
// 	s_packet.info.data[2] = s_obj.uint8Value[2];
// 	s_packet.info.data[3] = s_obj.uint8Value[3];
// 	transmitter(cob, s_packet.value, 8);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// 	// set sync(broadcast)
// 	s_obj.uint16Value[0] = TPDO2_INDEX2;

// 	s_packet.info.type = WRITE_REQUEST_1BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = 0x02;

// 	s_packet.info.data[0] = 0x01;
// 	transmitter(cob, s_packet.value, 5);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// 	// regist the object
// 	s_obj.uint16Value[0] = TPDO2_INDEX;
// 	s_packet.info.type = WRITE_REQUEST_1BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = OBJ_SUBINDEX_NULL;

// 	s_packet.info.data[0] = 0x02;
// 	transmitter(cob, s_packet.value, 5);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);
// }
// void Elmo_CiA402::PDO_MAPPING_3(unsigned char NodeID)
// {
// 	// mapping objects
// 	// 2byte status word, 2byte actual current, 4byte actual position
// 	cob = COB_SDO+NodeID;

// 	//pdo - statusword
// 	s_obj.uint16Value[0] = TPDO2_INDEX;
// 	s_obj.uint16Value[1] = OBJ_STATUSWORD;

// 	s_packet.info.type = WRITE_REQUEST_4BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = 0x01;

// 	s_packet.info.data[0] = PDO_2BYTE;
// 	s_packet.info.data[1] = 0x00;
// 	s_packet.info.data[2] = s_obj.uint8Value[2];
// 	s_packet.info.data[3] = s_obj.uint8Value[3];
// 	res = transmitter(cob, s_packet.value, 8);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// 	//pdo - actual current
// 	s_obj.uint16Value[0] = TPDO2_INDEX;
// 	s_obj.uint16Value[1] = OBJ_CURRENT_ACTUAL;

// 	s_packet.info.type = WRITE_REQUEST_2BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = 0x02;

// 	s_packet.info.data[0] = PDO_2BYTE;
// 	s_packet.info.data[1] = 0x00;
// 	s_packet.info.data[2] = s_obj.uint8Value[2];
// 	s_packet.info.data[3] = s_obj.uint8Value[3];
// 	res = transmitter(cob, s_packet.value, 8);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// 	//pdo - actual position
// 	s_obj.uint16Value[0] = TPDO2_INDEX;
// 	s_obj.uint16Value[1] = OBJ_POSITION_ACTUAL;

// 	s_packet.info.type = WRITE_REQUEST_4BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = 0x03;

// 	s_packet.info.data[0] = PDO_4BYTE;
// 	s_packet.info.data[1] = 0x00;
// 	s_packet.info.data[2] = s_obj.uint8Value[2];
// 	s_packet.info.data[3] = s_obj.uint8Value[3];
// 	transmitter(cob, s_packet.value, 8);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// 	// set sync(broadcast)
// 	s_obj.uint16Value[0] = TPDO2_INDEX2;

// 	s_packet.info.type = WRITE_REQUEST_1BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = 0x02;

// 	s_packet.info.data[0] = 0x01;
// 	transmitter(cob, s_packet.value, 5);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// 	// regist the object
// 	s_obj.uint16Value[0] = TPDO2_INDEX;
// 	s_packet.info.type = WRITE_REQUEST_1BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = OBJ_SUBINDEX_NULL;

// 	s_packet.info.data[0] = 0x03;
// 	transmitter(cob, s_packet.value, 5);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// }

// void Elmo_CiA402::RPDO2_MAPPING(unsigned char NodeID, unsigned short index)
// {
// 	cob = COB_SDO + NodeID;

// 	// rpdo2 - stop
// 	s_obj.uint16Value[0] = RPDO1_INDEX;
// 	s_packet.info.type = WRITE_REQUEST_1BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = OBJ_SUBINDEX_NULL;
// 	s_packet.info.data[0] = 0x00;
// 	res = transmitter(cob, s_packet.value, 8);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// 	//rpdo2 - target-torque
// 	s_obj.uint16Value[0] = RPDO1_INDEX;
// 	s_obj.uint16Value[1] = OBJ_TARGET_TORQUE;

// 	s_packet.info.type = WRITE_REQUEST_4BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = 0x01;

// 	s_packet.info.data[0] = PDO_2BYTE;
// 	s_packet.info.data[1] = 0x00;
// 	s_packet.info.data[2] = s_obj.uint8Value[2];
// 	s_packet.info.data[3] = s_obj.uint8Value[3];
// 	res = transmitter(cob, s_packet.value, 8);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// 	//rpdo2 - 1 mapping object
// 	s_obj.uint16Value[0] = RPDO1_INDEX;
// 	s_packet.info.type = WRITE_REQUEST_1BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = OBJ_SUBINDEX_NULL;
// 	s_packet.info.data[0] = 0x01;
// 	res = transmitter(cob, s_packet.value, 8);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);
// }

// void Elmo_CiA402::RPDO2_MAPPING(unsigned char NodeID, unsigned short index, unsigned short index2)
// {
// 	cob = COB_SDO + NodeID;

// 	// rpdo2 - stop
// 	s_obj.uint16Value[0] = RPDO1_INDEX;
// 	s_packet.info.type = WRITE_REQUEST_1BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = OBJ_SUBINDEX_NULL;
// 	s_packet.info.data[0] = 0x00;
// 	res = transmitter(cob, s_packet.value, 8);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// 	//rpdo2 - target-torque
// 	s_obj.uint16Value[0] = RPDO1_INDEX;
// 	s_obj.uint16Value[1] = OBJ_TARGET_TORQUE;

// 	s_packet.info.type = WRITE_REQUEST_4BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = 0x01;

// 	s_packet.info.data[0] = PDO_2BYTE;
// 	s_packet.info.data[1] = 0x00;
// 	s_packet.info.data[2] = s_obj.uint8Value[2];
// 	s_packet.info.data[3] = s_obj.uint8Value[3];
// 	res = transmitter(cob, s_packet.value, 8);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// 	//rpdo2 - controlword
// 	s_obj.uint16Value[0] = RPDO1_INDEX;
// 	s_obj.uint16Value[1] = OBJ_CONTROLWORD;

// 	s_packet.info.type = WRITE_REQUEST_4BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = 0x02;

// 	s_packet.info.data[0] = PDO_2BYTE;
// 	s_packet.info.data[1] = 0x00;
// 	s_packet.info.data[2] = s_obj.uint8Value[2];
// 	s_packet.info.data[3] = s_obj.uint8Value[3];
// 	res = transmitter(cob, s_packet.value, 8);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);

// 	//rpdo2 - 1 mapping object
// 	s_obj.uint16Value[0] = RPDO1_INDEX;
// 	s_packet.info.type = WRITE_REQUEST_1BYTE;
// 	s_packet.info.index_low = s_obj.uint8Value[0];
// 	s_packet.info.index_high = s_obj.uint8Value[1];
// 	s_packet.info.subindex = OBJ_SUBINDEX_NULL;
// 	s_packet.info.data[0] = 0x02;
// 	res = transmitter(cob, s_packet.value, 8);
// 	Print_CAN_FRAME(OBJ_WRITE);
// 	res = receiver(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
// 	Print_CAN_FRAME(OBJ_READ);
// }

// void Elmo_CiA402::RPDO2_SEND(unsigned char NodeID, short RPDO_VAL)
// {
// 	cob = COB_RPDO1 + NodeID;

// 	unsigned short tmp = (unsigned short)RPDO_VAL;

// 	s_packet.value[0] = tmp & 0x00FF;
// 	s_packet.value[1] = (tmp & 0xFF00) >> 8;

// 	s_packet.value[2] = 0x0f;
// 	s_packet.value[3] = 0x00;

// 	res = transmitter(cob, s_packet.value, 4);
// 	usleep(50);
// 	//Print_CAN_FRAME(OBJ_WRITE);
// }

// void Elmo_CiA402::SYNC(void)
// {
// 	cob = COB_SYNC;
// 	transmitter(cob, 0, 0);
// }

// void Elmo_CiA402::Elmo_STATE(int *d1, int *d2, int *d3)
// {
// 	SYNC();
// 	for(int i=0; i<6;++i)
// 		TPDO2_READ(d1, d2, d3);
// }


// void Elmo_CiA402::Print_CAN_FRAME(int type)
// {
// #if DEBUG_PRINT
// 	switch(type)
// 	{
// 	case OBJ_WRITE:
// 		printf("TX 0x%03x, %d\t", tx_frame.can_id, tx_frame.can_dlc);
// 		printf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
// 				tx_frame.data[0],
// 				tx_frame.data[1],
// 				tx_frame.data[2],
// 				tx_frame.data[3],
// 				tx_frame.data[4],
// 				tx_frame.data[5],
// 				tx_frame.data[6],
// 				tx_frame.data[7]);
// 		break;
// 	case OBJ_READ:
// 		printf("RX 0x%03x, %d\t", rx_frame.can_id, rx_frame.can_dlc);
// 		printf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
// 				rx_frame.data[0],
// 				rx_frame.data[1],
// 				rx_frame.data[2],
// 				rx_frame.data[3],
// 				rx_frame.data[4],
// 				rx_frame.data[5],
// 				rx_frame.data[6],
// 				rx_frame.data[7]);
// 		break;
// 	default:
// 		printf("Print error\n");
// 		break;
// 	}
// #endif
// }

// void Elmo_CiA402::TPDO2_READ(int *d1, int *d2, int *d3)
// {
// 	res = receiver(can_elmo.can_id, can_elmo.data, can_elmo.can_dlc);

// 	elmo_id = can_elmo.can_id & 0x00F;

// 	d1[elmo_id-1] = (int)(can_elmo.data[0] + (can_elmo.data[1]<<8));
// 	d2[elmo_id-1] = (short)(can_elmo.data[2] + (can_elmo.data[3]<<8));
// 	d3[elmo_id-1] = (int)(can_elmo.data[4] + (can_elmo.data[5]<<8)
// 			+ (can_elmo.data[6]<<16) + (can_elmo.data[7]<<24));
// }

// void Elmo_CiA402::TPDO2_READ(int *d1, int *d2)
// {
// 	res = receiver(can_elmo.can_id, can_elmo.data, can_elmo.can_dlc);

// 	elmo_id = can_elmo.can_id & 0x00F;

// 	d1[elmo_id-1] = (int)(can_elmo.data[0] + (can_elmo.data[1]<<8)
// 			+ (can_elmo.data[2]<<16) + (can_elmo.data[3]<<24));
// 	d2[elmo_id-1] = (int)(can_elmo.data[4] + (can_elmo.data[5]<<8)
// 			+ (can_elmo.data[6]<<16) + (can_elmo.data[7]<<24));
// }

// void Elmo_CiA402::elmo_activate(int ID)
// {
// 	SDO_CONTROLWORD(ID, OBJ_WRITE, SUB_OBJ_SHUTDOWN);
// 	usleep(10000);
// 	SDO_CONTROLWORD(ID, OBJ_WRITE, SUB_OBJ_SWITCHON);
// 	usleep(10000);
// 	SDO_CONTROLWORD(ID, OBJ_WRITE, SUB_OBJ_ENABLE_OPERATION);
// 	usleep(10000);
// }

// void Elmo_CiA402::elmo_deactivate(int ID)
// {

// 	SDO_CONTROLWORD(ID, OBJ_WRITE, SUB_OBJ_SHUTDOWN);
// 	usleep(10000);
// 	SDO_CONTROLWORD(ID, OBJ_WRITE, SUB_OBJ_DISABLE_VOLTAGE);
// 	usleep(10000);
// 	//NMT_STATE(ID, NMT_RESET_NODE );
// }

// void Elmo_CiA402::activate_all(char *ifname)
// {
// 	sock = rtsock_init(ifname);

// 	for(int i=1; i<=6; ++i)
// 	{
// 		NMT_STATE(i, NMT_PREOP_MODE);
// 		for(int j=1; j<4; ++j)
// 			PDO_STOP(i, j);

// 		PDO_MAPPING_3(i);
// 		//RPDO2_MAPPING(i, OBJ_TARGET_TORQUE);
// 		RPDO2_MAPPING(i,OBJ_TARGET_TORQUE, OBJ_CONTROLWORD );
// 		SDO_MODES_OPERTAION(i, OBJ_WRITE, TORQUE_MODE);

// 		elmo_activate(i);

// 	}

// 	NMT_STATE(0, NMT_START_NODE);
// }

// void Elmo_CiA402::deactivate_all(void)
// {
// 	for(int i=1; i<=6; ++i)
// 	{
// 		SDO_CONTROLWORD(i, OBJ_WRITE, SUB_OBJ_SHUTDOWN);
// 		usleep(10000);
// 		SDO_CONTROLWORD(i, OBJ_WRITE, SUB_OBJ_DISABLE_VOLTAGE);
// 		usleep(10000);
// 		//NMT_STATE(i, NMT_RESET_NODE );
// 	}

// 	deactivate();
// }
