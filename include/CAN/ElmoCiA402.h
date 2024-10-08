// /*
//  * [WIP]ElmoCiA402.h
//  *
//  *  Created on: Dec 6, 2017
//  *      Author: spec
//  */

// #ifndef ELMOCIA402_H_
// #define ELMOCIA402_H_

// #include "PCANDevice.h"
// #include "CiA402_Def.h"

// #define DEBUG_PRINT 0



// class Elmo_CiA402: public PCANDevice {
// public:
// 	Elmo_CiA402();
// 	virtual ~Elmo_CiA402();

// 	void SDO_CONTROLWORD(int NodeID, int RW, unsigned char data);
// 	void SDO_MODES_OPERTAION(unsigned char NodeID, int RW, unsigned char data);
// 	void SDO_TARGET_TORQUE(unsigned char NodeID, int val);
// 	int SDO_RATE_CURRENT(unsigned char NodeID);
// 	int SDO_RECIEVE(void);

// 	void NMT_STATE(unsigned char NodeID, unsigned char data);

// 	void PDO_STOP(unsigned char NodeID, unsigned char TPDO_VAL);
// 	void PDO_MAPPING_2(unsigned char NodeID);
// 	void PDO_MAPPING_2_1(unsigned char NodeID);
// 	void PDO_MAPPING_3(unsigned char NodeID);
// 	void RPDO2_MAPPING(unsigned char NodeID, unsigned short index);
// 	void RPDO2_MAPPING(unsigned char NodeID, unsigned short index, unsigned short index2);
// 	void RPDO2_SEND(unsigned char NodeID, short RPDO_VAL);

// 	void SYNC(void);
// 	void Elmo_STATE(int *d1, int *d2, int *d3);

// 	void Print_CAN_FRAME(int type);
// 	void TPDO2_READ(int *d1, int *d2, int *d3);
// 	void TPDO2_READ(int *d1, int *d2);
// 	void elmo_activate(int ID);
// 	void elmo_deactivate(int ID);

// 	void activate_all(char *ifname);
// 	void deactivate_all(void);
// private:
// 	int res;
// 	uint32_t cob;
// 	SDO_PACKET s_packet;
// 	DATA_OBJECT s_obj;
// 	int elmo_id;
// 	unsigned char elmo_c[4], elmo_p[4];
// 	struct can_frame can_elmo;
// };

// #endif /* ELMOCIA402_H_ */
