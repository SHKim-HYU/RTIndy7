/*
 * can_define.h
 *
 *  Created on: 2018. 7. 12.
 *      Author: Administrator
 */

#ifndef CAN_DEFINE_H_
#define CAN_DEFINE_H_

#define NUM_CLIENT 6

#define TX_ID 0x272
#define RX_ID 0x200
#define QUICK_GRIP_ID 0x301

#define HAND_IDLE 	0x00
#define HAND_CLOSE 	0x01
#define HAND_OPEN 	0x02
#define HAND_CLOSE_STALL 0x03
#define HAND_OPEN_STALL 0x04

#define GRIP_PINCH 0x01
#define GRIP_LATERAL 0x05
#define GRIP_TRIPOD 0x0B
#define GRIP_POWER 0x07
#define GRIP_COVER 0x18

#endif /* CAN_DEFINE_H_ */
