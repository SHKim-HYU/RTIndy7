/*
 * RoboLimb.cpp
 *
 *  Created on: 2019. 8. 2.
 *      Author: root
 */

#include "RoboLimb.h"



RoboLimb::RoboLimb(int _rtsercan_fd)
{
	rtsercan_fd=_rtsercan_fd;
}

int RoboLimb::isHandopen(void)
{
	if(open_flag)
		return 1;
	else
		return 0;
}

void RoboLimb::IdleHand(void)
{
	for(int i=1; i<=NUM_CLIENT; i++)
	{
		MoveFinger(i,HAND_IDLE);
	}
}

void RoboLimb::OpenHand(void)
{
	for(int i=1; i<NUM_CLIENT; i++)
	{
			MoveFinger(i,HAND_IDLE);
			MoveFinger(i,HAND_OPEN);
			//ReadFinger(i);
	}

}

void RoboLimb::CloseHand(void)
{
	for(int i=5; i>1; i--)
	{
			MoveFinger(i,HAND_IDLE);
			MoveFinger(i,HAND_CLOSE);
			//ReadFinger(i);
	}

	usleep(500000);
	MoveFinger(1,HAND_IDLE);
	MoveFinger(1,HAND_CLOSE);
}

void RoboLimb::MoveFinger(int num, int move)
{
	CAN_FRAME txframe;
	HandPacket txpacket;

	txframe.can_id = TX_ID + num;
	txframe.can_dlc = 4;
	txpacket.info.reserve = 0x00;
	switch(move)
	{
	case HAND_IDLE:
		txpacket.info.opt = HAND_IDLE;
		break;
	case HAND_OPEN:
		txpacket.info.opt = HAND_OPEN;
		break;
	case HAND_CLOSE:
		txpacket.info.opt = HAND_CLOSE;
		break;
	default:
		txpacket.info.opt = HAND_IDLE;
		break;
	}

	txpacket.info.data = 277;
	txframe.data[0] = txpacket.value[0];
	txframe.data[1] = txpacket.value[1];
	txframe.data[2] = txpacket.value[3];
	txframe.data[3] = txpacket.value[2];
	SERCAN_write(rtsercan_fd, txframe);
	//print_CANFrame(txframe);
	usleep(2000);

}

void RoboLimb::ReadFinger(int num)
{
	CAN_FRAME rxframe;
	HandPacket rxpacket;

	rxframe.can_id = RX_ID + num;
	rxframe.can_dlc = 4;
	rxpacket.info.reserve = 0x00;

	SERCAN_read(rtsercan_fd, &rxframe);
	//print_CANFrame(rxframe);

	if (rxframe.data[1] == HAND_CLOSE_STALL)
	{
		finger_state[num-1]=HAND_CLOSE_STALL;
	}

	else if (rxframe.data[1] == HAND_OPEN_STALL)
	{
		finger_state[num-1]=HAND_OPEN_STALL;
	}
}
