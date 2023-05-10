/*
 * RoboLimb.h
 *
 *  Created on: 2019. 8. 2.
 *      Author: Administrator
 */

#ifndef ROBOLIMB_H_
#define ROBOLIMB_H_

#include <stdio.h>
#include <signal.h>
#include <cstdlib>
#include <unistd.h>
#include <errno.h>
#include <getopt.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>

#include "NRMKsercan_tp.h"
#include "NRMKhw_tp.h"

#include "can_define.h"

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;

typedef union{
	struct{
		uint8 reserve;
		uint8 opt;
		uint16 data;
	}info;
	uint8 value[8];
}HandPacket;


class RoboLimb
{
public:
	RoboLimb(int _rtsercan_fd);
	~RoboLimb(){};

	int isHandopen(void);
	void IdleHand(void);
	void OpenHand(void);
	void CloseHand(void);
	void MoveFinger(int num, int move);
	void ReadFinger(int num);

private:
	int rtsercan_fd;
	int finger_state[NUM_CLIENT]={0,};
	int open_flag;

};


#endif
