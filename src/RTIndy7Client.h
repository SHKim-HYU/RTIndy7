/*
 * RTRArmClient.h
 *
 *  Created on: 2018. 11. 28.
 *      Author: Administrator
 */

#ifndef RTRARMCLIENT_H_
#define RTRARMCLIENT_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h>		// string function definitions
#include <fcntl.h>		// File control definitions
#include <errno.h>		// Error number definitions
#include <termios.h>	// POSIX terminal control definitions
#include <time.h>		// time calls
#include <sys/ioctl.h>
#include <math.h>
#include <string>
#include "iostream"
#include <fstream>
#include <sstream>

//-xenomai-///////////////////////////////////////////////////////////////
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <rtdk.h>		//The rdtk real-time printing library
/****************************************************************************/

//#include "EcatSystem/SystemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.h"
#include "Ecat_NRMK_Indy_Tool.h"
#include "Ecat_Master.h"
#include "Ecat_Elmo.h"

#include "Trajectory.h"
#include "Controller.h"
#include "SerialRobot.h"
#include "PoEKinematics.h"
#include "PropertyDefinition.h"

using namespace std;


#define NUM_AXIS	6	//Modify this number to indicate the actual number of motor on the network
#ifndef PI
#define PI	(3.14159265359)
#define PI2	(6.28318530718)
#endif

#define WAKEUP_TIME				(5)	// wake up timeout before really run, in second
#define NSEC_PER_SEC 			1000000000

typedef unsigned int UINT32;
typedef int32_t INT32;
typedef int16_t INT16;
typedef uint16_t UINT16;
typedef uint8_t UINT8;
typedef int8_t INT8;


typedef struct JOINT_INFO{

	int Position;

	int aq_inc[NUM_AXIS];
	int atoq_per[NUM_AXIS];
	short dtor_per[NUM_AXIS];

	int statusword[NUM_AXIS];

	double* TargetTrajPos_Rad[NUM_AXIS];
	double TargetTrajTime[NUM_AXIS];

	STATE act;
	STATE des;
}JointInfo;

typedef union{
	struct{
		UINT8 reserve;
		UINT8 opt;
		UINT16 data;
	}info;
	UINT8 value[8];
}FTPacket;

// Cycle time in nanosecond
unsigned int cycle_ns = 1000000; /* 1 ms */
static int period = 1000000;

#endif /* RTRARMCLIENT_H_ */
