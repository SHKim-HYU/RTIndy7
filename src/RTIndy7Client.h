/*
 * RTIndy7Client.h
 *
 *  Created on: 2023. 06. 06.
 *      Author: Sunhong Kim
 */

#ifndef RTINDY7CLIENT_H_
#define RTINDY7CLIENT_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <assert.h>
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
#include <Eigen/Dense>
#include <stdexcept>

//-xenomai-///////////////////////////////////////////////////////////////
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <rtdk.h>		//The rdtk real-time printing library
/****************************************************************************/

/////////////////////////////////////////////////////////////
#ifdef __CASADI__
// #include <dlfcn.h>

// typedef long long int casadi_int;
// typedef int (*eval_t)(const double**, double**, casadi_int*, double*, int);

#include "CS_Indy7.h"
#endif
////////////////////////////////////////////////////////////
#ifdef __BULLET__
#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/PhysicsClientSharedMemory_C_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3HashMap.h"

#include "Utils/b3Clock.h"
#include <map>
#include <vector>
#include "bullet_Indy7.h"
#endif
////////////////////////////////////////////////////////////


#include "Ecat_Master.h"

#include "PropertyDefinition.h"

#include "ServoAxis.h"  // For Indy7 interface
// #include "MR_Indy7.h"

using namespace std;

#ifdef __CB__
#define NUM_IO_MODULE 	1
#else
#define NUM_IO_MODULE 	0
#endif
#define NUM_TOOL 		1
#ifdef __RP__
#define NUM_AXIS		7
#else
#define NUM_AXIS		6
#endif
#define NUM_SLAVES (NUM_IO_MODULE+NUM_AXIS+NUM_TOOL)		//Modify this number to indicate the actual number of motor on the network

#ifndef PI
#define PI	(3.14159265359)
#define PI2	(6.28318530718)
#endif

#define WAKEUP_TIME				(5)	// wake up timeout before really run, in second
#define NSEC_PER_SEC 			1000000000

// Cycle time in nanosecond
unsigned int cycle_ns = 1000000; /* 1 ms */

typedef unsigned int UINT32;
typedef int32_t INT32;
typedef int16_t INT16;
typedef uint16_t UINT16;
typedef uint8_t UINT8;
typedef int8_t INT8;

// For RT thread management
static int run = 1;

unsigned long periodCycle = 0, worstCycle = 0;
unsigned long periodCompute = 0, worstCompute = 0;
unsigned long periodEcat = 0, worstEcat = 0;
unsigned long periodBuffer = 0, worstBuffer = 0;
unsigned int overruns = 0;
#ifdef __POCO__
unsigned long periodPoco = 0;
#endif
#ifdef __BULLET__
unsigned long periodBullet = 0;
#endif
#ifdef __CASADI__
unsigned long periodIndysim = 0;
#endif


////////// LOGGING BUFFER ///////////////
#define MAX_BUFF_SIZE 		1000

#define FT_START_DEVICE 	0x0000000B
#define FT_STOP_DEVICE 		0x0000000C

#define FT_SET_FILTER_500 	0x00010108
#define FT_SET_FILTER_300 	0x00020108
#define FT_SET_FILTER_200 	0x00030108
#define FT_SET_FILTER_150 	0x00040108
#define FT_SET_FILTER_100 	0x00050108
#define FT_SET_FILTER_50 	0x00060108
#define FT_SET_FILTER_40 	0x00070108
#define FT_SET_FILTER_30 	0x00080108
#define FT_SET_FILTER_20 	0x00090108
#define FT_SET_FILTER_10 	0x000A0108

#define FT_SET_BIAS 		0x00000111
#define FT_UNSET_BIAS 		0x00000011

unsigned int frontIdx = 0, rearIdx = 0;
/////////////////////////////////////////

// EtherCAT System interface object
NRMK_Master nrmk_master;

// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;

// Global time (beginning from zero)
double gt=0;

// Trajectory parameers
double traj_time=0;
int motion=-1;

/// TO DO: This is user-code.
double sine_amp=50000, f=0.2, period;

int InitFlag[NUM_AXIS] = {0,};

////////////// TxPDO //////////////
// Drive
UINT16	StatusWord[NUM_SLAVES] = {0,};
INT32 	ActualPos[NUM_SLAVES] = {0,};
INT32 	ActualVel[NUM_SLAVES] = {0,};
INT16 	ActualTor[NUM_SLAVES] = {0,};
UINT32	DataIn[NUM_SLAVES] = {0,};
UINT8	ModeOfOperationDisplay[NUM_SLAVES] = {0,};
// IO
#ifdef __CB__
UINT8	StatusCode[NUM_SLAVES] = {0,};
UINT8	DI5V[NUM_SLAVES] = {0,};
UINT8	DI1[NUM_SLAVES] = {0,};
UINT8	DI2[NUM_SLAVES] = {0,};
UINT16	AI1[NUM_SLAVES] = {0,};
UINT16	AI2[NUM_SLAVES] = {0,};
INT16	FTRawFxCB[NUM_SLAVES] = {0,};
INT16	FTRawFyCB[NUM_SLAVES] = {0,};
INT16	FTRawFzCB[NUM_SLAVES] = {0,};
INT16	FTRawTxCB[NUM_SLAVES] = {0,};
INT16	FTRawTyCB[NUM_SLAVES] = {0,};
INT16	FTRawTzCB[NUM_SLAVES] = {0,};
UINT8	FTOverloadStatusCB[NUM_SLAVES] = {0,};
UINT8	FTErrorFlagCB[NUM_SLAVES] = {0,};
UINT8	RS485RxCnt[NUM_SLAVES] = {0,};
UINT8	RS485RxD0[NUM_SLAVES] = {0,};
UINT8	RS485RxD1[NUM_SLAVES] = {0,};
UINT8	RS485RxD2[NUM_SLAVES] = {0,};
UINT8	RS485RxD3[NUM_SLAVES] = {0,};
UINT8	RS485RxD4[NUM_SLAVES] = {0,};
UINT8	RS485RxD5[NUM_SLAVES] = {0,};
UINT8	RS485RxD6[NUM_SLAVES] = {0,};
UINT8	RS485RxD7[NUM_SLAVES] = {0,};
UINT8	RS485RxD8[NUM_SLAVES] = {0,};
UINT8	RS485RxD9[NUM_SLAVES] = {0,};
#endif
// Tool
UINT8	IStatus[NUM_SLAVES] = {0,};
UINT8	IButton[NUM_SLAVES] = {0,};
INT16	FTRawFx[NUM_SLAVES] = {0,};
INT16	FTRawFy[NUM_SLAVES] = {0,};
INT16	FTRawFz[NUM_SLAVES] = {0,};
INT16	FTRawTx[NUM_SLAVES] = {0,};
INT16	FTRawTy[NUM_SLAVES] = {0,};
INT16	FTRawTz[NUM_SLAVES] = {0,};
UINT8	FTOverloadStatus[NUM_SLAVES] = {0,};
UINT8	FTErrorFlag[NUM_SLAVES] = {0,};

////////////// RxPDO //////////////
// Drive
INT32 	TargetPos[NUM_SLAVES] = {0,};
INT32 	TargetVel[NUM_SLAVES] = {0,};
INT16 	TargetTor[NUM_SLAVES] = {0,};
UINT8 	ModeOfOperation[NUM_SLAVES] = {0,};
UINT16	Controlword[NUM_SLAVES] = {0,};

// IO
#ifdef __CB__
UINT8	ControlCode[NUM_SLAVES] = {0,};
UINT8	DO5V[NUM_SLAVES] = {0,};
UINT8	TO[NUM_SLAVES] = {0,};
UINT8	DO[NUM_SLAVES] = {0,};
UINT16	AO1[NUM_SLAVES] = {0,};
UINT16	AO2[NUM_SLAVES] = {0,};
UINT32	FTConfigParamCB[NUM_SLAVES] = {0,};
UINT8	RS485ConfigParam[NUM_SLAVES] = {0,};
UINT8	RS485CMD[NUM_SLAVES] = {0,};
UINT8	RS485TxCnt[NUM_SLAVES] = {0,};
UINT8	RS485TxD0[NUM_SLAVES] = {0,};
UINT8	RS485TxD1[NUM_SLAVES] = {0,};
UINT8	RS485TxD2[NUM_SLAVES] = {0,};
UINT8	RS485TxD3[NUM_SLAVES] = {0,};
UINT8	RS485TxD4[NUM_SLAVES] = {0,};
UINT8	RS485TxD5[NUM_SLAVES] = {0,};
UINT8	RS485TxD6[NUM_SLAVES] = {0,};
UINT8	RS485TxD7[NUM_SLAVES] = {0,};
UINT8	RS485TxD8[NUM_SLAVES] = {0,};
UINT8	RS485TxD9[NUM_SLAVES] = {0,};
#endif
// Tool
UINT8	ILed[NUM_SLAVES] = {0,};
UINT8	IGripper[NUM_SLAVES] = {0,};
UINT32	FTConfigParam[NUM_SLAVES] = {0,};
UINT8	LEDMode[NUM_SLAVES] = {0,};
UINT8	LEDG[NUM_SLAVES] = {0,};
UINT8	LEDR[NUM_SLAVES] = {0,};
UINT8	LEDB[NUM_SLAVES] = {0,};

// Robotous FT EtherCAT
union DeviceConfig
{
	uint8_t u8Param[4];
	uint32_t u32Param;
};
double force_divider =50.0;
double torque_divider = 2000.0;

// Axis for CORE
#ifdef __RP__
const int 	 zeroPos[NUM_AXIS] = {ZERO_POS_1,ZERO_POS_2,ZERO_POS_3,ZERO_POS_4,ZERO_POS_5,ZERO_POS_6,ZERO_POS_7};
const int 	 gearRatio[NUM_AXIS] = {GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_101,GEAR_RATIO_101,GEAR_RATIO_101};
const int 	 TauADC[NUM_AXIS] = {TORQUE_ADC_500,TORQUE_ADC_500,TORQUE_ADC_200,TORQUE_ADC_200,TORQUE_ADC_100,TORQUE_ADC_100,TORQUE_ADC_100};
const double TauK[NUM_AXIS] = {TORQUE_CONST_500,TORQUE_CONST_500,TORQUE_CONST_200,TORQUE_CONST_200,TORQUE_CONST_100,TORQUE_CONST_100,TORQUE_CONST_100};
const int 	 dirQ[NUM_AXIS] = {-1,-1,-1,1,-1,-1,-1};
const int 	 dirTau[NUM_AXIS] = {-1,-1,-1,1,-1,-1,-1};
#else
const int 	 zeroPos[NUM_AXIS] = {ZERO_POS_1,ZERO_POS_2,ZERO_POS_3,ZERO_POS_4,ZERO_POS_5,ZERO_POS_6};
const int 	 gearRatio[NUM_AXIS] = {GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_101,GEAR_RATIO_101,GEAR_RATIO_101};
const int 	 TauADC[NUM_AXIS] = {TORQUE_ADC_500,TORQUE_ADC_500,TORQUE_ADC_200,TORQUE_ADC_100,TORQUE_ADC_100,TORQUE_ADC_100};
const double TauK[NUM_AXIS] = {TORQUE_CONST_500,TORQUE_CONST_500,TORQUE_CONST_200,TORQUE_CONST_100,TORQUE_CONST_100,TORQUE_CONST_100};
const int 	 dirQ[NUM_AXIS] = {-1,-1,1,-1,-1,1};
const int 	 dirTau[NUM_AXIS] = {-1,-1,1,-1,-1,1};
#endif




//////////////////////////

// Interface to physical axes
NRMKHelper::ServoAxis Axis[NUM_AXIS];

struct LOGGING_PACK
{
	double Time;
	INT32 	ActualPos[JOINTNUM];
	INT32 	ActualVel[JOINTNUM];
};

typedef struct STATE{
	JVec q;
	JVec q_dot;
	JVec q_ddot;
	JVec tau;
	JVec tau_ext;
	JVec G;

	Vector6d x;                           //Task space
	Vector6d x_dot;
	Vector6d x_ddot;
	Vector6d F;
	Vector6d F_CB;

    double s_time;
}state;

typedef struct JOINT_INFO{
	int Position;
	int aq_inc[NUM_AXIS];
	int atoq_per[NUM_AXIS];
	short dtor_per[NUM_AXIS];
	int statusword[NUM_AXIS];

	JVec q_target;
	JVec qdot_target;
	JVec qddot_target;
	JVec traj_time;

	STATE act;
	STATE des;
	STATE nom;

}JointInfo;

JVec MAX_TORQUES;

JMat Hinf_Kp;
JMat Hinf_Kv;
JMat Hinf_Ki;
JMat Hinf_K_gamma;

#ifdef __BULLET__
extern const int CONTROL_RATE;
const int CONTROL_RATE = 100;

// Bullet globals
const b3Scalar FIXED_TIMESTEP = 1.0 / ((b3Scalar)CONTROL_RATE);
b3SharedMemoryCommandHandle command;
int statusType, ret;

#endif

#endif /* RTINDY7CLIENT_H_ */
