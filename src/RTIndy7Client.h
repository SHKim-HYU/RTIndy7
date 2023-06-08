/*
 * RTIndy7Client.h
 *
 *  Created on: 2023. 06. 06.
 *      Author: Sunhong Kim
 */

#ifndef RTINDY7CLIENT_H_
#define RTINDY7CLIENT_H_

//#define USE_MR
//#define USE_MR_KDL
//#define USE_NRMK_SDK
//#define USE_CASADI

/////////////////////////////////////////////////////////////

#include <stdio.h>
#include <dlfcn.h>
#include <iostream>
#include <chrono>


using namespace std;
using namespace chrono;

typedef long long int casadi_int;
typedef int (*eval_t)(const double**, double**, casadi_int*, double*, int);


////////////////////////////////////////////////////////////


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
#include <Eigen/Dense>

//-xenomai-///////////////////////////////////////////////////////////////
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <rtdk.h>		//The rdtk real-time printing library
/****************************************************************************/

#include "Ecat_Master.h"

#include "PropertyDefinition.h"

#include "ServoAxis.h"  // For Indy7 interface

// #include "casadi/casadi.hpp"

using namespace std;
// using namespace casadi;


#define NUM_IO_MODULE 	1
#define NUM_TOOL 		1
#define NUM_AXIS		6
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


typedef Eigen::Matrix<double, JOINTNUM, 1> JVec;
typedef Eigen::Matrix<double, 4, 4> SE3;
typedef Eigen::Matrix<double, 3, 3> SO3;
typedef Eigen::Matrix<double, 4, 4> se3;
typedef Eigen::Matrix<double, 3, 3> so3;
typedef Eigen::Matrix<double, 6, JOINTNUM> ScrewList;
typedef Eigen::Matrix<double, 6, JOINTNUM> Jacobian;
typedef Eigen::Matrix<double, JOINTNUM,6 > pinvJacobian;
typedef Eigen::Matrix<double, 6*JOINTNUM, JOINTNUM> DerivativeJacobianVec;
typedef Eigen::Matrix<double, 6*JOINTNUM, 1> vecJVec;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<float, 6, 1> Vector6f;   
typedef Eigen::Matrix<double, 3, 1> Vector3d;   
typedef Eigen::Matrix<double, 4, 1> Vector4d;  
typedef Eigen::Matrix<double, 6, 6> Matrix6d;  
typedef Eigen::Matrix<double, 3, 3> Matrix3d;  
typedef Eigen::Matrix<double, 6, JOINTNUM> Matrix6xn;
typedef Eigen::Matrix<double, 6, JOINTNUM+1> Matrix6xn_1;
typedef Eigen::Matrix<double, JOINTNUM, JOINTNUM> MassMat;
////////// LOGGING BUFFER ///////////////
#define MAX_BUFF_SIZE 		1000

#define FT_READ_ONCE 0x0A
#define FT_START_DEVICE 0x0B
#define FT_STOP_DEVICE 0x0C
#define FT_SET_OUTPUT_RATE 0x0F
#define FT_GET_OUTPUT_RATE 0x10
#define FT_SET_BIAS 0x11
#define FT_BIAS_SUB 0x01
#define FT_UNBIAS_SUB 0x00

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
int motion=1;

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
float force_divider =50.0;
float torque_divider = 2000.0;

// Axis for CORE
const int 	 zeroPos[NUM_AXIS] = {ZERO_POS_1,ZERO_POS_2,ZERO_POS_3,ZERO_POS_4,ZERO_POS_5,ZERO_POS_6};
const int 	 gearRatio[NUM_AXIS] = {GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_101,GEAR_RATIO_101,GEAR_RATIO_101};
const int 	 TauADC[NUM_AXIS] = {TORQUE_ADC_500,TORQUE_ADC_500,TORQUE_ADC_200,TORQUE_ADC_100,TORQUE_ADC_100,TORQUE_ADC_100};
const double TauK[NUM_AXIS] = {TORQUE_CONST_500,TORQUE_CONST_500,TORQUE_CONST_200,TORQUE_CONST_100,TORQUE_CONST_100,TORQUE_CONST_100};
const int 	 dirQ[NUM_AXIS] = {-1,-1,1,-1,-1,1};
const int 	 dirTau[NUM_AXIS] = {-1,-1,1,-1,-1,1};


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

	Vector6d x;                           //Task space
	Vector6d x_dot;
	Vector6d x_ddot;
	Vector6f F;
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

#endif /* RTINDY7CLIENT_H_ */
