/*
 * RTRArmClient.h
 *
 *  Created on: 2018. 11. 28.
 *      Author: Administrator
 */

#ifndef RTRARMCLIENT_H_
#define RTRARMCLIENT_H_

/***** License Information *****/
#define USERNAME "USER"
#define EMAIL "user@email.com"
#define SERIAL "ABCD"
/******************************/

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

#include "EcatDataSocket/EcatDataSocket.h"
#include "EcatDataSocket/EcatControlSocket.h"
//#include "EcatSystem/SystemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.h"
#include "EcatSystem/Ecat_NRMK_Indy_Tool.h"
#include "EcatSystem/Ecat_Master.h"
#include "EcatSystem/Ecat_Elmo.h"


#include "PropertyDefinition.h"
#include "NRMKsercan_tp.h"
#include "NRMKhw_tp.h"


// TCP-Server Communication
//#include "TCP/RTTCP.h"
#include "Poco/Net/TCPServer.h"
#include "Poco/Net/TCPServerConnection.h"
#include "Poco/Net/TCPServerConnectionFactory.h"
#include "Poco/Thread.h"

using Poco::Net::ServerSocket;
using Poco::Net::StreamSocket;
using Poco::Net::TCPServerConnection;
using Poco::Net::TCPServerConnectionFactory;
using Poco::Net::TCPServer;
using Poco::Timestamp;
using Poco::Thread;

const Poco::UInt16 SERVER_PORT = 9911;

using namespace std;

#include "CAN/can_define.h"
#include "CAN/RoboLimb.h"


#include "ServoAxis.h"

#define NUM_IO_MODULE 	1
#define NUM_TOOL 		1
#define NUM_AXIS		6		//Modify this number to indicate the actual number of motor on the network

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

unsigned long fault_count=0;
long ethercat_time=0, worst_time=0;

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
typedef Eigen::Matrix<double, 3, 1> Vector3d;   
typedef Eigen::Matrix<double, 4, 1> Vector4d;  
typedef Eigen::Matrix<double, 6, 6> Matrix6d;  
typedef Eigen::Matrix<double, 3, 3> Matrix3d;  
typedef Eigen::Matrix<double, 6, JOINTNUM> Matrix6xn;
typedef Eigen::Matrix<double, 6, JOINTNUM+1> Matrix6xn_1;
typedef Eigen::Matrix<double, JOINTNUM, JOINTNUM> MassMat;
////////// LOGGING BUFFER ///////////////
#define MAX_BUFF_SIZE 		1000

unsigned int frontIdx = 0, rearIdx = 0;
/////////////////////////////////////////

// NRMKDataSocket for plotting axes data in Data Scope
EcatDataSocket datasocket;

// EtherCAT System interface object
NRMK_Master nrmk_master;

// NRMK socket for online commands from NRMK EtherLab Configuration Tool
NRMKHelper::EcatControlSocket<NUM_AXIS> guicontrolsocket;

// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;

// Global time (beginning from zero)
double gt=0;

/// TO DO: This is user-code.
double sine_amp=50000, f=0.2, period;

int InitFlag[NUM_AXIS] = {0,};

// EtherCAT Data (in pulse)
INT32 	ZeroPos[NUM_AXIS] = {0,};
// Drive
UINT16	StatusWord[NUM_AXIS] = {0,};
INT32 	ActualPos[NUM_AXIS] = {0,};
INT32 	ActualVel[NUM_AXIS] = {0,};
INT16 	ActualTor[NUM_AXIS] = {0,};
UINT32	DataIn[NUM_AXIS] = {0,};
UINT8	ModeOfOperationDisplay[NUM_AXIS] = {0,};
// IO
UINT8	StatusCode[NUM_IO_MODULE] = {0,};
UINT8	DI5V[NUM_IO_MODULE] = {0,};
UINT8	DI1[NUM_IO_MODULE] = {0,};
UINT8	DI2[NUM_IO_MODULE] = {0,};
UINT16	AI1[NUM_IO_MODULE] = {0,};
UINT16	AI2[NUM_IO_MODULE] = {0,};
INT16	FTRawFxCB[NUM_IO_MODULE] = {0,};
INT16	FTRawFyCB[NUM_IO_MODULE] = {0,};
INT16	FTRawFzCB[NUM_IO_MODULE] = {0,};
INT16	FTRawTxCB[NUM_IO_MODULE] = {0,};
INT16	FTRawTyCB[NUM_IO_MODULE] = {0,};
INT16	FTRawTzCB[NUM_IO_MODULE] = {0,};
UINT8	FTOverloadStatusCB[NUM_IO_MODULE] = {0,};
UINT8	FTErrorFlagCB[NUM_IO_MODULE] = {0,};
UINT8	RS485RxCnt[NUM_IO_MODULE] = {0,};
UINT8	RS485RxD0[NUM_IO_MODULE] = {0,};
UINT8	RS485RxD1[NUM_IO_MODULE] = {0,};
UINT8	RS485RxD2[NUM_IO_MODULE] = {0,};
UINT8	RS485RxD3[NUM_IO_MODULE] = {0,};
UINT8	RS485RxD4[NUM_IO_MODULE] = {0,};
UINT8	RS485RxD5[NUM_IO_MODULE] = {0,};
UINT8	RS485RxD6[NUM_IO_MODULE] = {0,};
UINT8	RS485RxD7[NUM_IO_MODULE] = {0,};
UINT8	RS485RxD8[NUM_IO_MODULE] = {0,};
UINT8	RS485RxD9[NUM_IO_MODULE] = {0,};
// Tool
UINT8	IStatus[NUM_TOOL] = {0,};
UINT8	IButton[NUM_TOOL] = {0,};
INT16	FTRawFx[NUM_TOOL] = {0,};
INT16	FTRawFy[NUM_TOOL] = {0,};
INT16	FTRawFz[NUM_TOOL] = {0,};
INT16	FTRawTx[NUM_TOOL] = {0,};
INT16	FTRawTy[NUM_TOOL] = {0,};
INT16	FTRawTz[NUM_TOOL] = {0,};
UINT8	FTOverloadStatus[NUM_TOOL] = {0,};
UINT8	FTErrorFlag[NUM_TOOL] = {0,};

// Drive
INT32 	TargetPos[NUM_AXIS] = {0,};
INT32 	TargetVel[NUM_AXIS] = {0,};
INT16 	TargetTor[NUM_AXIS] = {0,};
UINT8 	ModeOfOperation[NUM_AXIS] = {0,};
UINT16	Controlword[NUM_AXIS] = {0,};

// IO
UINT8	ControlCode[NUM_IO_MODULE] = {0,};
UINT8	DO5V[NUM_IO_MODULE] = {0,};
UINT8	TO[NUM_IO_MODULE] = {0,};
UINT8	DO[NUM_IO_MODULE] = {0,};
UINT16	AO1[NUM_IO_MODULE] = {0,};
UINT16	AO2[NUM_IO_MODULE] = {0,};
UINT32	FTConfigParamCB[NUM_IO_MODULE] = {0,};
UINT8	RS485ConfigParam[NUM_IO_MODULE] = {0,};
UINT8	RS485CMD[NUM_IO_MODULE] = {0,};
UINT8	RS485TxCnt[NUM_IO_MODULE] = {0,};
UINT8	RS485TxD0[NUM_IO_MODULE] = {0,};
UINT8	RS485TxD1[NUM_IO_MODULE] = {0,};
UINT8	RS485TxD2[NUM_IO_MODULE] = {0,};
UINT8	RS485TxD3[NUM_IO_MODULE] = {0,};
UINT8	RS485TxD4[NUM_IO_MODULE] = {0,};
UINT8	RS485TxD5[NUM_IO_MODULE] = {0,};
UINT8	RS485TxD6[NUM_IO_MODULE] = {0,};
UINT8	RS485TxD7[NUM_IO_MODULE] = {0,};
UINT8	RS485TxD8[NUM_IO_MODULE] = {0,};
UINT8	RS485TxD9[NUM_IO_MODULE] = {0,};

// Tool
UINT8	ILed[NUM_TOOL] = {0,};
UINT8	IGripper[NUM_TOOL] = {0,};
UINT32	FTConfigParam[NUM_TOOL] = {0,};
UINT8	LEDMode[NUM_TOOL] = {0,};
UINT8	LEDG[NUM_TOOL] = {0,};
UINT8	LEDR[NUM_TOOL] = {0,};
UINT8	LEDB[NUM_TOOL] = {0,};


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
	JVec dq;
	JVec ddq;
	JVec torque;

	Vector6d x;                           //Task space
	Vector6d x_dot;
	Vector6d x_ddot;
    double s_time;
}state;

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

// Cycle time in nanosecond
unsigned int cycle_ns = (unsigned int)(1000.0/CONTROL_FREQ*1000000.0); /* 1 ms */
double dt = 1.0/CONTROL_FREQ; /* 1 ms */
double wc = 105.0; /* CUT OFF FREQUENCY*/
JVec MAX_TORQUES;
static int period = 1000000;
int traj_flag = 0;
#endif /* RTRARMCLIENT_H_ */
