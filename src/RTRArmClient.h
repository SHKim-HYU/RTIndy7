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

#define NUM_FT	 	1
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

// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;

// Global time (beginning from zero)
double gt=0;
double double_gt=0;

// EtherCAT Data (in pulse)
INT32 	ZeroPos[JOINTNUM] = {0,};
UINT16	StatusWord[JOINTNUM] = {0,};
INT32 	ECAT_ActualPos[JOINTNUM] = {0,};
INT32	ECAT_ActualPos_Old[JOINTNUM] = {0,};
INT32 	ECAT_ActualVel[JOINTNUM] = {0,};

INT32	ECAT_ActualAcc[JOINTNUM] = {0,};
INT32	ECAT_ActualVel_Old[JOINTNUM] = {0,};
INT32	ECAT_ActualAcc_Old[JOINTNUM] = {0,};
INT16 	ECAT_ActualTor[JOINTNUM] = {0,};
UINT32	DataIn[JOINTNUM] = {0,};
INT8	ModeOfOperationDisplay[JOINTNUM] = {0,};
INT8	DeviceState[JOINTNUM] = {0,};
UINT32	DigitalInput[JOINTNUM] = {0,};
INT32   HomePos[JOINTNUM]={0, 0, 0, -6553600, 0, 0};
UINT32 	DataOut[JOINTNUM] = {0,};
INT8 	ModeOfOperation[JOINTNUM] = {0,};
UINT16	ControlWord[JOINTNUM] = {0,};
INT32	VelocityOffset[JOINTNUM] = {0,};
INT16	TorqueOffset[JOINTNUM] = {0,};
UINT32	DigitalOutput[JOINTNUM] = {0,};

UINT8   iLed              = 0;       // write
UINT8   iGripper          = 0; 		// write
UINT32  FT_configparam    = 0; 		// write
UINT8   LED_mode          = 0; 		// write (max torque (max current) = 1000)
UINT8   LED_G             = 0; 		// write (use enum ModeOfOperation for convenience)
UINT8   LED_R             = 0;       // write (use enum ModeOfOperation for convenience)
UINT8   LED_B             = 0;       // write (use enum ModeOfOperation for convenience)

UINT8   iStatus           = 0;       // read
UINT32  iButton           = 0; 		// read
INT16  	FT_Raw_Fx         = 0;       // read
INT16  	FT_Raw_Fy         = 0;       // read
INT16  	FT_Raw_Fz         = 0;       // read
INT16  	FT_Raw_Tx         = 0;       // read
INT16  	FT_Raw_Ty         = 0;       // read
INT16  	FT_Raw_Tz         = 0; 		// read
UINT8   FT_OverloadStatus = 0; 		// read
UINT8   FT_ErrorFlag      = 0;       // read

double Tx[NUM_FT]={0.0};
double Ty[NUM_FT]={0.0};
double Tz[NUM_FT]={0.0};
double Fx[NUM_FT]={0.0};
double Fy[NUM_FT]={0.0};
double Fz[NUM_FT]={0.0};

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

typedef union{
	struct{
		uint8 reserve;
		uint8 opt;
		uint16 data;
	}info;
	uint8 value[8];
}FTPacket;

// Cycle time in nanosecond
unsigned int cycle_ns = (unsigned int)(1000.0/CONTROL_FREQ*1000000.0); /* 1 ms */
double dt = 1.0/CONTROL_FREQ; /* 1 ms */
double wc = 105.0; /* CUT OFF FREQUENCY*/
JVec MAX_TORQUES;
static int period = 1000000;
int traj_flag = 0;
#endif /* RTRARMCLIENT_H_ */
