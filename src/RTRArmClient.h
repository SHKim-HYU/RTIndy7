/*
 * RTRArmClient.h
 *
 *  Created on: 2018. 11. 28.
 *      Author: Administrator
 */

#ifndef RTRARMCLIENT_H_
#define RTRARMCLIENT_H_

/***** License Information *****/
#define USERNAME "NeuromekaDev"
#define EMAIL "dev@neuromeka.com"
#define SERIAL "nrmk13766"
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

// #include "Control/Trajectory.h"
// #include "Control/Controller.h"
// #include "KDL/SerialRobot.h"
// #include "KDL/PoEKinematics.h"

#include "KDL/PropertyDefinition.h"
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


typedef struct STATE{
    double q[ROBOT_DOF];
    double q_dot[ROBOT_DOF];
    double q_ddot[ROBOT_DOF];
    double torque[ROBOT_DOF];
    double dq[ROBOT_DOF];
    double dq_dot[ROBOT_DOF];
    double dq_ddot[ROBOT_DOF];

	JVec j_q;
	JVec j_q_d;
	JVec j_q_dd;
	JVec j_torque;

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
double MAX_TORQUE[JOINTNUM]={431.97,431.97,197.23,79.79,79.79,79.79};
static int period = 1000000;

#endif /* RTRARMCLIENT_H_ */
