/*
 * IndyDCPConnector.h
 *
 *  Created on: 2019. 1. 10.
 *      Author: Hanter Jung
 */

#ifndef NRMKINDY_SERVICE_INDYDCPCONNECTOR_H_
#define NRMKINDY_SERVICE_INDYDCPCONNECTOR_H_
#pragma once

#include "IndyDCP.h"
#include "IndyDCPUtility.h"
#include "IndyDCPException.h"

#if defined (WINDOWS)
#define _CRT_SECURE_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WinSock2.h>
#pragma comment(lib,"ws2_32")	//Must Link Winsock Library "ws2_32.dll"
//#include <WS2tcpip.h>
#include <string.h>
#include <sys/types.h>
#else
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#endif	//WINDOWS

#include <string>
#include <sstream>
#include <iostream>
#include <memory>
#include <array>
#include <limits>
#include <thread>
#include <mutex>

namespace NRMKIndy
{
namespace Service
{
namespace DCP
{

class IndyDCPConnector
{
protected:
	enum {
		SERVER_PORT = 6066,
		DEFAULT_TIMEOUT = 10,
		MAX_COMMAND_ARR = 200,
	};

public:
	bool connect();
	void disconnect();
	void shutdown();

	bool isConnected();
	void setTimeoutSeconds(double seconds=DEFAULT_TIMEOUT);

public:
	//DCP Commands
	int check();
	int stopEmergency();
	int resetRobot();
	int setServoOnOff(const char * const servoVec);
	int setServoOnOff(const bool * const servoVec);
	int setServoOnOff(std::initializer_list<char> servoVec);
	int setBrakeOnOff(const char * const brakeVec);
	int setBrakeOnOff(const bool * const brakeVec);
	int setBrakeOnOff(std::initializer_list<char> brakeVec);

	int stopMotion();
	int executeMoveCommand(const std::string & cmdName);
	int moveJointHome();
	int moveJointZero();
	int moveJointTo(const double * const qvec);
	int moveJointTo(std::initializer_list<double> qvec);
	int moveJointBy(const double * const qvec);
	int moveJointBy(std::initializer_list<double> qvec);
	int moveTaskTo(const double * const pvec);
	int moveTaskTo(std::initializer_list<double> pvec);
	int moveTaskBy(const double * const pvec);
	int moveTaskBy(std::initializer_list<double> pvec);

	int startCurrProgram();
	int pauseProgram();
	int resumeProgram();
	int stopProgram();
	int startRegisteredDefaultProgram();
	int registerDefaultProgram(int idx);
	int getRegisteredDefaultProgram(int & ret);

	// Old Status Getting Functions
	int isRobotRunning(bool & ret);
	int isRobotReady(bool & ret);
	int isEmergencyStopped(bool & ret);
	int isCollided(bool & ret);
	int isErrorState(bool & ret);
	int isBusy(bool & ret);
	int isMoveFinished(bool & ret);
	int isHome(bool & ret);
	int isZero(bool & ret);
	int isInResetting(bool & ret);
	int isDirectTeachingMode(bool & ret);
	int isTeachingMode(bool & ret);
	int isProgramRunning(bool & ret);
	int isProgramPaused(bool & ret);
	int isContyConnected(bool & ret);

	int changeToDirectTeachingMode();
	int finishDirectTeachingMode();

	int addJointWaypointSet(const double * const qvec);
	int addJointWaypointSet(std::initializer_list<double> qvec);
	int removeJointWaypointSet();
	int clearJointWaypointSet();
	int executeJointWaypointSet();

	int addTaskWaypointSet(const double * const pvec);
	int addTaskWaypointSet(std::initializer_list<double> pvec);
	int removeTaskWaypointSet();
	int clearTaskWaypointSet();
	int executeTaskWaypointSet();

	int setDefaultTCP(const double * const tcpVec);
	int setDefaultTCP(std::initializer_list<double> tcpVec);
	int resetDefaultTCP();
	int setTCPCompensation(const double * const tcpVec);
	int setTCPCompensation(std::initializer_list<double> tcpVec);
	int resetTCPCompensation();
	int setReferenceFrame(const double * const refFrameVec);
	int setReferenceFrame(std::initializer_list<double> refFrameVec);
	int resetReferenceFrame();
	int setCollisionDetectionLevel(int level);
	int setJointBoundaryLevel(int level);
	int setTaskBoundaryLevel(int level);
	int setJointBlendingRadius(double radius);
	int setTaskBlendingRadius(double radius);
	int setJointWaypointTime(double time);
	int setTaskWaypointTime(double time);
	int setTaskBaseMode(int mode);

	int getDefaultTCP(double * const ret);
	int getTCPCompensation(double * const ret);
	int getReferenceFrame(double * const ret);
	int getCollisionDetectionLevel(int & ret);
	int getJointBoundaryLevel(int & ret);
	int getTaskBoundaryLevel(int & ret);
	int getJointBlendingRadius(double & ret);
	int getTaskBlendingRadius(double & ret);
	int getJointWaypointTime(double & ret);
	int getTaskWaypointTime(double & ret);
	int getTaskBaseMode(int & ret);

	int getRobotRunningTime(double & ret);
	int getControlMode(int & ret);
	int getJointServoState(char * const ret);
	int getJointServoState(char * const retServo, char * const retBrake);
	int getJointServoState(bool * const retServo, bool * const retBrake);

	int getJointPosition(double * const ret);
	int getJointVelocity(double * const ret);
	int getTaskPosition(double * const ret);
	int getTaskVelocity(double * const ret);
	int getTorque(double * const ret);

	int getLastEmergencyInfo(int & retCode, int * const retIntArgs, double * const retDoubleArgs);

	int getSmartDigitalInput(int idx, char & ret);
	int getSmartDigitalInputs(char * const & ret);
	int setSmartDigitalOutput(int idx, char val);
	int setSmartDigitalOutputs(const char * const val);
	int getSmartAnalogInput(int idx, int & ret);
	int setSmartAnalogOutput(int idx, int val);
	int getSmartDigitalOutput(int idx, char & ret);
	int getSmartDigitalOutputs(char * const ret);
	int getSmartAnalogOutput(int idx, int & ret);

	int getRobotCanFTSensorRaw(int * const ret);
	int getRobotCanFTSensorTransformed(double * const ret);
	int getCBCanFTSensorRaw(int * const ret);
	int getCBCanFTSensorTransformed(double * const ret);

	int readDirectVariable(int type, int addr, void * ret);
	int readDirectVariables(int type, int addr, int len, void * ret);
	int writeDirectVariable(int type, int addr, void * ptrVal);
	int writeDirectVariables(int type, int addr, int len, void * ptrVal);

	//Extended DCP Commands
	int moveExtTrajBinaryData(const unsigned char * const data, int dataSize);
	int moveExtTrajTextData(const unsigned char * const data, int dataSize);
	int moveExtTrajBinaryFile(const std::string & fileName);
	int moveExtTrajTextFile(const std::string & fileName);

	int moveJointWaypointSet(const double * wpSet, int wpSetLen);
	int moveJointWaypointSet(std::initializer_list<double> wpQvec);
	int moveTaskWaypointSet(const double * wpSet, int wpSetLen);
	int moveTaskWaypointSet(std::initializer_list<double> wpPvec);

	//Get Status
	uint32_t getStatusBits();
	bool getStatus(uint32_t statusType);

protected:
//	bool _sendRequest(int invokeId, int cmd, int dataSize=0, const Data &reqData=Data());
//	bool _recvResponse(int invokeId, int & dataSize, Data &resData);
//	bool _sendExtRequest(int invokeId, int extCmd, int extDataSize, const unsigned char * extReqData=NULL);
//	bool _recvExtResponse(int invokeId, int & dataSize, Data &resData);

	int _handleCommand(int cmd, int & resDataSize, Data &resData,
			int reqDataSize=0, const Data & reqData=Data());
	int _handleExtCommand(int extCmd, int & resExtDataSize, unsigned char *& resExtData,
			int reqExtDataSize=0, const unsigned char * reqExtData=NULL);

	//TODO change to socket
	bool _sendMessage(unsigned char const * buffer, size_t size);
	bool _recvMessage(unsigned char * buffer, size_t size);

	virtual int _checkRobotDOF(const std::string & _robot);
	virtual int _checkRobotAxes(const std::string & _robot);

private:
	const std::string _serverIP;
	const std::string _robot;
	const int _dof;
	const int _axes;
	const unsigned char _step;

#if defined (WINDOWS)
	SOCKET _sockFd;
#else
	int _sockFd;
#endif
	bool _connected;
	double _limitTimeout, _currTimeout;
	int _invokeCount;
	std::mutex _mtx;

	uint32_t _statusBits;

public:
	IndyDCPConnector(const std::string & serverIP, const std::string & robotName, unsigned char stepVersion=0x02);
	virtual ~IndyDCPConnector();
};

} /* namespace DCP */
} /* namespace Service */
} /* namespace NRMKIndy */

#endif /* NRMKINDY_SERVICE_INDYDCPCONNECTOR_H_ */
