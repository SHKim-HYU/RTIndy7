/*
 * SharedCommand.h
 *
 *  Created on: 2018. 01. 23.
 *      Author: Hanter Jung
 */

#ifndef NRMKINDY_SHAREDDATA_SHAREDCOMMAND_H_
#define NRMKINDY_SHAREDDATA_SHAREDCOMMAND_H_

#include "SharedData.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef DEGREE
#define DEGREE  (M_PI/180)
#endif
#ifndef RADIAN
#define RADIAN  (180/M_PI)
#endif

namespace NRMKIndy
{
namespace SharedData
{

class Commander
{
public:
	Commander(NRMKFramework::ShmemManager & indyShm, int jointDof);
	void setSyncMode(bool bSyncMode=true) { _syncMode = bSyncMode; }

	void reset(bool isHardReset=false);

	void stopMove();
	void stopEmergency();
	void stopSafe();

	bool jointMoveHome();
	bool jointMoveZero();
	bool jointMoveTo(const double * q);
	bool jointMoveBy(const double * q);
	bool taskMoveTo(const double * p);
	bool taskMoveBy(const double * p);
	bool extMoveWithBinFile(const char * filePath);
	bool extMoveWithTxtFile(const char * filePath);

	bool jointMoveWaypoint(const char * waypointName);
	bool taskMoveWaypoint(const char * waypointName);
	bool executeMoveCmd(const char * cmdMoveName);

	// bool setServoOnOff(const int * qState);
	// bool setServoOnOffAll();

	bool setCollisionHandlingMethod(int handlingType);
	bool setCollisionSensitivityLevel(int level);
	bool setTaskControlBaseMode(int taskCtrlMode);
	bool setJointHome(const double *q);
	bool setJointHomeCurrPos();
	bool setDefaultTcp(const double * defaultTcp);
	bool applyTcpCompensation(const double * compTcp);
	bool revokeTcpCompensation();
	bool setRefFrameDirect(const double * tref);
	bool setJointMoveWaypointTime(double wTime);
	bool setJointMoveVelocityLevel(int level);
	bool setDefaultJointBlendingRadius(double radius);
	bool setTaskMoveWaypointTime(double wTimeTask);
	bool setTaskMoveVelocityLevel(int level);
	bool setDefaultTaskBlendingRadius(double radius);

	bool switchDirectTeaching();
	bool finishDirectTeaching();

	bool startDefaultProgram();	//DEPRECATED
	bool startCurrProgram();
	bool stopCurrProgram();
	bool pauseCurrProgram();
	bool resumeCurrProgram();
	bool startRegisteredDefaultProgram();
	bool registerDefaultProgramIdx(int index);

private:
	NRMKFramework::ShmemManager & _indyShm;
	const int _jointDof;
	bool _syncMode;

	static const char _cmdFlag;
};


//Deprecated
// namespace Command
// {
// 	//Basically, Commands are asynchronous.
// 	void cmdSoftReset(NRMKFramework::ShmemManager & indyShm);
// 	void cmdHardReset(NRMKFramework::ShmemManager & indyShm);

// 	void cmdStopSlow(NRMKFramework::ShmemManager & indyShm);
// 	void cmdStopEmergency(NRMKFramework::ShmemManager & indyShm);

// 	void cmdJointMoveHome(NRMKFramework::ShmemManager & indyShm);
// 	void cmdJointMoveZero(NRMKFramework::ShmemManager & indyShm);
// 	void cmdJointMoveTo(NRMKFramework::ShmemManager & indyShm, double * q);
// 	void cmdJointMoveBy(NRMKFramework::ShmemManager & indyShm, double * q);
// 	void cmdTaskMoveTo(NRMKFramework::ShmemManager & indyShm, double * p);
// 	void cmdTaskMoveBy(NRMKFramework::ShmemManager & indyShm, double * p);
// 	void cmdExecuteMove(NRMKFramework::ShmemManager & indyShm, const char * cmdName);

// 	void cmdStartCurrProgram(NRMKFramework::ShmemManager & indyShm);
// 	void cmdStopCurrProgram(NRMKFramework::ShmemManager & indyShm);
// 	void cmdPauseCurrProgram(NRMKFramework::ShmemManager & indyShm);
// 	void cmdResumeCurrProgram(NRMKFramework::ShmemManager & indyShm);
// 	void cmdStartDefaultProgram(NRMKFramework::ShmemManager & indyShm);
// } /* namespace Command */


} /* namespace SharedData */
} /* namespace NRMKIndy */

#endif /* NRMKINDY_SHAREDDATA_SHAREDCOMMAND_H_ */
