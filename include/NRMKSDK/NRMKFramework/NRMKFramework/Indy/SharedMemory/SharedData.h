/*
 * SharedData.hpp
 *
 *  Created on: 2017. 10. 23.
 *      Author: Hanter Jung
 */

#ifndef NRMKINDY_SHAREDDATA_H_
#define NRMKINDY_SHAREDDATA_H_

#include "SharedMemoryAddress.h"
#include "../../shmem/ShmemManager.hpp"
#include "inttypes.h"

namespace NRMKIndy
{

namespace SharedData
{

enum
{
	MAX_NUM_BODIES = 11,
	MAX_NUM_JOINTS = 10,
	MAX_JOINT_DOF = MAX_NUM_JOINTS,
//	MAX_JOINT_DOF = 10,
	NUM_TASK_AXES = 6
};


#pragma pack(push)  /* push current alignment to stack */
#pragma pack(4)     /* set alignment to 4 byte boundary */

struct RobotRTSharedData
{
	double time;
	uint64_t taskTime;
	uint64_t taskTimeMax;
	uint64_t computeTime;
	uint64_t computeTimeMax;
	uint64_t ecatTime;
	uint64_t ecatTimeMax;
	uint32_t ecatMasterState;
	uint32_t ecatSlaveNum;

	RobotRTSharedData()
	: time(0.0)
	, taskTime(0), taskTimeMax(0), computeTime(0), computeTimeMax(0)
	, ecatTime(0), ecatTimeMax(0), ecatMasterState(0), ecatSlaveNum(0)
	{
	}

	RobotRTSharedData(
			const int numJoints,
			const double time,
			const uint64_t taskTime,
			const uint64_t taskTimeMax,
			const uint64_t computeTime,
			const uint64_t computeTimeMax,
			const uint64_t ecatTime,
			const uint64_t ecatTimeMax,
			const uint32_t ecatMasterState,
			const uint32_t ecatSlaveNum)
	: time(time)
	, taskTime(taskTime), taskTimeMax(taskTimeMax), computeTime(computeTime), computeTimeMax(computeTimeMax)
	, ecatTime(ecatTime), ecatTimeMax(ecatTimeMax), ecatMasterState(ecatMasterState), ecatSlaveNum(ecatSlaveNum)
	{
	}

	RobotRTSharedData(RobotRTSharedData const & data)
	{
		time			= data.time;
		taskTime		= data.taskTime;
		taskTimeMax		= data.taskTimeMax;
		computeTime		= data.computeTime;
		computeTimeMax	= data.computeTimeMax;
		ecatTime		= data.ecatTime;
		ecatTimeMax		= data.ecatTimeMax;
		ecatMasterState	= data.ecatMasterState;
		ecatSlaveNum	= data.ecatSlaveNum;
	}

	RobotRTSharedData & operator=(const RobotRTSharedData & data)
	{
		time			= data.time;
		taskTime		= data.taskTime;
		taskTimeMax		= data.taskTimeMax;
		computeTime		= data.computeTime;
		computeTimeMax	= data.computeTimeMax;
		ecatTime		= data.ecatTime;
		ecatTimeMax		= data.ecatTimeMax;
		ecatMasterState	= data.ecatMasterState;
		ecatSlaveNum	= data.ecatSlaveNum;

		return (*this);
	}

	void update(
			const int numJoints,
			const double time,
			const uint64_t taskTime,
			const uint64_t taskTimeMax,
			const uint64_t computeTime,
			const uint64_t computeTimeMax,
			const uint64_t ecatTime,
			const uint64_t ecatTimeMax,
			const uint32_t ecatMasterState,
			const uint32_t ecatSlaveNum)
	{
		this->time				= time;
		this->taskTime			= taskTime;
		this->taskTimeMax		= taskTimeMax;
		this->computeTime		= computeTime;
		this->computeTimeMax	= computeTimeMax;
		this->ecatTime			= ecatTime;
		this->ecatTimeMax		= ecatTimeMax;
		this->ecatMasterState	= ecatMasterState;
		this->ecatSlaveNum		= ecatSlaveNum;
	}
};

struct RobotControlSharedData
{
	double time;
	int cmode;
	int tCmode;

	double q[MAX_JOINT_DOF];		//actual position
	double qdot[MAX_JOINT_DOF];		//actual velocity
	double tauact[MAX_JOINT_DOF];	//actual torque

	double qd[MAX_JOINT_DOF];
	double qdotd[MAX_JOINT_DOF];
	double qdotref[MAX_JOINT_DOF];
	double qddot[MAX_JOINT_DOF];
	double qddotd[MAX_JOINT_DOF];
	double qddotref[MAX_JOINT_DOF];

	double p[NUM_TASK_AXES];
	double pd[NUM_TASK_AXES];
	double pdot[NUM_TASK_AXES];
	double pdotd[NUM_TASK_AXES];
	double pdotref[NUM_TASK_AXES];
	double pddot[NUM_TASK_AXES];
	double pddotd[NUM_TASK_AXES];
	double pddotref[NUM_TASK_AXES];

	double tau[MAX_JOINT_DOF];
	double tauext[MAX_JOINT_DOF];
	double taugrav[MAX_JOINT_DOF];
	double tauidyn[MAX_JOINT_DOF];
	double tauref[MAX_JOINT_DOF];

	double Fext[NUM_TASK_AXES];

	int qState[MAX_JOINT_DOF];
	int qBrake[MAX_JOINT_DOF];

	RobotControlSharedData()
	: time(0.0), cmode(0), tCmode(0)
	, q{0.0}, qdot{0.0}, tauact{0.0}
	, qd{0.0}, qdotd{0.0}, qdotref{0.0}
	, qddot{0.0}, qddotd{0.0}, qddotref{0.0}
	, p{0.0}, pd{0.0}, pdot{0.0}, pdotd{0.0}, pdotref{0.0}
	, pddot{0.0}, pddotd{0.0}, pddotref{0.0}
	, tau{0.0}, tauext{0.0}, taugrav{0.0}
	, tauidyn{0.0}, tauref{0.0}
	, Fext{0.0}
	, qState{0}, qBrake{0}
	{}

	RobotControlSharedData(const int jointDof,
			double time, int cmode, int tCmode,
			const double *q, const double *qdot, const double *tauact,
			const double *qd, const double *qdotd, const double *qdotref,
			const double *qddot, const double *qddotd, const double *qddotref,
			const double *p, const double *pd, const double *pdot, const double *pdotd, const double *pdotref,
			const double *pddot, const double *pddotd, const double *pddotref,
			const double *tau, const double *tauext, const double *taugrav,
			const double *tauidyn, const double *tauref,
			const double *Fext,
			const int *qState, const int *qBrake)
	: time(time), cmode(cmode), tCmode(tCmode)
	{
		memcpy(this->q, q, jointDof*sizeof(double));
		memcpy(this->qdot, qdot, jointDof*sizeof(double));
		memcpy(this->tauact, tauact, sizeof(this->tauact));
		
		memcpy(this->qd, qd, jointDof*sizeof(double));
		memcpy(this->qdotd, qdotd, jointDof*sizeof(double));
		memcpy(this->qdotref, qdotref, jointDof*sizeof(double));
		memcpy(this->qddot, qddot, jointDof*sizeof(double));
		memcpy(this->qddotd, qddotd, jointDof*sizeof(double));
		memcpy(this->qddotref, qddotref, jointDof*sizeof(double));

		memcpy(this->p, p, sizeof(this->p));
		memcpy(this->pd, pd, sizeof(this->pd));
		memcpy(this->pdot, pdot, sizeof(this->pdot));
		memcpy(this->pdotd, pdotd, sizeof(this->pdotd));
		memcpy(this->pdotref, pdotref, sizeof(this->p));
		memcpy(this->pddot, pddot, sizeof(this->pddot));
		memcpy(this->pddotd, pddotd, sizeof(this->pddotd));
		memcpy(this->pddotref, pddotref, sizeof(this->pddotref));

		memcpy(this->tau, tau, jointDof*sizeof(double));
		memcpy(this->tauext, tauext, jointDof*sizeof(double));
		memcpy(this->taugrav, taugrav, jointDof*sizeof(double));
		memcpy(this->tauidyn, tauidyn, jointDof*sizeof(double));
		memcpy(this->tauref, tauref, jointDof*sizeof(double));
		memcpy(this->Fext, Fext, sizeof(this->Fext));

		memcpy(this->qState, qState, jointDof*sizeof(double));
		memcpy(this->qBrake, qBrake, jointDof*sizeof(double));
	}

	RobotControlSharedData(RobotControlSharedData const & data)
	{
		memcpy(this, &data, sizeof(RobotControlSharedData));
	}

	RobotControlSharedData & operator=(const RobotControlSharedData & data)
	{
		memcpy(this, &data, sizeof(RobotControlSharedData));
		return (*this);
	}

	void update(const int jointDof,
			double time, int cmode, int tCmode,
			const double *q, const double *qdot, const double *tauact,
			const double *qd, const double *qdotd, const double *qdotref,
			const double *qddot, const double *qddotd, const double *qddotref,
			const double *p, const double *pd, const double *pdot, const double *pdotd, const double *pdotref,
			const double *pddot, const double *pddotd, const double *pddotref,
			const double *tau, const double *tauext, const double *taugrav,
			const double *tauidyn, const double *tauref,
			const double *Fext,
			const int *qState, const int *qBrake)
	{
		this->time = time;
		this->cmode = cmode;
		this->tCmode = tCmode;

		for (int i=0; i<jointDof; i++)
		{
			this->q[i] = q[i];
			this->qdot[i] = qdot[i];
			this->qd[i] = qd[i];
			this->qdotd[i] = qdotd[i];
			this->qdotref[i] = qdotref[i];
			this->qddot[i] = qddot[i];
			this->qddotd[i] = qddotd[i];
			this->qddotref[i] = qddotref[i];

			this->tau[i] = tau[i];
			this->tauext[i] = tauext[i];
			this->taugrav[i] = taugrav[i];
			this->tauidyn[i] = tauidyn[i];
			this->tauref[i] = tauref[i];
			this->tauact[i] = tauact[i];

			this->qState[i] = qState[i];
			this->qBrake[i] = qBrake[i];
		}

		for (int i=0; i<NUM_TASK_AXES; i++)	//task displacement, xyz first
		{
			this->p[i] = p[i];
			this->pd[i] = pd[i];
			this->pdot[i] = pdot[i];
			this->pdotd[i] = pddotd[i];
			this->pdotref[i] = pdotref[i];
			this->pddot[i] = pddot[i];
			this->pddotd[i] = pddotd[i];
			this->pddotref[i] = pddotref[i];
			this->Fext[i] = Fext[i];
		}
	}
};

#pragma pack(pop)   /* restore original alignment from stack */

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */

struct RobotStateSharedData
{
	bool isTaskRunning;
	bool isCadkitConnected;
	bool isContyConnected;
	bool isScriptRunning;
	bool isIndyEyeConnected;
	bool isToolConnected;
	bool isDebugMode;
	char programMode;
	char programState;

	RobotStateSharedData()
	: isTaskRunning(false)
	, isCadkitConnected(false)
	, isContyConnected(false)
	, isScriptRunning(false)
	, isIndyEyeConnected(false)
	, isToolConnected(false)
	, isDebugMode(false)
	, programMode(0)
	, programState(0)
	{
	}

	RobotStateSharedData(RobotStateSharedData const & data)
	{
		memcpy(this, &data, sizeof(RobotStateSharedData));
	}

	RobotStateSharedData & operator=(const RobotStateSharedData & data)
	{
		memcpy(this, &data, sizeof(RobotStateSharedData));
		return (*this);
	}
};

struct RobotControlStatusSharedData
{
	bool isReady;
	bool isEmergencyState;
	bool isCollided;
	bool isErrorState;
	bool isBusy;
	bool isMoveFinished;
	bool isHome;
	bool isZero;
	bool isInResetting;
	bool isInTeaching;
	bool isInDirectTeaching;

	RobotControlStatusSharedData()
	: isReady(false)
	, isEmergencyState(false)
	, isCollided(false)
	, isErrorState(false)
	, isBusy(false)
	, isMoveFinished(false)
	, isHome(false)
	, isZero(false)
	, isInResetting(false)
	, isInTeaching(false)
	, isInDirectTeaching(false)
	{
	}

	RobotControlStatusSharedData(RobotControlStatusSharedData const & data)
	{
		memcpy(this, &data, sizeof(RobotControlStatusSharedData));
	}

	RobotControlStatusSharedData & operator=(const RobotControlStatusSharedData & data)
	{
		memcpy(this, &data, sizeof(RobotControlStatusSharedData));
		return (*this);
	}
};

#pragma pack(pop)   /* restore original alignment from stack */

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(4)     /* set alignment to 4 byte boundary */

struct RobotMotorSharedData
{
	uint16_t statusWord[MAX_JOINT_DOF];
	uint16_t errorCode[MAX_JOINT_DOF];
    double temperature[MAX_JOINT_DOF];
    double current[MAX_JOINT_DOF];

	RobotMotorSharedData()
	: statusWord{0x0}, errorCode{0x0}, temperature{0.0}
	{
		memset(statusWord, sizeof(statusWord), 0);
		memset(errorCode, sizeof(errorCode), 0);
		memset(temperature, sizeof(temperature), 0);
	}

	RobotMotorSharedData(
			const int numJoints,
			const uint16_t * statusWord,
			const uint16_t * errorCode,
			const double * temperature)
	{
		for (int i = 0; i < numJoints; i++)
			this->statusWord[i] = statusWord[i];
		for (int i = 0; i < numJoints; i++)
			this->errorCode[i] = errorCode[i];
		for (int i = 0; i < numJoints; i++)
			this->temperature[i] = temperature[i];
	}

	RobotMotorSharedData & operator=(const RobotMotorSharedData & data)
	{
		for (int i = 0; i < MAX_JOINT_DOF; i++)
			statusWord[i] = data.statusWord[i];
		for (int i = 0; i < MAX_JOINT_DOF; i++)
			errorCode[i] = data.errorCode[i];
		for (int i = 0; i < MAX_JOINT_DOF; i++)
			this->temperature[i] = temperature[i];

		return (*this);
	}

	void update(
			const int numJoints,
			const uint16_t * statusWord,
			const uint16_t * errorCode = nullptr,
			const double * temperature = nullptr)
	{
		for (int i = 0; i < numJoints; i++)
			this->statusWord[i] = statusWord[i];
		if (errorCode != nullptr)
			for (int i = 0; i < numJoints; i++)
				this->errorCode[i] = errorCode[i];
		if (temperature != nullptr)
			for (int i = 0; i < numJoints; i++)
				this->temperature[i] = temperature[i];
	}
};

struct MotorVersion	//256
{
	char revisionName[32];
	char firmwareVersion[96];
	char reserve[128];

	MotorVersion()
	: revisionName{0}, firmwareVersion{0}, reserve{0}
	{
	}

	MotorVersion(
			const char * revisionName,
			const char * firmwareVersion)
	{
		strncpy(this->revisionName, revisionName, 31);
		strncpy(this->firmwareVersion, firmwareVersion, 95);
	}

	void set(
			const char * revisionName,
			const char * firmwareVersion)
	{
		strncpy(this->revisionName, revisionName, 31);
		strncpy(this->firmwareVersion, firmwareVersion, 95);
	}
};

struct RobotInfoSharedData
{
	char model[256];
	char buildVersion[128];
	char buildDate[128];
	char robotSN[128];		//serial number of robot
	char cbSN[128];			//serial number of control box
	char stepSN[128];		//serial number of step
	MotorVersion motorVersion[MAX_JOINT_DOF];
	int64_t zeroPosition[MAX_JOINT_DOF];
	int32_t numBodies, numJoints, jointDof;

	RobotInfoSharedData()
	: model{0}, buildVersion{0}, buildDate{0}
	, robotSN{0}, stepSN{0}
	, numBodies(MAX_NUM_BODIES), numJoints(MAX_JOINT_DOF), jointDof(MAX_JOINT_DOF)
	{
		memset(zeroPosition, sizeof(zeroPosition), 0);
	}

	RobotInfoSharedData(
			const char * model,
			const char * buildVersion,
			const char * buildDate,
			const char * robotSN,
			const char * cbSN,
			const char * stepSN,
			const MotorVersion * motorVersion,
			const int64_t * zeroPosition,
			const int32_t numBodies,
			const int32_t numJoints,
			const int32_t jointDof)
	{
		strncpy(this->model, model, 255);
		strncpy(this->buildVersion, buildVersion, 127);
		strncpy(this->buildDate, buildDate, 127);
		strncpy(this->robotSN, robotSN, 127);
		strncpy(this->cbSN, stepSN, 127);
		strncpy(this->stepSN, stepSN, 127);

		for (int i=0; i<jointDof; i++)
		{
			this->motorVersion[i] = motorVersion[i];
			this->zeroPosition[i] = zeroPosition[i];
		}
		for (int i=jointDof; i<MAX_JOINT_DOF; i++)
		{
			this->motorVersion[i] = MotorVersion();
			this->zeroPosition[i] = 0;
		}

		this->numBodies = numBodies;
		this->numJoints = numJoints;
		this->jointDof = jointDof;
	}
};

struct RobotConfigSharedData
{
	double wTime;
	double wTimeTask;
	int32_t jointVelBoundaryLevel;
	int32_t jointAccBoundaryLevel;
	int32_t taskVelBoundaryLevel;
	int32_t taskAccBoundaryLevel;
	int32_t collisionLevel;
	int32_t refFrameType;
	double toolProperties[4];
	double defaultTcp[NUM_TASK_AXES];
	double compTcp[NUM_TASK_AXES];
	double refFrameTRef[NUM_TASK_AXES];
	double refFramePoints[3][3];
	double jointHome[MAX_JOINT_DOF];
	double taskHome[NUM_TASK_AXES];
	double defaultJointBlendRadius;
	double defaultTaskBlendRadius;
	double mountAngle[2];

	RobotConfigSharedData()
	: wTime(4)
	, wTimeTask(4)
	, jointVelBoundaryLevel(5)
	, jointAccBoundaryLevel(5)
	, taskVelBoundaryLevel(5)
	, taskAccBoundaryLevel(5)
	, collisionLevel(2)
	, refFrameType(0)
	, defaultJointBlendRadius(0.0)
	, defaultTaskBlendRadius(0.0)
	, mountAngle{0, 0}
	{
		for (int i=0; i<MAX_JOINT_DOF; i++)
		{
			jointHome[i] = 0;
		}

		for (int i=0; i<NUM_TASK_AXES; i++)
		{
			defaultTcp[i] = 0;
			compTcp[i] = 0;
			taskHome[i] = 0;
			refFrameTRef[i] = 0;
		}

		for (int i=0; i<3; i++)
			for (int j=0; j<3; j++)
				refFramePoints[i][j] = 0;
	}

	RobotConfigSharedData(RobotConfigSharedData const & data)
	{
		memcpy(this, &data, sizeof(RobotConfigSharedData));
	}

	RobotConfigSharedData & operator=(const RobotConfigSharedData & data)
	{
		memcpy(this, &data, sizeof(RobotConfigSharedData));
		return (*this);
	}
};

#pragma pack(pop)   /* restore original alignment from stack */

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */

struct RobotEmergencySharedData
{
	int errorCode;			//4
	int argsInt[3];			//4*3=12
	double argsDouble[3];	//8*3=24
	double time;			//8
	char msg[256];			//256

	RobotEmergencySharedData()
	: errorCode(-1), argsInt{0}, argsDouble{0.0}, time{0.0}
	{}

	RobotEmergencySharedData(int errorCode, double t, int arg1=0, int arg2=0, int arg3=0)
	: errorCode(errorCode)
	{
		time = t;
		argsInt[0] = arg1;
		argsInt[1] = arg2;
		argsInt[2] = arg3;
		for (int i = 0; i < 3; i++)
		{
			argsDouble[i] = 0.0;
		}
	}

	RobotEmergencySharedData(int errorCode, double t, double arg1, double arg2=0.0, double arg3=0.0)
	: errorCode(errorCode)
	{
		time = t;
		argsDouble[0] = arg1;
		argsDouble[1] = arg2;
		argsDouble[2] = arg3;
		for (int i = 0; i < 3; i++)
		{
			argsInt[i] = 0;
		}
	}

	RobotEmergencySharedData(int errorCode, double t, int arg1, int arg2, int arg3, double arg4, double arg5, double arg6)
	: errorCode(errorCode)
	{
		time = t;
		argsInt[0] = arg1;
		argsInt[1] = arg2;
		argsInt[2] = arg3;
		argsDouble[0] = arg4;
		argsDouble[1] = arg5;
		argsDouble[2] = arg6;
	}

	RobotEmergencySharedData(RobotEmergencySharedData const & data)
	{
		errorCode = data.errorCode;
		time = data.time;
		for (int i = 0; i < 3; i++)
		{
			argsInt[i] = data.argsInt[i];
			argsDouble[i] = data.argsDouble[i];
		}
	}

	RobotEmergencySharedData & operator=(const RobotEmergencySharedData & data)
	{
		errorCode = data.errorCode;
		time = data.time;
		for (int i = 0; i < 3; i++)
		{
			argsInt[i] = data.argsInt[i];
			argsDouble[i] = data.argsDouble[i];
		}
		return (*this);
	}

	void reset()
	{
		errorCode = -1;
		time = 0.0;
		for (int i = 0; i < 3; i++)
		{
			argsInt[i] = 0;
			argsDouble[i] = 0.0;
		}
	}
};

#pragma pack(pop)   /* restore original alignment from stack */


#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */
struct UserConfigSharedData
{
	int registeredDefaultProgramIdx;

	UserConfigSharedData()
	: registeredDefaultProgramIdx(-1)
	{
	}

	UserConfigSharedData(UserConfigSharedData const & data)
	{
		memcpy(this, &data, sizeof(UserConfigSharedData));
	}

	UserConfigSharedData & operator=(const UserConfigSharedData & data)
	{
		memcpy(this, &data, sizeof(UserConfigSharedData));
		return (*this);
	}
};
#pragma pack(pop)   /* restore original alignment from stack */


#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */
struct SmartDIOSharedData
{
	SmartDIOSharedData()
	{
		memset(smartDI, 0, sizeof(smartDI));
		memset(smartDO, 0, sizeof(smartDO));
	}

	void reset()
	{
		memset(smartDI, 0, sizeof(smartDI));
		memset(smartDO, 0, sizeof(smartDO));
	}

	bool smartDI[32];
	bool smartDO[32];
};

struct SmartAIOSharedData
{
	SmartAIOSharedData()
	{
		memset(smartAI, 0, sizeof(smartAI));
		memset(smartAO, 0, sizeof(smartAO));
	}

	void reset()
	{
		memset(smartAI, 0, sizeof(smartAI));
		memset(smartAO, 0, sizeof(smartAO));
	}

	uint16_t smartAI[4];
	uint16_t smartAO[4];
};

struct EndToolDOSharedData
{
	EndToolDOSharedData()
	{
		memset(endtoolDO, 0, sizeof(endtoolDO));
	}

	void reset()
	{
		memset(endtoolDO, 0, sizeof(endtoolDO));
	}

	bool endtoolDO[4];
};
#pragma pack(pop)   /* restore original alignment from stack */


#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 4 byte boundary */

struct ExtraIOData
{
	int16_t ftRobotCanRaw[NUM_TASK_AXES];
	double ftRobotCanTr[NUM_TASK_AXES];
	int16_t ftCBCanRaw[NUM_TASK_AXES];
	double ftCBCanTr[NUM_TASK_AXES];

	ExtraIOData() {}

	ExtraIOData(const int16_t* ftRobotCanRaw, const double *ftRobotCanTr,
			const int16_t* ftCBCanRaw, const double *ftCBCanTr)
	{
		memcpy(this->ftRobotCanRaw, ftRobotCanRaw, sizeof(this->ftRobotCanRaw));
		memcpy(this->ftRobotCanTr, ftRobotCanTr, sizeof(this->ftRobotCanTr));
		memcpy(this->ftCBCanRaw, ftCBCanRaw, sizeof(this->ftCBCanRaw));
		memcpy(this->ftCBCanTr, ftCBCanTr, sizeof(this->ftCBCanTr));
	}

	ExtraIOData(ExtraIOData const & data)
	{
		memcpy(this, &data, sizeof(ExtraIOData));
	}

	ExtraIOData & operator=(const ExtraIOData & data)
	{
		memcpy(this, &data, sizeof(ExtraIOData));
		return (*this);
	}
};
#pragma pack(pop)   /* restore original alignment from stack */


#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 4 byte boundary */

struct IndyCareStateData
{
    bool isReporterRunning;
    bool isServerConnected;
    char indyCareVersion[128];
    char lastSyncDate[128];
    char indyCareServerIPAddr[128];
    int indyCareServerPort;
};

#pragma pack(pop)   /* restore original alignment from stack */



inline void getRTData(NRMKFramework::ShmemManager & indyShm, RobotRTSharedData & rtData)
{
	indyShm.readMemory(INDY_SHM_RT_ADDR_STRUCT_DATA, sizeof(RobotRTSharedData), &rtData);
}

inline RobotRTSharedData getRTData(NRMKFramework::ShmemManager & indyShm)
{
	RobotRTSharedData rtData;
	getRTData(indyShm, rtData);
	return rtData;
}

inline void getControlData(NRMKFramework::ShmemManager & indyShm, RobotControlSharedData & ctrlData)
{
//	static const uint32_t sizeTime = sizeof(double);
//	static const uint32_t sizeCmode = (sizeof(int)*2);	//cmode, tcmode
//	static const uint32_t sizeCtrl = (sizeof(double)*MAX_JOINT_DOF*13) + (sizeof(double)*NUM_TASK_AXES*9) + (sizeof(int)*MAX_JOINT_DOF*2);	//q~Fext, qState, qBrake
//
//	indyShm.readMemory(INDY_SHM_RT_ADDR_RUNNING_TIME, sizeTime, &ctrlData.time);
//	indyShm.readMemory(INDY_SHM_ROBOT_ADDR_CTRL_CMODE, sizeCmode, &ctrlData.cmode);
//	indyShm.readMemory(INDY_SHM_ROBOT_ADDR_CTRL_STRUCT_DATA, sizeCtrl, ctrlData.q);

	indyShm.readMemory(INDY_SHM_ROBOT_ADDR_CTRL_STRUCT_DATA, sizeof(RobotControlSharedData), &ctrlData);
}

inline RobotControlSharedData getControlData(NRMKFramework::ShmemManager & indyShm)
{
	RobotControlSharedData ctrlData;
	getControlData(indyShm, ctrlData);
	return ctrlData;
}

inline void getRobotStateData(NRMKFramework::ShmemManager & indyShm, RobotStateSharedData & stateData)
{
	indyShm.readMemory(INDY_SHM_ROBOT_ADDR_STATE_STRUCT_DATA, sizeof(RobotStateSharedData), &stateData);
}

inline RobotStateSharedData getRobotStateData(NRMKFramework::ShmemManager & indyShm)
{
	RobotStateSharedData stateData;
	getRobotStateData(indyShm, stateData);
	return stateData;
}

inline void getControlStatusData(NRMKFramework::ShmemManager & indyShm, RobotControlStatusSharedData & ctrlStatusData)
{
	indyShm.readMemory(INDY_SHM_ROBOT_ADDR_CTRL_STATUS_STRUCT_DATA, sizeof(RobotControlStatusSharedData), &ctrlStatusData);
}

inline RobotControlStatusSharedData getControlStatusData(NRMKFramework::ShmemManager & indyShm)
{
	RobotControlStatusSharedData ctrlStatusData;
	getControlStatusData(indyShm, ctrlStatusData);
	return ctrlStatusData;
}

inline void getRobotMotorData(NRMKFramework::ShmemManager & indyShm, RobotMotorSharedData & motorData)
{
	indyShm.readMemory(INDY_SHM_ROBOT_ADDR_MOTOR_STRUCT_DATA, sizeof(RobotMotorSharedData), &motorData);
}

inline RobotMotorSharedData getRobotMotorData(NRMKFramework::ShmemManager & indyShm)
{
	RobotMotorSharedData motorData;
	getRobotMotorData(indyShm, motorData);
	return motorData;
}

inline void getRobotInfoData(NRMKFramework::ShmemManager & indyShm, RobotInfoSharedData & infoData)
{
	indyShm.readMemory(INDY_SHM_ROBOT_ADDR_INFO_STRUCT_DATA, sizeof(RobotInfoSharedData), &infoData);
}

inline RobotInfoSharedData getRobotInfoData(NRMKFramework::ShmemManager & indyShm)
{
	RobotInfoSharedData infoData;
	getRobotInfoData(indyShm, infoData);
	return infoData;
}

inline void getRobotConfigData(NRMKFramework::ShmemManager & indyShm, RobotConfigSharedData & infoData)
{
	indyShm.readMemory(INDY_SHM_ROBOT_ADDR_CONFIG_STRUCT_DATA, sizeof(RobotConfigSharedData), &infoData);
}

inline RobotConfigSharedData getRobotConfigData(NRMKFramework::ShmemManager & indyShm)
{
	RobotConfigSharedData infoData;
	getRobotConfigData(indyShm, infoData);
	return infoData;
}

inline void getEmergencyData(NRMKFramework::ShmemManager & indyShm, RobotEmergencySharedData & emgData)
{
	indyShm.readMemory(INDY_SHM_ROBOT_ADDR_EMERG_STRUCT_DATA, sizeof(RobotEmergencySharedData), &emgData);
}

inline void resetEmergencyData(NRMKFramework::ShmemManager & indyShm, RobotEmergencySharedData & emgData)
{
	static const RobotEmergencySharedData noEmgData;
	emgData.reset();
	indyShm.writeMemory(INDY_SHM_ROBOT_ADDR_EMERG_STRUCT_DATA, sizeof(RobotEmergencySharedData), &noEmgData);
}

inline RobotEmergencySharedData getEmergencyData(NRMKFramework::ShmemManager & indyShm)
{
	RobotEmergencySharedData emgData;
	getEmergencyData(indyShm, emgData);
	return emgData;
}

inline void getUserConfigData(NRMKFramework::ShmemManager & indyShm, UserConfigSharedData & configData)
{
	indyShm.readMemory(INDY_SHM_ROBOT_ADDR_CONFIG_USER_STRUCT_DATA, sizeof(UserConfigSharedData), &configData);
}

inline UserConfigSharedData getUserConfigData(NRMKFramework::ShmemManager & indyShm)
{
	UserConfigSharedData configData;
	getUserConfigData(indyShm, configData);
	return configData;
}

inline void getIndyCareStateData(NRMKFramework::ShmemManager &indyShm, IndyCareStateData &indyCareData)
{
    indyShm.readMemory(INDY_SHM_ROBOT_ADDR_INDYCARE_STATE_STRUCT_DATA, sizeof(IndyCareStateData), &indyCareData);
}

inline IndyCareStateData getIndyCareStateData(NRMKFramework::ShmemManager &indyShm)
{
    IndyCareStateData indyCareData;
    getIndyCareStateData(indyShm, indyCareData);
    return indyCareData;
}


inline void getSmartDIO(NRMKFramework::ShmemManager & indyShm, SmartDIOSharedData & dio)
{
	indyShm.readMemory(INDY_SHM_SERVER_ADDRCUST_SMART_DI, sizeof(SmartDIOSharedData), &dio);
}

inline SmartDIOSharedData getSmartDIO(NRMKFramework::ShmemManager & indyShm)
{
	SmartDIOSharedData dio;
	getSmartDIO(indyShm, dio);
	return dio;
}

inline void getPtrSmartDIO(NRMKFramework::ShmemManager & indyShm, SmartDIOSharedData ** ptrDio)
{
	*ptrDio = indyShm.getPtrObjByAddr<SmartDIOSharedData>(INDY_SHM_SERVER_ADDRCUST_SMART_DI);
}

inline bool getSmartDI(NRMKFramework::ShmemManager & indyShm, int diNum)
{
	bool diVal;
	indyShm.readMemory(INDY_SHM_SERVER_ADDRCUST_SMART_DI+diNum, sizeof(bool), &diVal);
	return diVal;
}

inline bool getSmartDO(NRMKFramework::ShmemManager & indyShm, int doNum)
{
	bool doVal;
	indyShm.readMemory(INDY_SHM_SERVER_ADDRCUST_SMART_DO+doNum, sizeof(bool), &doVal);
	return doVal;
}

inline void setSmartDO(NRMKFramework::ShmemManager & indyShm, int doNum, bool doVal)
{
	indyShm.writeMemory(INDY_SHM_SERVER_ADDRCUST_SMART_DO+doNum, sizeof(bool), &doVal);
}

inline void getSmartDI(NRMKFramework::ShmemManager & indyShm, bool * smartDI)
{
	indyShm.readMemory(INDY_SHM_SERVER_ADDRCUST_SMART_DI, sizeof(bool)*32, smartDI);
}

inline void setSmartDO(NRMKFramework::ShmemManager & indyShm, const bool * smartDO)
{
	indyShm.writeMemory(INDY_SHM_SERVER_ADDRCUST_SMART_DO, sizeof(bool)*32, smartDO);
}

inline void getSmartAIO(NRMKFramework::ShmemManager & indyShm, SmartAIOSharedData & aio)
{
	indyShm.readMemory(INDY_SHM_SERVER_ADDRCUST_SMART_AI, sizeof(SmartAIOSharedData), &aio);
}

inline SmartAIOSharedData getSmartAIO(NRMKFramework::ShmemManager & indyShm)
{
	SmartAIOSharedData aio;
	getSmartAIO(indyShm, aio);
	return aio;
}

inline void getPtrSmartAIO(NRMKFramework::ShmemManager & indyShm, SmartAIOSharedData ** ptrAio)
{
	*ptrAio = indyShm.getPtrObjByAddr<SmartAIOSharedData>(INDY_SHM_SERVER_ADDRCUST_SMART_AI);
}

inline uint16_t getSmartAI(NRMKFramework::ShmemManager & indyShm, int aiNum)
{
	uint16_t aiVal;
	indyShm.readMemory(INDY_SHM_SERVER_ADDRCUST_SMART_AI+(aiNum*sizeof(uint16_t)), sizeof(uint16_t), &aiVal);
	return aiVal;
}

inline uint16_t getSmartAO(NRMKFramework::ShmemManager & indyShm, int aoNum)
{
	uint16_t aoVal;
	indyShm.readMemory(INDY_SHM_SERVER_ADDRCUST_SMART_AO+(aoNum*sizeof(uint16_t)), sizeof(uint16_t), &aoVal);
	return aoVal;
}

inline void setSmartAO(NRMKFramework::ShmemManager & indyShm, int aoNum, uint16_t aoVal)
{
	indyShm.writeMemory(INDY_SHM_SERVER_ADDRCUST_SMART_AO+(aoNum*sizeof(uint16_t)), sizeof(uint16_t), &aoVal);
}

inline void getEndToolDO(NRMKFramework::ShmemManager & indyShm, EndToolDOSharedData & endtoolDo)
{
	indyShm.readMemory(INDY_SHM_SERVER_ADDRCUST_ENDTOOL_DO, sizeof(EndToolDOSharedData), &endtoolDo);
    //indyShm.writeMemory(INDY_SHM_SERVER_ADDRCUST_ENDTOOL_DO, sizeof(EndToolDOSharedData), &endtoolDo);
}

inline EndToolDOSharedData getEndToolDO(NRMKFramework::ShmemManager & indyShm)
{
	EndToolDOSharedData endtoolDo;
	getEndToolDO(indyShm, endtoolDo);
	return endtoolDo;
}

//WYLee
inline void setEndToolDO(NRMKFramework::ShmemManager & indyShm, EndToolDOSharedData & endtoolDo)
{
    indyShm.writeMemory(INDY_SHM_SERVER_ADDRCUST_ENDTOOL_DO, sizeof(EndToolDOSharedData), &endtoolDo);
}


inline void getPtrEndToolDO(NRMKFramework::ShmemManager & indyShm, EndToolDOSharedData ** ptrEndtoolDo)
{
	*ptrEndtoolDo = indyShm.getPtrObjByAddr<EndToolDOSharedData>(INDY_SHM_SERVER_ADDRCUST_ENDTOOL_DO);
}

inline void getExtraIOData(NRMKFramework::ShmemManager & indyShm, ExtraIOData & extIOData)
{
	indyShm.readMemory(INDY_SHM_SERVER_ADDRCUST_EXTRA_IO_DATA, sizeof(ExtraIOData), &extIOData);
}

inline ExtraIOData getExtraIOData(NRMKFramework::ShmemManager & indyShm)
{
	ExtraIOData extIOData;
	getExtraIOData(indyShm, extIOData);
	return extIOData;
}

inline void getPtrExtraIOData(NRMKFramework::ShmemManager & indyShm, ExtraIOData ** ptrExtIOData)
{
	*ptrExtIOData = indyShm.getPtrObjByAddr<ExtraIOData>(INDY_SHM_SERVER_ADDRCUST_EXTRA_IO_DATA);
}



} /* namespace SharedData */

} /* namespace NRMKIndy */

#endif /* NRMKINDY_SHAREDDATA_H_ */
