/*
 * AbstractHardware.hpp
 *
 *  Created on: Aug 14, 2018
 *      Author: ThachDo-MAC
 */

#ifndef ABSTRACTSYSTEM_H_
#define ABSTRACTSYSTEM_H_

#include <stdint.h>
#include <Poco/ClassLibrary.h>
#include <DefineConstant.h>

class AbstractSystem
{
public:
	typedef unsigned int UINT32;
	typedef int32_t INT32;
	typedef int16_t INT16;
	typedef uint16_t UINT16;
	typedef uint8_t UINT8;
	typedef int8_t INT8;

public:
	AbstractSystem() {}
	virtual ~AbstractSystem() {}

	virtual bool configure(double cycleInSec, bool isPosControl = false) = 0;
	virtual bool cleanup() = 0;
	virtual bool isSystemReady() = 0;
	virtual bool isSystemError(int & jIndex, int & errCode) = 0;
	virtual bool isJointActive(int jIndex) = 0;
	virtual bool isPosLimitReached(int & jIndex, int & jPosition) = 0;
	virtual bool isVelLimitReached(int & jIndex, int & jVelocity) = 0;
	virtual bool isPosLimitClosed(int & jIndex, int & jPosition) { return false; }

	virtual void setJointActive(int jIndex, bool mode) = 0;
	virtual void releaseJointBrake(int jIndex) {}
	virtual void closeJointBrake(int jIndex) {}
	virtual void resetAllZeroEncoder() {}
	virtual void resetAllJoints() {}
	virtual void sendReadRequest() {}
	virtual void sendWriteRequest() {}
	virtual void readRobotData(double * const q, double * const qdot) = 0;
	virtual void writeRobotCommand(double const * const qdes, double const * const tau) = 0;
	virtual void readExternalData(double * const dData, const int dCount, int * const iData, const int iCount) {}
	virtual void writeExternalData(double const * const dData, const int dCount, int const * const iData, const int iCount) {}

	virtual UINT16 getJointStatus(int jIndex) { return 0x0; }
	virtual INT32 getJointErrorCode(int & jIndex) { return 0x0; }
	virtual int getSysVersion() { return 0; }

};

#endif /* ABSTRACTSYSTEM_H_ */
