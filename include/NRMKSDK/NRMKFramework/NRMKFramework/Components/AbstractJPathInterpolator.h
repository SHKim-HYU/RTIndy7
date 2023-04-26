/*
 * AbstractJPathInterpolator.hpp
 *
 *  Created on: Aug 17, 2018
 *      Author: ThachDo-MAC
 */

#ifndef ABSTRACTJPATHINTERPOLATOR_HPP_
#define ABSTRACTJPATHINTERPOLATOR_HPP_

#include <Poco/ClassLibrary.h>
#include <Eigen/Eigen>

#include <NRMKFramework/Components/AbstractComponent.h>

template<int DIM>
class AbstractJPathInterpolator : public NRMKLeanFramework::AbstractComponent
{
public:
	typedef Eigen::Matrix<double, DIM, 1> VecType;

	enum
	{
		SET_PATH_TYPE_JOINT = 0,
		DIMENSION = DIM
	};

public:
	AbstractJPathInterpolator(std::string userName, std::string email, std::string serialNo)
    : NRMKLeanFramework::AbstractComponent(userName, email, serialNo)
    {}
	virtual ~AbstractJPathInterpolator() {}

	virtual void initialize(double delt) = 0;

	virtual void setInitialTime(double t0) = 0;
	virtual void setTraj(double t0) = 0;
	virtual void traj(double time, VecType & posDes, VecType & velDes, VecType & accDes) = 0;
	virtual void setBoundaryCondAll(const VecType & vel_max, const VecType & acc_max) = 0;
//	virtual void setPath(const VecType *path, int len, const bool *nStopPoint, double maxDeviation = 0.0) = 0; // DEPRECATED
	virtual void setPath(const VecType *path, int len, double * maxDeviation) = 0;
	virtual void resetPath() {}
	virtual void setupTraj(int mode) {}
	virtual void setLoop(bool enable) {}

	virtual double getDuration() = 0;
	virtual int getNTotalSegment() = 0;
	virtual int	getCurrentWaypoint() = 0;

	virtual bool isTargetReached() = 0;
	virtual bool isTrajSetFailed() = 0;
	virtual bool isLoop() { return false; }


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif /* ABSTRACTJPATHINTERPOLATOR_HPP_ */
