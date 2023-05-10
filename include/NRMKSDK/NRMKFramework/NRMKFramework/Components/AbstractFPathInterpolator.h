/*
 * AbstractFPathInterpolator.hpp
 *
 *  Created on: Jun 04, 2019
 *      Author: ThachDo-MAC
 */

#ifndef ABSTRACTFPATHINTERPOLATOR_HPP_
#define ABSTRACTFPATHINTERPOLATOR_HPP_

#include <Poco/ClassLibrary.h>
#include <Eigen/Eigen>

#include <Framework/CompositeHTransformTask.h>
#include <NRMKFramework/Components/AbstractComponent.h>

template<int DIM>
class AbstractFPathInterpolator : public NRMKLeanFramework::AbstractComponent
{
public:
	typedef NRMKFoundation::CompositeHTransformTaskPosition PosType;
	typedef NRMKFoundation::CompositeHTransformTaskVelocity VelType;
	typedef VelType AccType;
	typedef Eigen::Matrix<double, 2, 1> VecType2D;

	enum
	{
		DIMENSION = DIM
	};

public:
	AbstractFPathInterpolator(std::string userName, std::string email, std::string serialNo)
	: NRMKLeanFramework::AbstractComponent(userName, email, serialNo)
	{}
	virtual ~AbstractFPathInterpolator() {}

	virtual void initialize(double delt) = 0;

	virtual void setInitialTime(double t0) = 0;
	virtual void setTraj(double t0) = 0;
	virtual void traj(double time, PosType & pos, VelType & vel, AccType & acc) = 0;
	virtual void setBoundaryCondAll(VecType2D const & vel_max, VecType2D const & acc_max) = 0;
//	virtual void setPath(const PosType *path, const int len, const bool *nStopPoint, double maxDeviation = 0.0, const int *dirAngle = NULL, bool simulCheck = false) = 0; // DEPRECATED
	virtual void setPath(const PosType *path, const int len, double * maxDeviation, const int *dirAngle = NULL, bool simulCheck = false) = 0;
	virtual void resetPath() {}
	virtual void setupTraj(int mode) {}
	virtual void setLoop(bool enable) {}
	virtual void setReadyPos(PosType & pos) {}

	virtual double getDuration() = 0;
	virtual int getNTotalSegment() = 0;
	virtual int	getCurrentWaypoint() = 0;

	virtual bool isTargetReached() = 0;
	virtual bool isTrajSetFailed() = 0;
	virtual bool isLoop() { return false; }

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif /* ABSTRACTFPATHINTERPOLATOR_HPP_ */
