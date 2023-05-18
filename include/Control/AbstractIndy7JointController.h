/*
 * AbstractIndy7JointControl.hpp
 *
 *  Created on: Aug 20, 2018
 *      Author: ThachDo-MAC
 */

#ifndef ABSTRACTINDY7JOINTCONTROLLER_HPP_
#define ABSTRACTINDY7JOINTCONTROLLER_HPP_

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <json/value.h>

#include <Poco/ClassLibrary.h>
#include <Poco/ClassLoader.h>
#include <Eigen/Eigen>
#include <LieGroup/LieGroup.h>
#include <Poco/MetaObject.h>

template<typename AbstractRobotType>

class AbstractIndy7JointController
{
public:
	typedef AbstractRobotType ROBOT;
	typedef Poco::ClassLoader<ROBOT> RobotLoader;
	typedef typename ROBOT::JointVec JointVec;

public:
	AbstractIndy7JointController() : _robotLoader(NULL), _modelName("") {}
	virtual ~AbstractIndy7JointController() {}

	//virtual void initialize(double delt) = 0;
    virtual void initialize(ROBOT & robot, double delt) = 0;

	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Set the period for the controller
	//!
	//! \param delt Unit time step
	//  ----------------------------------------------------------
//	virtual void setPeriod(double delt) {}
	
	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Decide the controller consider the passivity or not
	//!
	//! \param enable Considering
	//  ----------------------------------------------------------
	virtual void setPassivityMode(bool enable) {}
	
	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Set the Gain for the Controller
	//!
	//! \param kp Position gain
	//! \param kv Velocity gain
	//! \param ki Inverse L-2 square gain
	//  ----------------------------------------------------------
	virtual void setGains(JointVec const & kp, JointVec const & kv, JointVec const & ki) {}
	
	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Reset the overall joint space controller
	//  ----------------------------------------------------------
	virtual void reset() {}
	
	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Reset the selected joint space controller
	//!
	//! \param jIndex Index of selected joint
	//  ----------------------------------------------------------
	virtual void reset(int jIndex) {}
	
	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Compute torque to compensate the gravitational force
	//!
	//! \param robot Robot object
	//! \param qDesired Desired joint position
	//! \param qdotDesired Desired joint velocity
	//! \param qddotDesired Desired joint acceleration
	//! \param torque Computed control torque
	//  ----------------------------------------------------------
	virtual int computeControlTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec const & qDesired, JointVec const & qdotDesired, JointVec const & qddotDesired, JointVec & torque) = 0;
	
	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Compute torque to compensate the gravitational force
	//!
	//! \param robot Robot object
	//! \param gravDir Gravity Direction
	//! \param torque Computed gravitational torque
	//  ----------------------------------------------------------
	virtual int computeGravityTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec & torque) = 0;

	virtual void startMotion(ROBOT & robot, LieGroup::Vector3D const & gravDir) {}
	virtual void endMotion(ROBOT & robot, LieGroup::Vector3D const & gravDir) {}

	/* FIXME Reorganize the API after testing */
	void setRobotLoader(RobotLoader * robotLoader, std::string modelName) { _robotLoader = robotLoader; _modelName = modelName; }
	ROBOT * createRobotObj() { return (_robotLoader != NULL) ? _robotLoader->create(_modelName) : NULL; }

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	RobotLoader * _robotLoader;
	std::string _modelName;
};

#endif /* ABSTRACTJOINTCONTROLLER_HPP_ */
