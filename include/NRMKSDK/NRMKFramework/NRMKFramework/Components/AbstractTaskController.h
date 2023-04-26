/*
 * AbstractJointControl.hpp
 *
 *  Created on: Aug 20, 2018
 *      Author: ThachDo-MAC
 */

#ifndef ABSTRACTTASKCONTROLLER_HPP_
#define ABSTRACTTASKCONTROLLER_HPP_

#include <Poco/ClassLibrary.h>
#include <Poco/ClassLoader.h>
#include <Eigen/Eigen>
#include <Framework/CompositeHTransformTask.h>

#include <NRMKFramework/Components/AbstractComponent.h>

template<typename AbstractRobotType>
class AbstractTaskController : public NRMKLeanFramework::AbstractComponent
{
public:
	typedef AbstractRobotType ROBOT;
	typedef Poco::ClassLoader<ROBOT> RobotLoader;
	typedef typename ROBOT::JointVec JointVec;
	typedef NRMKFoundation::CompositeHTransformTaskPosition TaskPosition;
	typedef NRMKFoundation::CompositeHTransformTaskVelocity TaskVelocity;
	typedef NRMKFoundation::CompositeHTransformTaskVelocity TaskAcceleration;
	typedef Eigen::Matrix<double, 6, 1> TaskVec; //!< Typedef of task vector

public:
	AbstractTaskController(std::string userName, std::string email, std::string serialNo)
    : NRMKLeanFramework::AbstractComponent(userName, email, serialNo)
    , _robotLoader(NULL), _modelName("")
    {}
	virtual ~AbstractTaskController() {}

	/// ADDED @20190617
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Initialize the controller
	//! Note: This function is suitable for memory allocation and object instantiation (e.g. virtual robot object)
    //!
    //! \param delt Unit time step
	//  ----------------------------------------------------------
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
	virtual void setGains(TaskVec const & kp, TaskVec const & kv, TaskVec const & ki) {}

	/// ADDED @20190617
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Set the Impedance Gain for the Controller (optional)
	//!
	//! \param iMass Impedance Mass
	//! \param iDamping Impedance Damping
	//! \param iStiffness Impedance Stiffness
	//! \param iKi Integral Impedance Gain (optinal)
	//  ----------------------------------------------------------
	virtual void setImpedanceParams(TaskVec const & iMass, TaskVec const & iDamping, TaskVec const & iStiffness, TaskVec const & iKi = TaskVec::Zero()) {}

	/// ADDED @20190617
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Set the Impedance Gain for the Controller (optional)
	//!
	//! \param iMass Impedance Mass
	//! \param iDamping Impedance Damping
	//! \param iStiffness Impedance Stiffness
	//! \param iKi Integral Impedance Gain (optinal)
	//  ----------------------------------------------------------
	virtual void setImpedanceGains(TaskVec const & iKp, TaskVec const & iKv, TaskVec const & iKi, TaskVec const & iKf = TaskVec::Zero()) {}

	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Reset the overall task space controller
	//  ----------------------------------------------------------
	virtual void reset() {}

	/// ADDED @20190617
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Compute torque to compensate the gravitational force
	//!
	//! \param robot Robot object
	//! \param gravDir Gravity vector
	//! \param pDesired Desired task position
	//! \param pdotDesired Desired task velocity
	//! \param pddotDesired Desired task acceleration
	//! \param torque Computed control torque
	//  ----------------------------------------------------------
	virtual int computeControlTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, TaskPosition const & posDesired, TaskVelocity const & velDesired, TaskAcceleration const & accDesired, JointVec & torque) = 0;

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

#endif /* ABSTRACTTASKCONTROLLER_HPP_ */
