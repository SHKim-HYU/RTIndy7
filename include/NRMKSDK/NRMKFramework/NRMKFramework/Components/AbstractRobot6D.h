/*
 * AbstractRobot.h
 *
 *  Created on: Jun 04, 2019
 *      Author: ThachDo-MAC
 */

#ifndef ABSTRACTROBOT6D_H_
#define ABSTRACTROBOT6D_H_

#include <Indy/Indy6D.h>
#include <Poco/ClassLibrary.h>
#include <DefineConstant.h>
#include <Framework/CompositeHTransformTask.h>

#include <NRMKFramework/Components/AbstractComponent.h>

class AbstractRobot6D : public NRMKLeanFramework::AbstractComponent, public NRMKFoundation::IndySDK::IndyBase6D
{
public:
	enum
	{
		REF_BODY = -1,
		TARGET_BODY = 6,
	};
	typedef Eigen::Matrix<int, JOINT_DOF, 1> StateVec;	//!< Typedef of state vector
	typedef Eigen::Matrix<double, 2, 1> Vector2D;
	typedef NRMKFoundation::IndySDK::IndyBase6D::JointMat JointMat;
	typedef NRMKFoundation::IndySDK::IndyBase6D::JointVec JointVec;
	
	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \class CompositeHTransformTaskPosition
	//!
	//! \brief
	//! This defines the position variable for composite transform task kinematics. 
	//! 
	//! \details 
	//!	 Composite transform task positions are defined by the pair of rotation matrix and displacement vector. 
	//  ----------------------------------------------------------
	typedef NRMKFoundation::CompositeHTransformTaskPosition TaskPosition;
	
	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \class CompositeHTransformTaskVelocity
	//!
	//! \brief
	//! This defines the velocity variable for composite transform task kinematics. 
	//! 
	//! \details 
	//!	 Composite transform task velocity are defined by the pair of rotational velocity and translational velocity. 
	//  ----------------------------------------------------------
	typedef NRMKFoundation::CompositeHTransformTaskVelocity TaskVelocity;
	//wylee
    typedef NRMKFoundation::CompositeHTransformTaskVelocity TaskAcceleration;

	typedef Eigen::Matrix<double, 6, 1> TaskVec;		//!< Typedef of task vector
	typedef Eigen::Matrix<double, 6, 6> TaskJacobian;	//!< Typedef of task matrix

public:
	AbstractRobot6D(std::string userName, std::string email, std::string serialNo)
    : NRMKLeanFramework::AbstractComponent(userName, email, serialNo)
    , NRMKFoundation::IndySDK::IndyBase6D()
    {}
	virtual ~AbstractRobot6D() {}

	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Setting initial configuration before using other functions
	//  ----------------------------------------------------------
	virtual void setInitialConfiguration() {}

	/// ADDED @20190425
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Initialize Tool/End Effector properties
	//  ----------------------------------------------------------
	virtual void initToolProperties() = 0;

	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Cleanup the configurations after before program end
	//  ----------------------------------------------------------
	virtual void cleanup() {}
	
	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Read and setting the home position from the file
	//  ----------------------------------------------------------
	virtual bool setJointHome() = 0;

	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Setting the HTransform matrix from last joint to TCP
	//! 
	//! \param R Rotation matrix from last joint to TCP
	//! \param r Displacement vector from last joint to TCP
	//  ----------------------------------------------------------
	virtual void setTtarget(LieGroup::Rotation const & R, LieGroup::Displacement const & r){_Ttarget.set(R, r);};

	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Setting the HTransform matrix from global origin to Robot origin
	//! 
	//! \param R Rotation matrix from global origin to Robot origin
	//! \param r Displacement vector from global origin to Robot origin
	//  ----------------------------------------------------------
	virtual void setTReference(LieGroup::Rotation const & R, LieGroup::Displacement const & r){_Tref.set(R, r);};

	// wylee
    virtual void setTFTSensor(LieGroup::Rotation const & R, LieGroup::Displacement const & r){_Tft.set(R, r);};
    virtual void initHinfController(double delt) = 0;
    virtual void resetHinfController() = 0;
    virtual void setHinfControlGain(const JointVec &kp, const JointVec &kv, const JointVec &ki) = 0;
    //virtual void HinfController(LieGroup::Vector3D const & grav, JointVec const & qd, JointVec const & qdotd, JointVec const & qddotd, JointVec & torque) = 0;
    //virtual int HinfController(LieGroup::Vector3D const & grav, JointVec const & qd, JointVec const & qdotd, JointVec const & qddotd, JointVec & torque) = 0;
    //virtual int HinfController(LieGroup::Vector3D const & grav, TaskPosition const & pd, TaskVelocity const & pdotd, TaskAcceleration const & pddotd, JointVec & torque) = 0;
    virtual int HinfController(LieGroup::Vector3D const & grav, JointVec const & qd, JointVec const & qdotd, JointVec const & qddotd) = 0;
    virtual int HinfController(LieGroup::Vector3D const & grav, TaskPosition const & pd, TaskVelocity const & pdotd, TaskAcceleration const & pddotd) = 0;

    //wylee-admittance-control
    virtual void resetAdmittanceTraj(double delT) = 0;
    virtual void updateAdmittanceTraj(TaskPosition const & posDes, TaskVelocity const & velDes, TaskAcceleration const & accDes, TaskPosition & tposProxy, TaskVelocity & tvelProxy, TaskAcceleration & taccProxy, LieGroup::Wrench f_ext, LieGroup::Wrench f_ext_filtered, int mode) = 0;
    virtual void ForwardDynController(AbstractRobot6D & robotNom, JointMat & Mn, JointMat & Cn, JointVec & Gn, TaskPosition const & posDes, TaskVelocity const & velDes, LieGroup::Vector3D const & gravDir) = 0;
    virtual void setForwardDynControlGain(JointVec const & kp, JointVec const & kv, JointVec const & ki) = 0;
    virtual void initForwardDynController(double delt) = 0;
    virtual void resetForwardDynController() = 0;

	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Compute the task position and task velocity from robot pose
	//! 
	//! \param pos Task position
	//! \param vel Task velocity
	//  ----------------------------------------------------------
	virtual void computeFK(TaskPosition & pos, TaskVelocity & vel) = 0;
	
	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Compute the task position from robot pose
	//! 
	//! \param pos Task position
	//  ----------------------------------------------------------
	virtual void computeFK(TaskPosition & pos) = 0;

	//wylee
	virtual void initTaskErr(double delt) = 0;
    virtual void computeTaskErr(TaskPosition const & pos, TaskVelocity const & vel, TaskPosition const & posDes, TaskVelocity const & velDes, TaskVec & e, TaskVec & edot) = 0;

	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Compute the task position, task velocity, task Jacobian,
	//! and task Jacobian derivative
	//! 
	//! \param pos Task position
	//! \param vel Task velocity
	//! \param J Task Jacobian
	//! \param Jdot Task Jacobian derivative
	//  ----------------------------------------------------------
	virtual void computeJacobian(TaskPosition & pos, TaskVelocity & vel, TaskJacobian & J, TaskJacobian & Jdot) = 0;

	virtual void computeDynamicsParams(LieGroup::Vector3D const & gravDir, JointMat & M, JointMat & C, JointVec & G) = 0;

	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity and acceleration as well as task position and velocity error
	//!
	//! \details
	//! They are computed by \f$ \ddot{p}_{ref} = \ddot{p}_{des} + k_v \dot{e} + k_p e \f$ and  
	//!  \f$ \dot{p}_{ref} = \dot{p}_{des} + k_ve + kp\int e dt \f$, where 
	//! the position and velocity errors are defined by \f$ e = p_{des} - p \f$ and \f$ \dot{e} = \dot{p}_{des} - \dot{p} \f$.
	//!
	//! \param J Task Jacobian
	//! \param Jdot Differentiation of task Jacobian by time
	//! \param velRef Task reference velocity
	//! \param accRef Task reference acceleration
	//! \param qdotRef Joint reference velocity
	//! \param qddotRef Joint reference acceleration
	//  ----------------------------------------------------------
	virtual int computeJointAcc(TaskJacobian const & J, TaskJacobian const & Jdot, TaskVec & velRef, TaskVec & accRef, JointVec & qdotRef, JointVec & qddotRef) = 0;
		
	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity and acceleration as well as task position and velocity error
	//!
	//! \details
	//! They are computed by \f$ \ddot{p}_{ref} = \ddot{p}_{des} + k_v \dot{e} + k_p e \f$ and  
	//!  \f$ \dot{p}_{ref} = \dot{p}_{des} + k_ve + kp\int e dt \f$, where 
	//! the position and velocity errors are defined by \f$ e = p_{des} - p \f$ and \f$ \dot{e} = \dot{p}_{des} - \dot{p} \f$.
	//!
	//! \param J Task Jacobian
	//! \param Jdot Differentiation of task Jacobian by time
	//! \param accRef Task reference acceleration
	//! \param qddotRef Joint reference acceleration
	//  ----------------------------------------------------------
	virtual int computeJointAcc(TaskJacobian const & J, TaskJacobian const & Jdot, TaskVec & accRef, JointVec & qddotRef) = 0;

	virtual int computeJointVel(TaskJacobian const & J, TaskVec & velRef, JointVec & qdotRef) = 0;

	//wylee
	//virtual void setHinfController(JointVec const & kp, JointVec const & kv, JointVec const & ki) = 0;
	//virtual void resetHinfController() = 0;
	//virtual void HinfJointPosController(JointVec qDesired, JointVec qdotDesired, JointVec qddotDesired, JointVec & torque, LieGroup::Vector3D const & gravDir) = 0;

	LieGroup::HTransform const & Ttarget() const { return _Ttarget; }
	LieGroup::HTransform & Ttarget() { return _Ttarget; }

	LieGroup::HTransform const & Tref() const { return _Tref; }
	LieGroup::HTransform & Tref() { return _Tref; }

	LieGroup::HTransform const & Tft() const { return _Tft; }
	LieGroup::HTransform & Tft() { return _Tft; }

	LieGroup::HTransform const & Ttask_init() const { return _Ttask_init; }
	LieGroup::HTransform & Ttask_init() { return _Ttask_init; }

	double const & toolWeight() const { return _tWeight; }
	double & toolWeight()	{	return _tWeight;	}

	double const & toolXcom() const { return _tXcom; }
	double & toolXcom() { return _tXcom; }

	double const & toolYcom() const { return _tYcom; }
	double & toolYcom() { return _tYcom; }

	double const & toolZcom() const { return _tZcom; }
	double & toolZcom() { return _tZcom; }

	JointVec const & qhome() const { return _qhome; }
	JointVec & qhome() { return _qhome; }

	StateVec const & qState() const { return _qState; }
	StateVec & qState() { return _qState; }

	StateVec const & qBrake() const { return _qBrake; }
	StateVec & qBrake() { return _qBrake; }

	Vector2D const & mountAngles() const { return _mountAngles; }
	Vector2D & mountAngles() { return _mountAngles; }

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	StateVec _qState;
	StateVec _qBrake;
	JointVec _qhome;
	Vector2D _mountAngles;

	LieGroup::HTransform _Ttarget;
	LieGroup::HTransform _Tref;

	LieGroup::HTransform _Tft;

	LieGroup::HTransform _Ttask_init;

	double _tWeight, _tXcom, _tYcom, _tZcom;
};

#endif /* ABSTRACTROBOT6D_H_ */
