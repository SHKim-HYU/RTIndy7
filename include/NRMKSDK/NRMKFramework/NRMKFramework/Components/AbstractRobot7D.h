/*
 * AbstractRobot.h
 *
 *  Created on: Jun 04, 2019
 *      Author: ThachDo-MAC
 */

#ifndef ABSTRACTROBOT7D_H_
#define ABSTRACTROBOT7D_H_

#include <Indy/Indy7D.h>
#include <Poco/ClassLibrary.h>
#include <DefineConstant.h>
#include <Framework/CompositeHTransformTask.h>
#include <Framework/ExtendedTask.h>

#include <NRMKFramework/Components/AbstractComponent.h>

class AbstractRobot7D : public NRMKLeanFramework::AbstractComponent, public NRMKFoundation::IndySDK::IndyBase7D
{
public:
	enum
	{
		REF_BODY = -1,
		TARGET_BODY = 7,
	};
	typedef Eigen::Matrix<int, JOINT_DOF, 1> StateVec;		//!< Typedef of state vector
	typedef Eigen::Matrix<double, 2, 1> Vector2D;
	typedef NRMKFoundation::IndySDK::IndyBase7D::JointMat JointMat;
	typedef NRMKFoundation::IndySDK::IndyBase7D::JointVec JointVec;

	//  ---------------------- Doxygen info ----------------------
	//! \class Kinematics
	//!
	//! \brief
	//! This implements a base task kinematics class.
	//!
	//! \details
	//! Class Kinematics models any type of kinematics algorithm which computes the position-level task value and 
	//! the velocity-level task value as well as the Jacobian matrix and its derivative relating the joint velocity vector 
	//! and the velocity-level task vector for the given system. 
	//! They are called the task position, task velocity, and task Jacobian and task Jacobian derivative. 
	//! Since there may exists many types to represent task position, task velocity, and task accelerations, 
	//! they are specified as the template arguments. 
	//! As different system may have different kinematics algorithm, the type of the system is also a template parameter.
	//!
	//! In order to enforce the static polymorphism the derived class should inherit the class by the CRTP pattern. 
	//! See http://en.wikipedia.org/wiki/Curiously_recurring_template_pattern.
	//!  
	//! \tparam SubsysType Type of the subsys
	//! \tparam _DIM Dimension of the task variable
	//! \tparam KinematicsType Derived class 
	//! \tparam _PosType Type of the position-level task variable. 
	//!		The default type is Eigen::Matrix<double, _DIM, 1>.
	//! \tparam _VelType Type of the velocity-level task variable.
	//!		The default type is Eigen::Matrix<double, _DIM, 1>.
	//! \tparam _AccType Type of the acceleration-level task variable.
	//!		The default type is same as _VelType.
	////! \tparam _ForceType Type of the force-level task variable.
	////!		The default type is same as _VelType.
	//!
	//! \sa FunctionalKinematics
	//! \sa QuasiCoordKinematics
	//! \sa DisplacementKinematics
	//! \sa RotationKinematics
	//! \sa HTransformKinematics
	//  ----------------------------------------------------------
	typedef NRMKFoundation::CompositeAbsHTransformTaskKinematics<AbstractRobot7D> TaskKinematics;
	
	//  ---------------------- Doxygen info ----------------------
	//! \class ExtendedTaskPosition
	//!
	//! \brief
	//! This defines the position variable for  extended task kinematics. 
	//! 
	//! \details 
	//!	 Extended task positions are defined by extending the primary task position variable 
	//! with a null position variable (having no physical meaning.) 
	//!
	//! \tparam TaskKinematics Primary task kinematics
	//  ----------------------------------------------------------
	typedef NRMKFoundation::ExtendedTaskPosition<TaskKinematics> ExtendedPosition;

	//  ---------------------- Doxygen info ----------------------
	//! \class ExtendedTaskVelocity
	//!
	//! \brief
	//! This defines the velocity variable for  extended task kinematics. 
	//! 
	//! \details 
	//!	 Extended task velocities are defined by extending the primary task velocity variable 
	//! with a null velocity variable. The null velocity is defined by the null velocity 
	//! kinematics and it parameterizes the self-motion velocity which does not affect the task velocity.
	//!
	//! \tparam TaskKinematics Primary task kinematics
	//  ----------------------------------------------------------
	typedef NRMKFoundation::ExtendedTaskVelocity<TaskKinematics> ExtendedVelocity;

	//  ---------------------- Doxygen info ----------------------
	//! \class ExtendedTaskKinematics
	//!
	//! \brief
	//! This defines the extended task kinematics. 
	//! 
	//! \details 
	//!	 Extended task kinematics is defined by extending the primary task kinematics 
	//! with a null task kinematics which defines null motion behaviors of the redundant robots.
	//! It also computes the pseudoinverse of the pramary task Jacobian 
	//! since it is computed while computing the null space Jacobian matrix.
	//!
	//! \tparam TaskKinematics Primary task kinematics
	//!
	//! \sa NullVelocityKinematics
	//  ----------------------------------------------------------
	typedef NRMKFoundation::ExtendedTaskKinematics<TaskKinematics> ExtendedKinematics;

	typedef ExtendedKinematics::TaskVec ExtendedTaskVec;	//!< Typedef of task vector
	typedef ExtendedKinematics::TaskJac TaskJacobian;		//!< Typedef of task matrix
	
public:
	AbstractRobot7D(std::string userName, std::string email, std::string serialNo)
    : NRMKLeanFramework::AbstractComponent(userName, email, serialNo)
    , NRMKFoundation::IndySDK::IndyBase7D()
    {}
	virtual ~AbstractRobot7D() {}
	
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
	virtual void setTtarget(LieGroup::Rotation const & R, LieGroup::Displacement const & r) = 0; //{_Ttarget.set(R, r);};

	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Setting the HTransform matrix from global origin to Robot origin
	//! 
	//! \param R Rotation matrix from global origin to Robot origin
	//! \param r Displacement vector from global origin to Robot origin
	//  ----------------------------------------------------------
	virtual void setTReference(LieGroup::Rotation const & R, LieGroup::Displacement const & r) = 0; //{_Tref.set(R, r);};

	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Compute the task position and task velocity from robot pose
	//! 
	//! \param pos Task position
	//! \param vel Task velocity
	//  ----------------------------------------------------------
	virtual void computeFK(ExtendedPosition & pos, ExtendedVelocity & vel) = 0;
	
	/// ADDED @20180906
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Compute the task position from robot pose
	//! 
	//! \param pos Task position
	//  ----------------------------------------------------------
	virtual void computeFK(ExtendedPosition & pos) = 0;

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
	virtual void computeJacobian(ExtendedPosition & pos, ExtendedVelocity & vel, TaskJacobian & J, TaskJacobian & Jdot) = 0;

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
	virtual int computeJointAcc(TaskJacobian const & J, TaskJacobian const & Jdot, ExtendedTaskVec & velRef, ExtendedTaskVec & accRef, JointVec & qdotRef, JointVec & qddotRef) = 0;
		
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
	virtual int computeJointAcc(TaskJacobian const & J, TaskJacobian const & Jdot, ExtendedTaskVec & accRef, JointVec & qddotRef) = 0;
	virtual int computeJointVel(TaskJacobian const & J, ExtendedTaskVec & velRef, JointVec & qdotRef) = 0;

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

#endif /* ABSTRACTROBOT7D_H_ */
