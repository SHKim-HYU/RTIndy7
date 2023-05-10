//  ---------------------- Doxygen info ----------------------
//! \file Joint.h
//!
//! \brief
//! Header file for the class Joint (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a joint class
//! to be used for the interface of NRMKFoundation library
//! \n
//! \n
//! \n
//! \copydetails Neuromeka Foundation Library
//! \n
//! \n
//! \n
//! Neuromeka \n
//! South Korea\n
//! \n
//! http://www.neuromeka.com\n
//!
//! \date November 2016
//! 
//! \version 1.9
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!
//! \note 
//!  - v1.9.5(20161104) : Now it supports joint limit information
//!
//! \note Copyright (C) 2013-2016 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#include <limits>

#include "../NRMKCommon.h"

#include "LieGroup/LieGroup.h"

namespace NRMKFoundation
{

//  ---------------------- Doxygen info ----------------------
//! \enum 
//!
//! \brief enumerations for specifying joint type
//  ----------------------------------------------------------
enum JOINTTYPE 
{ 
	CUSTOM = 0,				//!< custom joint
	FLOATING = 1,			//!< floating joint
	RIGID,					//!< rigid or fixed joint
	FREE,					//!< free 6-dimensional joint
	REVOLUTE, 				//!< revolute joint
	PRISMATIC,				//!< prismatic joint
	CYLINDRICAL,			//!< cylindrical joint
	SPHERICAL,				//!< spherical joint
	MDH_REVOLUTE,			//!< revolute joint by modified DH notation
	MDH_PRISMATIC,			//!< prismatic joint by modified DH notation
	NUM_SUPPORTED_JOINTS	//!< the number of total supported joints
};		

//  ---------------------- Doxygen info ----------------------
//! \brief table for joint degrees-of-freedom for each joint type
//  ----------------------------------------------------------
static int JOINTDOFTABLE[NUM_SUPPORTED_JOINTS] = { -1, 6, 0, 6, 1, 1, 2, 3, 1, 1};

// forward declaration
class FloatingJoint;
class RigidJoint;
class RevoluteJoint;
class PrismaticJoint;
class CylindricalJoint;
class SphericalJoint;
class FreeJoint;
class MDHRevoluteJoint;
class MDHPrismaticJoint;

namespace internal 
{
//  ---------------------- Doxygen info ----------------------
//! \brief Trait class to determine whether a joint type is floating or not
//! 
//! \note 
//! Check value to see if the joint is floating or not.
//  ----------------------------------------------------------
template<typename JointType>
struct IsFloatingJoint
{
	static const bool value = false; //!< value is false for general joint types
};

template<>
struct IsFloatingJoint<FloatingJoint>
{
	static const bool value = true; //!< value is true for type FloatingJoint
};

//  ---------------------- Doxygen info ----------------------
//! \brief
//! Trait class for a joint type. 
//!
//! \details 
//! It consists of dof for the degrees-of-freedom, and type for the joint type. 
//! The value dof is the degrees-of-freedom of the joint which is added to a system's joint vector. 
//! For internal use, there are additional two values.
//! num_joint is used to detect the number of joints to be added to the system. 
//! system_dof is used to count the additional dof of the system contributed by the joint. 
//! 
//! \note 
//! The value system_dof is generally same as dof. The only exception is FloatingJoint,
//! where its joint variable is not taken into account in counting joint, but it is 
//! system's base coordinates.
//  ----------------------------------------------------------
template<typename JointType>
struct JointTrait 
{
	static const int dof = 0;				//!< degrees-of-freedom added to system's joint dof
	static const int system_dof = 0;		//!< degrees-of-freedom added to system's dof
	static const JOINTTYPE type = CUSTOM;	//!< joint type (of type JOINTTYPE)
	static const int num_joint = 1;			//!< the number of joints
};

//  ---------------------- Doxygen info ----------------------
//! \brief
//! Trait class for FloatingJoint 
//! \note 
//! It is used to represent the floating base body, so its dof 
//! is not counted, but is taken into account by the body transform
//  ----------------------------------------------------------
template<>
struct JointTrait<FloatingJoint>
{
	//! \note FloatingJoint is not considered as a joint.
	static const int dof = 0;
	static const int system_dof = 6;		
	static const JOINTTYPE type = FLOATING;
	static const int num_joint = 0;
};

//  ---------------------- Doxygen info ----------------------
//! \brief
//! Trait class for RigidJoint 
//  ----------------------------------------------------------
template<>
struct JointTrait<RigidJoint>
{
	static const int dof = 0;
	static const int system_dof = 0;		
	static const JOINTTYPE type = RIGID;
	/// FIXED @20160312
	static const int num_joint = 1;
};

//  ---------------------- Doxygen info ----------------------
//! \brief
//! Trait class for RevoluteJoint 
//  ----------------------------------------------------------
template<>
struct JointTrait<RevoluteJoint>
{
	static const int dof = 1;
	static const int system_dof = 1;		
	static const JOINTTYPE type = REVOLUTE;
	static const int num_joint = 1;
};

//  ---------------------- Doxygen info ----------------------
//! \brief
//! Trait class for PrismaticJoint 
//  ----------------------------------------------------------
template<>
struct JointTrait<PrismaticJoint>
{
	static const int dof = 1;
	static const int system_dof = 1;	
	static const JOINTTYPE type = PRISMATIC;
	static const int num_joint = 1;
};

//  ---------------------- Doxygen info ----------------------
//! \brief
//! Trait class for CylindricalJoint 
//  ----------------------------------------------------------
template<>
struct JointTrait<CylindricalJoint>
{
	static const int dof = 2;
	static const int system_dof = 2;	
	static const JOINTTYPE type = CYLINDRICAL;
	static const int num_joint = 1;
};

//  ---------------------- Doxygen info ----------------------
//! \brief
//! Trait class for SphericalJoint 
//  ----------------------------------------------------------
template<>
struct JointTrait<SphericalJoint>
{
	static const int dof = 3;
	static const int system_dof = 3;	
	static const JOINTTYPE type = SPHERICAL;
	static const int num_joint = 1;
};

//  ---------------------- Doxygen info ----------------------
//! \brief
//! Trait class for FreeJoint 
//  ----------------------------------------------------------
template<>
struct JointTrait<FreeJoint>
{
	static const int dof = 6;
	static const int system_dof = 6;	
	static const JOINTTYPE type = FREE;
	static const int num_joint = 1;
};

//  ---------------------- Doxygen info ----------------------
//! \brief
//! Trait class for MDHRevoluteJoint 
//  ----------------------------------------------------------
template<>
struct JointTrait<MDHRevoluteJoint>
{
	static const int dof = 1;
	static const int system_dof = 1;	
	static const JOINTTYPE type = MDH_REVOLUTE;
	static const int num_joint = 1;
};

//  ---------------------- Doxygen info ----------------------
//! \brief
//! Trait class for MDHPrismaticJoint 
//  ----------------------------------------------------------
template<>
struct JointTrait<MDHPrismaticJoint>
{
	static const int dof = 1;
	static const int system_dof = 1;	
	static const JOINTTYPE type = MDH_PRISMATIC;
	static const int num_joint = 1;
};
}

// Forward declaration
class FlexibleJoint;

/// FIXME @20151022
class JointFrictionAlgorithm
{
public:
	/// FIXME @20151022
	//  ---------------------- Doxygen info ----------------------
	//! \brief compute joint friction 
	//  ----------------------------------------------------------
	virtual void frictionTorque(double const * const q, double const * const qdot, double const * const tau, double * const tau_fric) const = 0;
};

template <int DOF = 1>
class JointFrictionAlgorithm_StictionViscous : public  JointFrictionAlgorithm
{
public:
	typedef Eigen::Matrix<double, DOF, 1> JointVec;

public:
	/// FIXME @20151022

	//! \brief sets the joint stiction
	//! \param stiction joint stiction value
	//! \param qdot_stationary joint stationary velocity threshold
	inline void setStiction(double stiction, double qdot_stationary = 1e-5)
	{
		_stiction.setConstant(stiction);
		_qdot_stationary.setConstant(qdot_stationary); 
	}

	inline void setStiction(JointVec const & stiction, JointVec const & qdot_stationary = 1e-5)
	{
		_stiction = stiction;
		_qdot_stationary = qdot_stationary; 
	}

	//! \brief sets the joint viscosity
	//! \param viscosity joint viscosity value
	inline void setViscosity(double viscosity)
	{
		_viscosity.setConstant(viscosity);
	}

	inline void setViscosity(JointVec const & viscosity)
	{
		_viscosity = viscosity;
	}

	//! \brief sets the joint viscosity
	//! \param mu joint Coulomb coefficient
	inline void setCoulomb(double mu)
	{
		_mu.setConstant(mu);
	}

	inline void setCoulomb(JointVec const & mu)
	{
		_mu = mu;
	}

	
	//! \return generated friction torque
	//! \param q joint position
	//! \param qdot joint velocity
	//! \param tau joint torque
	JointVec frictionTorque(JointVec const & q, JointVec const & qdot, JointVec const & tau) const
	{
		JointVec tau_fric;

		for (int k = 0; k < DOF; k++)
		{
			if (fabs(qdot[k]) <= _qdot_stationary[k])
			{
				double tau_f_max = _stiction[k]*NRMKFoundation::signum(tau[k]);

				if (fabs(tau[k]) < fabs(tau_f_max))
					tau_fric[k] = tau[k];
				else
					tau_fric[k] = tau_f_max;
			}
			else
			{
				tau_fric[k] = _stiction[k]*NRMKFoundation::signum(qdot[k]) + _viscosity[k]*qdot[k];
			}
		}

		return tau_fric;
	}

	//! \brief Compute joint friction torque
	//! \param q joint position
	//! \param qdot joint velocity
	//! \param tau joint torque
	//! \param tau_fric joint torque
	virtual void frictionTorque(double const * const q, double const * const qdot, double const * const tau, double * const tau_fric) const
	{
		for (int k = 0; k < DOF; k++)
		{
			if (fabs(qdot[k]) <= _qdot_stationary[k])
			{
				double tau_f_max = _stiction[k]*NRMKFoundation::signum(tau[k]);

				if (fabs(tau[k]) < fabs(tau_f_max))
					tau_fric[k] = tau[k];
				else
					tau_fric[k] = tau_f_max;
			}
			else
			{
				tau_fric[k] = _stiction[k]*NRMKFoundation::signum(qdot[k]) + _viscosity[k]*qdot[k];
			}
		}
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	/// FIXME @20151022
	//! \brief threshold joint stationary velocity
	JointVec				_qdot_stationary;

	//! \brief joint stiction
	JointVec				_stiction;

	//! \brief joint viscosity
	JointVec				_viscosity;

	//! \brief joint coulomb coefficient
	JointVec				_mu;
};

//  ---------------------- Doxygen info ----------------------
//! \class Joint
//!
//! \brief
//! This implements the base joint class interface to be derived 
//! for other concrete joint class for the interface of NRMKFoundation library.
//! 
//! \details
//! A joint is needed to connect two bodies, that is from the parent body 
//! to the child body. Relative motion of two bodies is described in terms of 
//! a pair of joint frames, each fixed to each body. That is, joint transformation and 
//! joint twist describes the transformation and the body twist of the child 
//! joint frame relative to the parent joint frame. 
//! It is regarded that the joint belongs to the child body.
//! The body frame of the child body is defined by the joint frame. 
//! The location and direction of the parent joint frame is specified by 
//! the homogeneous transformation of the parent joint frame relative to
//! the parent body frame, which is called the joint offset transformation. Then the transformation and twist of the child body frame 
//! relative to the parent body are called the adjacent transformation and adjacent twist, respectively.
//! When a joint connects body \f$ i \f$ and body \f$ k \f$, the parent joint frame is denoted by \f$ \{ iJk \} \f$.
//! Hence the adjacent transformation is given by \f$ {^{i}T_{k}} = {^{i}T_{iJk}} {^{iJk}T_{k}(q)} \f$
//! and the adjacent twist is given by  \f$ {^{i}V_{k}} = {^{iJk}V_{k}}(q) \f$. Note that \f$ q \f$ is
//! the joint vector. In this case, the joint twist is expressed as the linear transformation of the joint velocity vector
//! i.e. \f$ {^{iJk}V_{k}}(q) = E(q) \dot{q} \f$, where the coefficient matrix is called the joint matrix.
//! Mathematically, any joint is determined by the joint transformation and the joint matrix as well as the joint offset transformation. 
//  ----------------------------------------------------------
class Joint
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the joint whose joint type is specified
	//!
	//! \param type Type from JOINTTYPE
	//  ----------------------------------------------------------
	inline Joint(JOINTTYPE type) 
		: _type(type), _dof(JOINTDOFTABLE[_type])
		, _Toffset(), _offset(false)
		, _Tadj(), _Vadj(), _E(Matrix6d::Identity()), _Edot(Matrix6d::Zero())
		, _kinematic(false), _index(0)
		, _motor(NULL)
		, _fric(NULL)
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the joint whose joint type and the degrees-of-freedom are specified
	//!
	//! \param type Type from JOINTTYPE
	//! \param dof Joint dof
	//  ----------------------------------------------------------
	inline Joint(JOINTTYPE type, int dof) 
		: _type(type), _dof(dof), _Toffset(), _Tadj(), _Vadj(), _E(Matrix6d::Identity()), _Edot(Matrix6d::Zero()) 
		, _kinematic(false), _index(0)
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the joint dof
	//! 
	//! \return Joint dof
	//  ----------------------------------------------------------
	inline int dof() const 
	{ 
		return _dof; 
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief 
	//! returns the joint type 
	//! 
	//! \return Joint type
	//!
	//! \sa JOINTTYPE
	//  ----------------------------------------------------------
	inline JOINTTYPE type() const 
	{ 
		return _type; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the head index
	//! 
	//! \return head index
	//  ----------------------------------------------------------
	inline int index() const 
	{ 
		return _index; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the head index
	//! 
	//! \param index head index
	//  ----------------------------------------------------------
	inline void setIndex(int index) 
	{ 
		_index = index; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief 
	//! returns the flag whether the joint is kinematic
	//!
	//! \return true when the joint is kinematic
	//  ----------------------------------------------------------
	inline bool kinematic() const
	{ 
		return _kinematic;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the joint to be kinematic
	//! 
	//! \details 
	//! Kinematic joints is stationary and can change the position immediately 
	//! regardless of any dynamic effects.
	//  ----------------------------------------------------------
	inline void setKinematic(bool kinematic) 
	{ 
		_kinematic = kinematic; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the adjacent transformation for reading
	//! 
	//! \return Adjacent transformation
	//!
	//! \sa Liegroup::HTransform
	//  ----------------------------------------------------------
	inline LieGroup::HTransform const & Tadj() const 
	{ 
		return _Tadj;
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the adjacent twist for reading
	//! 
	//! \return Adjacent twist
	//!
	//! \sa Liegroup::Twist
	//  ----------------------------------------------------------
	inline LieGroup::Twist const & Vadj() const 
	{ 
		return _Vadj; 
	}	
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the adjacent transformation for writing
	//! 
	//! \return Adjacent transformation
	//!
	//! \sa Liegroup::HTransform
	//  ----------------------------------------------------------
	inline LieGroup::HTransform & Tadj() 
	{ 
		return _Tadj;
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the adjacent twist for writing
	//! 
	//! \return Adjacent twist
	//!
	//! \sa Liegroup::Twist
	//  ----------------------------------------------------------
	inline LieGroup::Twist & Vadj() { return _Vadj; }

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the current joint matrix for reading
	//!
	//! \details
	//! The joint twist can be expressed as linear transformation of the joint velocity vector. 
	//! The coefficient matrix is called the joint matrix. That is, \f$ {^{iJk}V_{k}} = E(q) \dot{q} \f$, where 
	//! \f$ E(q) \f$ has the \f$ n \f$ columns if the joint's dof is \f$ n \f$.
	//! 
	//! \note 
	//! The returned matrix is not 6-by-\f$ n \f$ but 6-by-6 dimensional matrix, whose the left columns 
	//! as many as the joint dof contains the joint matrix or \f$ E(q) \f$, while the remainder contains  
	//! the joint constraint matrix.
	//! 
	//! \return Joint matrix
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d const & E() const 
	{ 
		return _E;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the time-derivative of joint matrix for reading
	//!
	//! \note 
	//! The returned matrix is not 6-by-\f$ n \f$ but 6-by-6 dimensional matrix, whose the left columns 
	//! as many as the joint dof contains the derivative of the joint matrix or \f$ \dot{E}(q, \dot{q}) \f$, 
	//! while the remainder contains the derivative of the joint constraint matrix.
	//! 
	//! \return Joint matrix derivative
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d const & Edot() const
	{ 
		return _Edot;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the current joint matrix for writing
	//!
	//! \details
	//! The joint twist can be expressed as linear transformation of the joint velocity vector. 
	//! The coefficient matrix is called the joint matrix. That is, \f$ {^{iJk}V_{k}} = E(q) \dot{q} \f$, where 
	//! \f$ E(q) \f$ has the \f$ n \f$ columns if the joint's dof is \f$ n \f$.
	//! 
	//! \note 
	//! The returned matrix is not 6-by-\f$ n \f$ but 6-by-6 dimensional matrix, whose the left columns 
	//! as many as the joint dof contains the joint matrix or \f$ E(q) \f$, while the remainder contains  
	//! the joint constraint matrix.
	//! 
	//! \return Joint matrix
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d & E() 
	{ 
		return _E;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the time-derivative of joint matrix for writing
	//!
	//! \note 
	//! The returned matrix is not 6-by-\f$ n \f$ but 6-by-6 dimensional matrix, whose the left columns 
	//! as many as the joint dof contains the derivative of the joint matrix or \f$ \dot{E}(q, \dot{q}) \f$, 
	//! while the remainder contains the derivative of the joint constraint matrix.
	//! 
	//! \return Joint matrix derivative
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d & Edot()
	{ 
		return _Edot;
	}

// 	inline Eigen::Block<const Matrix6d, 6, dof> const & E() const { return _E.topLeftCorner<6, DOF>(); }
// 	inline Eigen::Block<const Matrix6d, 6, dof> const & Edot() const { return _Edot.topLeftCorner<6, DOF>(); }
// 
// 	inline void fillJacobian(Eigen::MatrixBase<Derived> & J, int i, int j) const
// 	{
// 		J.block<6, DOF>(i, j) = _E.topLeftCorner<6, DOF>();
// 	}
// 
// 	inline void fillJacobianDot(Eigen::MatrixBase<Derived> & Jdot, int i, int j) const
// 	{
// 		Jdot.block<6, DOF>(i, j) = _Edot.topLeftCorner<6, DOF>();
// 	}


	// FIXME Do we need to JF as a member ?
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the joint offset transformation for reading
	//!
	//! \details
	//! The joint offset transformation is the transformation of the parent joint frame relative to the parent body frame.
	//! That is, it is \f$ ^iT_{iJk} \f$ for the joint connecting the pareny bodt \f$ i \f$ to the child body \f$ k \f$.
	//! 
	//! \return Joint offset frame
	//!
	//! \sa Liegroup::HTransform
	//  ----------------------------------------------------------
	inline LieGroup::HTransform const & jointFrame() const 
	{ 
		return _Toffset; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the joint offset transformation for writing
	//!
	//! \details
	//! The joint offset transformation is the transformation of the parent joint frame relative to the parent body frame.
	//! That is, it is \f$ ^iT_{iJk} \f$ for the joint connecting the pareny bodt \f$ i \f$ to the child body \f$ k \f$.
	//! 
	//! \return Joint offset frame
	//!
	//! \sa Liegroup::HTransform
	//  ----------------------------------------------------------
	inline LieGroup::HTransform & jointFrame() 
	{ 
		return _Toffset; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns true when the joint has offset
	//  ----------------------------------------------------------
	inline bool hasOffset() const 
	{ 
		return _offset;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the joint offset transformation 
	//! 
	//! \param Toffset Joint offset transformation
	//!
	//! \sa Liegroup::HTransform
	//  ----------------------------------------------------------
	inline void setJointFrame(LieGroup::HTransform const & Toffset) 
	{ 
		// FIXME In case of MDH joints, _JF is the identity.
		_offset = true;
		_Toffset = Toffset; 
		// FIXED@20140628
		//_Tadj = _Toffset * _Tadj;
		// 		if (JointTrait<>::type != MDH && type() != FREE)	
		// 		{
		// 			_JF = JF; 
		// 			_Tadj = _JF * _Tadj;
		// 		}
	}

	// Joint algorithms
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the joint update algorithm for a given joint value array 
	//!
	//! \details
	//! It should update the joint matrix _E and the adjacent transform _Tadj
	//! for the given joint value array. Concrete joint classes should implement this.
	//! 
	//! \param q Array of the joint position values
	//  ----------------------------------------------------------
	virtual void update(double const * const q) = 0;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the joint update algorithm for a given joint position and velocity array 
	//!
	//! \details
	//! It should update the joint matrix _E and the adjacent transform _Tadj
	//! for the given joint position array as well as _Edot and _Vadj 
	//! for the given joint velocity array.
	//! Concrete joint classes should implement this.
	//! 
	//! \param q Array of the joint position values
	//! \param qdot Array of the joint velocity values 
	//  ----------------------------------------------------------
	virtual void update(double const * const q, double const * const qdot) = 0;

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the joint update algorithm for a given joint velocity array 
	//!
	//! \details
	//! It should update the joint matrix derivative _Edot and _Vadj 
	//! for the given joint velocity array.
	//! Concrete joint classes should implement this.
	//! 
	//! \param q Array of the joint position values
	//! \param qdot Array of the joint velocity values 
	//  ----------------------------------------------------------
	virtual void updateVelocity(double const * const q, double const * const qdot) = 0;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements an algorithm to compute the adjacent twist for the joint
	//! for a given joint velocity array 
	//!
	//! \details
	//! Adjacent twist is \f$ ^{i} V_{k} \f$. A most generic implementation is \f$ E \dot{q} \f$.
	//!  One can implement more optimized version of the formula.
	//! Concrete joint classes should implement this.
	//! 
	//! \note
	//! It should be called after update() has been called.
	//! 
	//! \param qdot Array of the joint velocity values
	//  ----------------------------------------------------------
	virtual LieGroup::Twist adjTwist(double const * const qdot) const = 0;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements an algorithm to compute the adjacent acceleration, or the derivative of the adjacent twist, for the joint
	//! for a given joint velocity and acceleration array 
	//!
	//! \details
	//! Adjacent acceleration is \f$ ^{i} \dot{V}_{k} \f$. A most generic implementation is 
	//! \f$ E \ddot{q} + \dot{E} \dot{q} \f$. One can 
	//! implement more optimized version of the formula.
	//! Concrete joint classes should implement this.
	//! 
	//! \note
	//! It should be called after update() has been called.
	//! 
	//! \param qdot Array of the joint velocity values 
	//! \param qddot Array of the joint acceleration values 
	//  ----------------------------------------------------------
	virtual LieGroup::Twist adjAcc(double const * const qdot, double const * const qddot) const = 0;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements an algorithm to compute the adjacent acceleration, or the derivative of the adjacent twist, for the joint
	//! for a given joint acceleration array assuming that joint velocity io zero
	//!
	//! \details
	//! Adjacent acceleration is \f$ ^{i} \dot{V}_{k} \f$. A most generic implementation is 
	//! \f$ E \ddot{q} + \dot{E} \dot{q} \f$, while this returns only the first part. One can 
	//! implement more optimized version of the formula.
	//! Concrete joint classes should implement this.
	//! 
	//! \note
	//! It should be called after update() has been called.
	//!
	//! \param qddot Array of the joint acceleration values 
	//  ----------------------------------------------------------
	virtual LieGroup::Twist adjAcc(double const * const qddot) const = 0;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements an algorithm to compute the bias acceleration algorithm for the joint
	//! for a given body twist of the child body 
	//!
	//! \details
	//! Bias acceleration is defined by \f$ -{\rm adj}_{^{i}V_{k}} V \f$ for a given body twist \f$ V \f$,
	//! where \f$ {^{i}V_{k}} \f$ is the adjacent twist. One can 
	//! implement more optimized version of the formula.
	//! Concrete joint classes should implement this.
	//! 
	//! \note
	//! It should be called after update() has been called.
	//! 
	//! \param V Body twist 
	//  ----------------------------------------------------------
	virtual LieGroup::Twist biasAcc(LieGroup::Twist const & V) const = 0;
	
	// project 6X6 matrix, i.e. E^T * A
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements an algorithm to project a 6-by-6 matrix to the joint motion subspace
	//!
	//! \details 
	//! For the joint matrix \f$ E \f$ and a six-dimensional symmetric matrix \f$ A \f$, let us define 
	//! \f$ A_E = A E \in \mathbb{R}^{6 \times n} \f$ 
	//! and \f$ \Lambda = E^T A E = A_E^T E \in \mathbb{R}^{n \times n} \f$. 
	//! One can implement more optimized version of the formula.
	//! Concrete joint classes should implement this.
	//! 
	//! \note
	//! It should be called after update() has been called.
	//! 
	//! \param A Six-dimensional symmetric matrix
	//! \param EA Pointer to the array containing \f$ A_E^T \f$ 
	//! \param Lambda Pointer to the array containing \f$ \Lambda \f$
	//  ----------------------------------------------------------
	virtual void project(Matrix6d const & A, double * const EA, double * const Lambda) const = 0;
	
	// project 6X6 matrix, i.e. E^T * A
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements an algorithm to project a 6-by-6 matrix to the joint motion subspace
	//!
	//! \details 
	//! For the joint matrix \f$ E \f$ and a six-dimensional symmetric matrix \f$ A \f$, let us define 
	//! \f$ A_E = A E \in \mathbb{R}^{6 \times n} \f$. 
	//! One can implement more optimized version of the formula.
	//! Concrete joint classes should implement this.
	//! 
	//! \note
	//! It should be called after update() has been called.
	//! 
	//! \param A Six-dimensional symmetric matrix
	//! \param EA Pointer to the array containing \f$ A_E^T \f$ 
	//  ----------------------------------------------------------
	virtual void project(Matrix6d const & A, double * const EA) const = 0;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the algorithm to compute the inverse dynamics of the joint for a given body wrench
	//!
	//! \details
	//! It is generically defined by \f$ E^T F \f$ for the body wrench \f$ F \f$. 
	//! One can implement more optimized version of the formula.
	//! Concrete joint classes should implement this.
	//! 
	//! \note
	//! It should be called after update() has been called.
	//! 
	//! \param F Required body wrench
	//! \param torque Pointer to the array containing the torque
	//  ----------------------------------------------------------
	virtual void idyn(LieGroup::Wrench const & F, double * const torque) const = 0;
	
	//  ---------------------- Doxygen info ----------------------
	//! \return
	//! an array containing joint stiffness values
	//  ----------------------------------------------------------
	virtual double const * const stiffness() const = 0;

	//  ---------------------- Doxygen info ----------------------
	//! \return
	//! an array containing joint damping values
	//  ----------------------------------------------------------
	virtual double const * const damping() const = 0;

	//  ---------------------- Doxygen info ----------------------
	//! \return
	//! an array containing joint neutral position values
	//  ----------------------------------------------------------
	virtual double const * const neutralPos() const = 0;


	/// FIXME @20150807
	//! \brief sets the flexible joint
	//! \param motor flexible joint
	inline void setFlexibleJoint(FlexibleJoint *motor)
	{
		_motor = motor; 
	}

	//! \return the flexible joint
	//! \note 'NULL' means no flexible joint
	FlexibleJoint * flexibleJoint() const 
	{
		return _motor;
	}

	/// FIXME @20151022
	//! \brief sets the joint friction algorithm
	//! \param fric flexible joint
	inline void setFriction(JointFrictionAlgorithm *fric)
	{
		_fric = fric; 
	}

	//! \return the joint friction algorithm
	//! \note 'NULL' means no joint friction
	JointFrictionAlgorithm * friction() const 
	{
		return _fric;
	}
	
	void frictionTorque(double const * const q, double const * const qdot, double const * const tau, double * const tau_fric) const
	{
		if (_fric)
			_fric->frictionTorque(q, qdot, tau, tau_fric);		
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Joint type (JOINTTYPE enum constant)
	//  ----------------------------------------------------------
	JOINTTYPE _type;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Joint dof
	//  ----------------------------------------------------------
	int _dof;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Joint offset transformation 
	//!
	//! \sa LieGroup::HTransform
	//  ----------------------------------------------------------
	LieGroup::HTransform	_Toffset;	// Joint frame
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Flag to determine whether the joint has offset transformation or not (default = false)
	//  ----------------------------------------------------------
	bool _offset;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Adjacent transformation
	//!
	//! \sa LieGroup::HTransform
	//  ----------------------------------------------------------
	LieGroup::HTransform	_Tadj;	// adjacent kinematics
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Adjoint twist 
	//!
	//! \sa LieGroup::HTransform
	//  ----------------------------------------------------------
	LieGroup::Twist			_Vadj;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Joint matrix including joint constraint matrix
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	Matrix6d				_E;		// Joint matrix
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Derivative of the joint matrix including joint constraint matrix
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	Matrix6d				_Edot;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Flag to determine whether the joint is kinematic or not (default = false)
	//  ----------------------------------------------------------
	bool _kinematic;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Head index in the system joint vector 
	//  ----------------------------------------------------------
	int _index;


	//! \brief  Flexible joint
	FlexibleJoint			*_motor;

	//! \brief Joint friction algorithm
	JointFrictionAlgorithm	*_fric;
};

/// FIXME @20161104
//  ---------------------- Doxygen info ----------------------
//! \class JointLimit
//!
//! \brief
//! This implements joint limit handler.
//!
//! \details 
//! By default there are no limits in positon, velocity, and torque.
//! That is, every lower limit vectors is initialized as negative infinity,
//! while every upper limit vectors as positive infinity. 
//  ----------------------------------------------------------
template<int DOF>
class JointLimit
{
public:
	typedef Eigen::Matrix<double, DOF, 1> JointVec;

public:
	JointLimit()
	{
		_q_llimit.setConstant(-std::numeric_limits<double>::infinity());
		_q_ulimit.setConstant(std::numeric_limits<double>::infinity());

		_qdot_llimit.setConstant(-std::numeric_limits<double>::infinity());
		_qdot_ulimit.setConstant(std::numeric_limits<double>::infinity());

		_tau_llimit.setConstant(-std::numeric_limits<double>::infinity());
		_tau_ulimit.setConstant(std::numeric_limits<double>::infinity());
	}
	
	// Setting limit by vector 
	//! \brief sets the position lower limit vector
	//! \param q_llimit position lower limit vector
	void setPosLimitLower(JointVec const & q_llimit)
	{
		_q_llimit = q_llimit;
	}

	//! \brief sets the position upper limit vector
	//! \param q_ulimit position upper limit vector
	void setPosLimitUpper(JointVec const & q_ulimit)
	{
		_q_ulimit = q_ulimit;
	}

	//! \brief sets the velocity lower limit vector
	//! \param qdot_llimit velocity lower limit vector
	void setVelLimitLower(JointVec const & qdot_llimit)
	{
		_qdot_llimit = qdot_llimit;
	}

	//! \brief sets the velocity upper limit vector
	//! \param qdot_ulimit velocity upper limit vector
	void setVelLimitUpper(JointVec const & qdot_ulimit)
	{
		_qdot_ulimit = qdot_ulimit;
	}

	//! \brief sets the actuator lower limit vector
	//! \param tau_llimit actuator lower limit vector
	void setActLimitLower(JointVec const & tau_llimit)
	{
		_tau_llimit = tau_llimit;
	}

	//! \brief sets the actuator upper limit vector
	//! \param tau_ulimit actuator upper limit vector
	void setActLimitUpper(JointVec const & tau_ulimit)
	{
		_tau_ulimit = tau_ulimit;
	}

	// Setting limit by component

	//! \brief sets the position lower limit component
	//! \param q_llimit position lower limit component
	void setPosLimitLower(unsigned int k, double q_llimit)
	{
		_q_llimit[k] = q_llimit;
	}

	//! \brief sets the position upper limit component
	//! \param q_ulimit position upper limit component
	void setPosLimitUpper(unsigned int k, double q_ulimit)
	{
		_q_ulimit[k] = q_ulimit;
	}

	//! \brief sets the position lower limit component
	//! \param qdot_llimit position lower limit component
	void setVelLimitLower(unsigned int k, double qdot_llimit)
	{
		_qdot_llimit[k] = qdot_llimit;
	}

	//! \brief sets the position upper limit component
	//! \param qdot_ulimit position upper limit component
	void setVelLimitUpper(unsigned int k, double qdot_ulimit)
	{
		_qdot_ulimit[k] = qdot_ulimit;
	}

	//! \brief sets the position lower limit component
	//! \param tau_llimit position lower limit component
	void setActLimitLower(unsigned int k, double tau_llimit)
	{
		_tau_llimit[k] = tau_llimit;
	}

	//! \brief sets the position upper limit component
	//! \param tau_ulimit position upper limit component
	void setActLimitUpper(unsigned int k, double tau_ulimit)
	{
		_tau_ulimit[k] = tau_ulimit;
	}

	// Setting limit of the one-dof joint
	void setPosLimitLower(double q_llimit)
	{
		setPosLimitLower(0, q_llimit);
	}

	void setPosLimitUpper(double q_ulimit)
	{
		setPosLimitUpper(0, q_ulimit);
	}

	void setVelLimitLower(double qdot_llimit)
	{
		setVelLimitLower(0, qdot_llimit);
	}

	void setVelLimitUpper(double qdot_ulimit)
	{
		setVelLimitUpper(0, qdot_ulimit);
	}

	void setActLimitLower(double tau_llimit)
	{
		setActLimitLower(0, tau_llimit);
	}

	void setActLimitUpper(double tau_ulimit)
	{
		setActLimitUpper(0, tau_ulimit);
	}

	// Limit checking functions
	
	//! \brief check whether the k-th joint position component is over-/under-/un-limited 
	//! \note if q violates any limits, its value is clipped. 
	//!
	//! \return 1 if upper-limit is violated
	//! \return -1 if lower limit is violated
	//! \return 0 if no limit is violated
	//! \param k the joint index 
	//! \param q the test joint value 
	int isPosLimited(unsigned int k, double & q) const 
	{
		if (q > _q_ulimit[k])
		{
			q = _q_ulimit[k];
			return 1;
		}
		else if (q < _q_llimit[k])
		{
			q = _q_llimit[k];
			return -1;
		}
		else
			return 0;
	}

	//! \brief check whether the k-th joint velocity component is over-/under-/un-limited 
	//! \note if qdot violates any limits, its value is clipped. 
	//!
	//! \return 1 if upper-limit is violated
	//! \return -1 if lower limit is violated
	//! \return 0 if no limit is violated
	//! \param k the joint index 
	//! \param qdot the test joint value 
	int isVelLimited(unsigned int k, double qdot) const 
	{
		if (qdot > _qdot_ulimit[k])
		{
			qdot = _qdot_ulimit[k];
			return 1;
		}
		else if (qdot < _qdot_llimit[k])
		{
			qdot = _qdot_llimit[k];
			return -1;
		}
		else
			return 0;
	}

	//! \brief check whether the k-th joint velocity component is over-/under-/un-limited 
	//!		while taking into account joint position limit at the same time.
	//! \note if qdot violates any limits, its value is clipped. 
	//!
	//! \return 1 if upper-limit is violated
	//! \return -1 if lower limit is violated
	//! \return 0 if no limit is violated
	//! \param k the joint index 
	//! \param qdot the test joint value 
	int isPosVelLimited(unsigned int k, double qdot, double q, double dT) const 
	{
		double ulimit = (_q_ulimit[k] - q) / dT;
		if (ulimit > _qdot_ulimit[k])
			ulimit = _qdot_ulimit[k];

		if (qdot > ulimit)
		{
			qdot = ulimit;
			return 1;
		}
		else 
		{
			double llimit = (_q_llimit[k] - q) / dT;
			if (llimit < _qdot_llimit[k])
				llimit = _qdot_llimit[k];
				
			if (qdot < llimit)
			{
				qdot = llimit;
				return -1;
			}
			else
				return 0;
		}
	}

	//! \brief check whether the k-th joint actuator component is over-/under-/un-limited 
	//! \note if tau violates any limits, its value is clipped. 
	//!
	//! \return 1 if upper-limit is violated
	//! \return -1 if lower limit is violated
	//! \return 0 if no limit is violated
	//! \param k the joint index 
	//! \param tau the test joint value 
	int isActLimited(unsigned int k, double tau) const 
	{
		if (tau > _tau_ulimit[k])
		{
			tau = _tau_ulimit[k];
			return 1;
		}
		else if (tau < _tau_llimit[k])
		{
			tau = _tau_llimit[k];
			return -1;
		}
		else
			return 0;
	}
	
	// Limit checking functions for one-dof joints

	int isPosLimited(double q) const 
	{
		return isPosLimited(0, q);
	}

	int isVelLimited(double qdot) const 
	{
		return isVelLimited(0, qdot);
	}

	int isPosVelLimited(double qdot, double q, double dT) const 
	{
		return isPosVelLimited(0, qdot, q, dT);
	}	

	int isActLimited(double tau) const 
	{
		return isActLimited(0, tau);
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//! \brief position lower limit
	JointVec		_q_llimit;
	//! \brief position upper limit
	JointVec		_q_ulimit;

	//! \brief velocity lower limit
	JointVec		_qdot_llimit;
	//! \brief velocity upper limit
	JointVec		_qdot_ulimit;

	//! \brief actuator lower limit
	JointVec		_tau_llimit;
	//! \brief actuator upper limit
	JointVec		_tau_ulimit;
};

//  ---------------------- Doxygen info ----------------------
//! \class JointBase
//!
//! \brief
//! This implements a joint base class inheriting class Joint.
//! 
//! \note
//! Users should inherit this class to define a customized joint.
//! 
//! \sa FixedJoint
//! \sa FreeJoint
//! \sa RevoluteJoint
//! \sa PrismaticJoint
//! \sa CylindricalJoint
//! \sa SphericalJoint
//! \sa FloatingJoint
//! \sa MDHRevoluteJoint
//! \sa MDHPrismaticJoint
//  ----------------------------------------------------------
template<typename JointType>
class JointBase : public Joint
{
public:
	inline JointBase() : Joint(internal::JointTrait<JointType>::type), _h(0)
		/// FIXME @20161104
		, _limit()
	{
		_K.setZero();
		_D.setZero();
		_q_neutral.setZero();
	}

	inline virtual ~JointBase() {}

	/** \returns a reference to the derived object */
	inline JointType& derived() { return *static_cast<JointType*>(this); }
	/** \returns a const reference to the derived object */
	inline const JointType& derived() const { return *static_cast<const JointType*>(this); }

// 	inline Derived& const_cast_derived() const { return *static_cast<Derived*>(const_cast<JointBase*>(this)); }
// 	inline const Derived& const_derived() const { return *static_cast<const Derived*>(this); }

	enum 
	{
		DOF = internal::JointTrait<JointType>::dof,
		TYPE = internal::JointTrait<JointType>::type
	};

	typedef Eigen::Matrix<double, DOF, 1> JointVec;
	typedef Eigen::Matrix<double, DOF, DOF> SqJointMat;
	typedef Eigen::Matrix<double, 6, DOF> JointMat;
	typedef Eigen::Matrix<double, DOF, 6> JointMatTr;

// 	inline Eigen::Block<const Matrix6d, 6, DOF> const & E() const { return _E.topLeftCorner<6, DOF>(); }
// 	inline Eigen::Block<const Matrix6d, 6, DOF> const & Edot() const { return _Edot.topLeftCorner<6, DOF>(); }

// 	inline void fillJacobian(Eigen::MatrixBase<Derived> & J, int i, int j) const
// 	{
// 		J.block<6, DOF>(i, j) = _E.topLeftCorner<6, DOF>();
// 	}
// 
// 	inline void fillJacobianDot(Eigen::MatrixBase<Derived> & Jdot, int i, int j) const
// 	{
// 		Jdot.block<6, DOF>(i, j) = _Edot.topLeftCorner<6, DOF>();
// 	}

	//  ---------------------- Doxygen info ----------------------
	//! \return
	//! an array containing joint stiffness values
	//  ----------------------------------------------------------
	inline virtual double const * const stiffness() const
	{
		return _K.data();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \return
	//! an array containing joint damping values
	//  ----------------------------------------------------------
	inline virtual double const * const damping() const
	{
		return _D.data();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \return
	//! an array containing joint neutral position values
	//  ----------------------------------------------------------
	inline virtual double const * const neutralPos() const
	{
		return _q_neutral.data();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the uniform joint damping
	//! 
	//! \param d joint damping value
	//  ----------------------------------------------------------
	inline void setDamping(double d)
	{
		_D.setConstant(d);
	}

	//! \brief sets the uniform joint stiffness
	//! \param k joint stiffness value
	inline void setStiffness(double k)
	{
		_K.setConstant(k);
	}

	//! \brief sets the joint damping
	//! \param d joint damping vector
	inline void setDamping(JointVec const & d)
	{
		_D = d;
	}

	//! \brief sets the joint stiffness
	//! \param k joint stiffness vector
	inline void setStiffness(JointVec const & k)
	{
		_K = k; 
	}

	//! \brief sets the uniform joint neutral position vector
	//! \param q joint neutral position value
	inline void setNeutralPos(double q)
	{
		_q_neutral.setConstant(q);
	}

	//! \brief sets the joint neutral position vector
	//! \param q joint neutral position vector
	inline void setNeutralPos(JointVec const & q)
	{
		_q_neutral = q;
	}

	//! \brief sets the integration period
	//! \param h integration period
	inline void setPeriod(double h)
	{
		_h = h; 
	}

	//! \return added inertia to link
	JointVec addedInertia() const
	{
		return _h*_D;
	}

	//! \return generated joint torque 
	JointVec addedTorque(JointVec const & q, JointVec const & qdot) const
	{
		return _K.cwiseProduct(_q_neutral - q) - (_D + _h*_K).cwiseProduct(qdot);
	}

// 	/// FIXME @20151022
// 
// 	//! \brief sets the joint stiction
// 	//! \param stiction joint stiction value
// 	//! \param qdot_stationary joint stationary velocity threshold
// 	inline void setStiction(double stiction, double qdot_stationary = 1e-5)
// 	{
// 		_stiction.setConstant(stiction);
// 		_qdot_stationary.setConstant(qdot_stationary); 
// 	}
// 
// 	inline void setStiction(JointVec const & stiction, JointVec const & qdot_stationary = 1e-5)
// 	{
// 		_stiction = stiction;
// 		_qdot_stationary = qdot_stationary; 
// 	}
// 
// 	//! \brief sets the joint viscosity
// 	//! \param viscosity joint viscosity value
// 	inline void setViscosity(double viscosity)
// 	{
// 		_viscosity.setConstant(viscosity);
// 	}
// 
// 	inline void setViscosity(JointVec const & viscosity)
// 	{
// 		_viscosity = viscosity;
// 	}
// 
// 	//! \brief sets the joint viscosity
// 	//! \param mu joint Coulomb coefficient
// 	inline void setCoulomb(double mu)
// 	{
// 		_mu.setConstant(mu);
// 	}
// 
// 	inline void setCoulomb(JointVec const & mu)
// 	{
// 		_mu = mu;
// 	}
// 
// 	//! \return generated friction torque
// 	//! \param q joint position
// 	//! \param qdot joint velocity
// 	//! \param tau joint torque
// 	inline virtual JointVec frictionTorque(JointVec const & q, JointVec const & qdot, JointVec const & tau) const
// 	{
// 		JointVec tau_fric;
// 
// 		for (int k = 0; k < DOF; k++)
// 		{
// 			if (fabs(qdot[k]) <= _qdot_stationary[k])
// 			{
// 				double tau_f_max = _stiction*NRMKFoundation::signum(tau[k]);
// 
// 				if (fabs(tau[k]) < fabs(tau_f_max))
// 					tau_fric[k] = tau[k];
// 				else
// 					tau_fric[k] = tau_f_max;
// 			}
// 			else
// 			{
// 				tau_fric[k] = _stiction*NRMKFoundation::signum(qdot[k]) + _viscosity*qdot[k];
// 			}
// 		}
// 
// 		return tau_fric;
// 	}

	/// FIXME @20161104
	//! \return joint limit handler
	JointLimit<DOF> & limit() 
	{
		return _limit;
	}

	//! \return joint limit handler
	JointLimit<DOF> const & limit() const
	{
		return _limit;
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
	//! \brief Joint damping vector
	JointVec				_D;

	//! \brief Joint stiffness vector
	JointVec				_K;

	//! \brief Join neutral position vector for stiffness
	JointVec				_q_neutral;

	//! \brief Integration period
	double					_h;

// 	/// FIXME @20151022
// 	//! \brief threshold joint stationary velocity
// 	JointVec				_qdot_stationary;
// 
// 	//! \brief joint stiction
// 	JointVec				_stiction;
// 
// 	//! \brief joint viscosity
// 	JointVec				_viscosity;
// 
// 	//! \brief joint coulomb coefficient
// 	JointVec				_mu;

	/// FIXME @20161104
	JointLimit<DOF>			_limit; 
};

//  ---------------------- Doxygen info ----------------------
//! \class FloatingJoint
//!
//! \brief
//! This implements a floating joint class which describes the motion of a rigid body.
//! This joint is used for the floating base body.
//! 
//! \sa JointBase
//  ----------------------------------------------------------
class FloatingJoint : public JointBase<FloatingJoint>
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a default floating joint
	//  ----------------------------------------------------------
	inline FloatingJoint() : JointBase<FloatingJoint>() { }

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a floating joint whose initial adjacent transformation is specified
	//! 
	//! \param T0 Initial adjacent transformation
	//  ----------------------------------------------------------
	inline FloatingJoint(LieGroup::HTransform const & T0) : JointBase<FloatingJoint>() { _Tadj = T0; }

	inline virtual void update(double const * const q) 
	{
		assert(false);
// 		Tadj().R() << q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8];
// 		Tadj().r() << q[9], q[10], q[11];
	}

	inline virtual void update(double const * const q, double const * const qdot)
	{
		assert(false);
// 		update(q);
// 		Vadj() << qdot[0], qdot[1], qdot[2], qdot[3], qdot[4], qdot[5];
	}

	inline virtual void updateVelocity(double const * const q, double const * const qdot)
	{
		assert(false);
		// 		update(q);
		// 		Vadj() << qdot[0], qdot[1], qdot[2], qdot[3], qdot[4], qdot[5];
	}

	inline virtual LieGroup::Twist adjTwist(double const * const qdot) const
	{
		assert(false);
		return LieGroup::Twist(qdot[0], qdot[1], qdot[2], qdot[3], qdot[4], qdot[5]);
	}

	inline virtual LieGroup::Twist adjAcc(double const * const qdot, double const * const qddot) const
	{
		return adjAcc(qddot);
	}

	inline virtual LieGroup::Twist adjAcc(double const * const qddot) const
	{
		assert(false);
		return LieGroup::Twist(qddot[0], qddot[1], qddot[2], qddot[3], qddot[4], qddot[5]);
	}

	inline virtual LieGroup::Twist biasAcc(LieGroup::Twist const & V) const
	{
		return V.adjoint(Vadj());
	}

	inline virtual void idyn(LieGroup::Wrench const & F, double * const torque) const
	{ 
		Eigen::Map<Vector6d> tau(torque);
		tau = F;
	}

	inline virtual void project(Matrix6d const & A, double * const EA, double * const lambda) const
	{
		project(A, EA); 

		Eigen::Map<Matrix6d> Lambda(lambda);
		Lambda = A;
	}

	inline virtual void project(Matrix6d const & A, double * const EA) const 
	{
		Eigen::Map<Matrix6d> ea(EA);
		ea = A;
	}
};

//  ---------------------- Doxygen info ----------------------
//! \class RigidJoint
//!
//! \brief
//! This implements a rigid or fixed joint class. 
//! This joint is used to fix the base body to the ground.
//! 
//! \sa JointBase
//  ----------------------------------------------------------
class RigidJoint : public JointBase<RigidJoint>
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a default rigid joint
	//  ----------------------------------------------------------
	inline RigidJoint() : JointBase<RigidJoint>() { }
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a rigid joint whose adjacent transformation is specified
	//! 
	//! \param T0 Adjacent transformation
	//  ----------------------------------------------------------
	inline RigidJoint(LieGroup::HTransform const & T0) : JointBase<RigidJoint>() 
	{ 
		_Tadj = T0; 
	}

	inline void set(LieGroup::Rotation const & R0, LieGroup::Displacement const & r0)
	{
		_Tadj.r() = r0;
		_Tadj.R() = R0; 
	}

	inline void setOrigin(LieGroup::Rotation const & R0, LieGroup::Displacement const & r0)
	{
		_Tadj = LieGroup::HTransform(R0, r0).cascade(_Tadj);
	}

	inline virtual void update(double const * const q) 
	{
	}

	inline virtual void update(double const * const q, double const * const qdot)
	{
	}

	inline virtual void updateVelocity(double const * const q, double const * const qdot)
	{
	}

	inline virtual LieGroup::Twist adjTwist(double const * const qdot) const
	{
		return LieGroup::Twist();
	}

	inline virtual LieGroup::Twist adjAcc(double const * const qdot, double const * const qddot) const
	{
		return LieGroup::Twist();
	}

	inline virtual LieGroup::Twist adjAcc(double const * const qddot) const
	{
		return LieGroup::Twist();
	}

	inline virtual LieGroup::Twist biasAcc(LieGroup::Twist const & V) const
	{
		return LieGroup::Twist();
	}

	inline virtual void idyn(LieGroup::Wrench const & F, double * const torque) const
	{	// do nothing, since DOF = 0
	}

	inline virtual void project(Matrix6d const & A, double * const EA, double * const lambda) const
	{	// do nothing, since DOF = 0
	}

	inline virtual void project(Matrix6d const & A, double * const EA) const 
	{		// do nothing, since DOF = 0
	}
};

//  ---------------------- Doxygen info ----------------------
//! \class RevoluteJoint
//!
//! \brief
//! This implements a one dof revolute joint class.
//!
//! \sa JointBase
//  ----------------------------------------------------------
class RevoluteJoint : public JointBase<RevoluteJoint>
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a revolute joint having the initial joint angle
	//! 
	//! \param q0 Initial joint angle (in radian)
	//  ----------------------------------------------------------
	inline RevoluteJoint(double q0 = 0.0) : JointBase<RevoluteJoint>()
	{
		update(&q0);

		Eigen::Matrix<int, 6, 1> indices;
		indices << 5, 0, 1, 2, 3, 4;
		_E = _E * indices.asPermutation();
	}

	inline virtual void update(double const * const q)
	{
		double c = cos(q[0]);
		double s = sin(q[0]);

		// FIXED@20150127
		//_Tadj.R().topLeftCorner<3,2>() = _Toffset.R().topLeftCorner<3,2>()*(Eigen::Matrix2d() << c, -s, s, c).finished();

		LieGroup::Rotation R(c, -s, 0, s, c, 0, 0, 0, 1);

		_Tadj.R() = _Toffset.R()*R;
		_Tadj.r() = _Toffset.r();
	}

	inline virtual void update(double const * const q, double const * const qdot) 
	{ 
		update(q);
		//_Vadj(5) = qdot[0];
		updateVelocity(q, qdot);
	}

	inline virtual void updateVelocity(double const * const q, double const * const qdot) 
	{ 
		_Vadj(5) = qdot[0];
	}

	inline virtual LieGroup::Twist adjTwist(double const * const qdot) const
	{
		return LieGroup::Twist(0, 0, 0, 0, 0, qdot[0]);
	}

	inline virtual LieGroup::Twist adjAcc(double const * const qdot, double const * const qddot) const
	{
		return adjAcc(qddot);
	}

	inline virtual LieGroup::Twist adjAcc(double const * const qddot) const
	{
		return LieGroup::Twist(0, 0, 0, 0, 0, qddot ? qddot[0] : 0);
	}

	inline virtual LieGroup::Twist biasAcc(LieGroup::Twist const & V) const
	{
		double qdot = _Vadj(5); 
		return LieGroup::Twist(V(1)*qdot, -V(0)*qdot, 0, V(4)*qdot, -V(3)*qdot, 0); 
	}

	inline virtual void idyn(LieGroup::Wrench const & F, double * const torque) const
	{ 
		torque[0] = F[5];
	}

	inline virtual void project(Matrix6d const & A, double * const EA, double * const lambda) const
	{
		project(A, EA);

		lambda[0] = A(5, 5);
	}

	inline virtual void project(Matrix6d const & A, double * const EA) const 
	{		
		Eigen::Map<JointMatTr> ea(EA);
		ea = A.row(5);
	}
};

//  ---------------------- Doxygen info ----------------------
//! \class PrismaticJoint
//!
//! \brief
//! This implements a one dof prismatic joint class.
//!
//! \sa JointBase
//  ----------------------------------------------------------
class PrismaticJoint : public JointBase<PrismaticJoint>
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a prismatic joint having the initial joint angle
	//! 
	//! \param q0 Initial joint angle (in radian)
	//  ----------------------------------------------------------
	inline PrismaticJoint(double q0 = 0.0) : JointBase<PrismaticJoint>()
	{
		update(&q0);

		_E.col(0) = Vector6d::Unit(2);
		_E.col(1) = Vector6d::Unit(0);
		_E.col(2) = Vector6d::Unit(1);
	}

	inline virtual void update(double const * const q)
	{
		//_HT.r().z() = q[0];

		_Tadj.R() = _Toffset.R();
		_Tadj.r() = _Toffset.r() + q[0]*_Toffset.R().col(2);
	}

	inline virtual void update(double const * const q, double const * const qdot) 
	{ 
		update(q);
		//_Vadj(2) = qdot[0];
		updateVelocity(q, qdot);
	}

	inline virtual void updateVelocity(double const * const q, double const * const qdot) 
	{ 
		_Vadj(2) = qdot[0];
	}

	inline virtual LieGroup::Twist adjTwist(double const * const qdot) const
	{
		return LieGroup::Twist(0, 0, qdot[0], 0, 0, 0);
	}

	inline virtual LieGroup::Twist adjAcc(double const * const qdot, double const * const qddot) const
	{
		return adjAcc(qddot);
	}

	inline virtual LieGroup::Twist adjAcc(double const * const qddot) const
	{
		return LieGroup::Twist(0, 0, qddot ? qddot[0] : 0, 0, 0, 0);
	}

	inline virtual LieGroup::Twist biasAcc(LieGroup::Twist const & V) const
	{
		double qdot = _Vadj(2); 
		return LieGroup::Twist(V(4)*qdot, -V(3)*qdot, 0, 0, 0, 0); 
	}

	inline virtual void idyn(LieGroup::Wrench const & F, double * const torque) const
	{ 
		torque[0] = F[2];
	}

	inline virtual void project(Matrix6d const & A, double * const EA, double * const lambda) const
	{
		project(A, EA);

		lambda[0] = A(2, 2);
	}

	inline virtual void project(Matrix6d const & A, double * const EA) const 
	{		
		Eigen::Map<JointMatTr> ea(EA);
		ea = A.row(2);
	}
};

//  ---------------------- Doxygen info ----------------------
//! \class MDHRevoluteJoint
//!
//! \brief
//! This implements a modified DH revolute joint class.
//!
//! \details 
//! Modified DH parameters consists of four: \f$ a \f$, \f$ alpha \f$, \f$ d_0 \f$, and \f$ \theta_0 \f$.
//! The last two parameters \f$ d_0 \f$, and \f$ \theta_0 \f$ determines the zero configuration. 
//!
//! \sa JointBase
//  ----------------------------------------------------------
class MDHRevoluteJoint : public JointBase<MDHRevoluteJoint>
{
public:
// 	typedef JointBase<MDHRevoluteJoint> Base;
// 	
// 	Base& base() { return *static_cast<Base*>(this); }
//	const Base& base() const { return *static_cast<const Base*>(this); }

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a modified DH revolute joint with parameters
	//! 
	//! \param a Link length 
	//! \param \alpha Link twist (in radian)
	//! \param d0 Joint offset
	//! \param theta Joint angle (in radian)
	//  ----------------------------------------------------------
	inline MDHRevoluteJoint(double a = 0, double alpha = 0, double d0 = 0.0, double theta0 = 0.0) 
		: JointBase<MDHRevoluteJoint>(), _cal(cos(alpha)), _sal(sin(alpha)), _theta0(theta0)
	{
		double sth = sin(theta0);
		double cth = cos(theta0);

		_Tadj.r() << a, -d0*_sal, d0*_cal;
		_Tadj.R() << cth, -sth, 0, sth*_cal, cth*_cal, -_sal, sth*_sal, cth*_sal, _cal;

		if (hasOffset())
			_Tadj = jointFrame().cascade(_Tadj);

		Eigen::Matrix<int, 6, 1> indices;
		indices << 5, 0, 1, 2, 3, 4;
		_E = _E * indices.asPermutation();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets a modified DH parameters
	//! 
	//! \param a Link length 
	//! \param \alpha Link twist (in radian)
	//! \param d0 Joint offset
	//! \param theta Joint angle (in radian)
	//  ----------------------------------------------------------
	inline void set(double a, double alpha, double d0, double theta0 = 0.0)
	{
		_cal = cos(alpha);
		_sal = sin(alpha);
		_theta0 = theta0;

		double sth = sin(theta0);
		double cth = cos(theta0);

		_Tadj.r() << a, -d0*_sal, d0*_cal;
		_Tadj.R() << cth, -sth, 0, sth*_cal, cth*_cal, -_sal, sth*_sal, cth*_sal, _cal;

		if (hasOffset())
			_Tadj = jointFrame().cascade(_Tadj);
	}

	inline virtual void update(double const * const q)
	{
		double th = q[0] + _theta0;
		double sth = sin(th);
		double cth = cos(th);

		_Tadj.R().topLeftCorner<3,2>() << cth, -sth, sth*_cal, cth*_cal, sth*_sal, cth*_sal;

		if (hasOffset())
		{
			//_Tadj = jointFrame().cascade(_Tadj);
			_Tadj.R().topLeftCorner<3,2>() = jointFrame().R()*_Tadj.R().topLeftCorner<3,2>();
		}
	}

	inline virtual void update(double const * const q, double const * const qdot) 
	{ 
		update(q);
		//_Vadj(5) = qdot[0];
		updateVelocity(q, qdot);
	}

	inline virtual void updateVelocity(double const * const q, double const * const qdot) 
	{ 
		_Vadj(5) = qdot[0];
	}

	inline virtual LieGroup::Twist adjTwist(double const * const qdot) const
	{
		return LieGroup::Twist(0, 0, 0, 0, 0, qdot[0]);
	}

	inline virtual LieGroup::Twist adjAcc(double const * const qdot, double const * const qddot) const
	{
		return adjAcc(qddot);
	}

	inline virtual LieGroup::Twist adjAcc(double const * const qddot) const
	{
		return LieGroup::Twist(0, 0, 0, 0, 0, qddot ? qddot[0] : 0);
	}

	inline virtual LieGroup::Twist biasAcc(LieGroup::Twist const & V) const
	{
		double qdot = _Vadj(5); 
		return LieGroup::Twist(V(1)*qdot, -V(0)*qdot, 0, V(4)*qdot, -V(3)*qdot, 0); 
	}

	inline virtual void idyn(LieGroup::Wrench const & F, double * const torque) const
	{ 
		torque[0] = F[5];
	}

	inline virtual void project(Matrix6d const & A, double * const EA, double * const lambda) const
	{
		project(A, EA);

		lambda[0] = A(5, 5);
	}

	inline virtual void project(Matrix6d const & A, double * const EA) const 
	{		
		Eigen::Map<JointMatTr> ea(EA);
		ea = A.row(5);
	}

	inline double theta0() const { return _theta0; }

private:
	double _cal;
	double _sal;
	double _theta0;
};

//  ---------------------- Doxygen info ----------------------
//! \class MDHPrismaticJoint
//!
//! \brief
//! This implements a modified DH prismatic joint class.
//!
//! \details 
//! Modified DH parameters consists of four: \f$ a \f$, \f$ alpha \f$, \f$ d_0 \f$, and \f$ \theta_0 \f$.
//! The last two parameters \f$ d_0 \f$, and \f$ \theta_0 \f$ determines the zero configuration. 
//!
//! \sa JointBase
//  ----------------------------------------------------------
class MDHPrismaticJoint : public JointBase<MDHPrismaticJoint>
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a modified DH prismatic joint with parameters
	//! 
	//! \param a Link length 
	//! \param \alpha Link twist (in radian)
	//! \param d0 Joint offset
	//! \param theta Joint angle (in radian)
	//  ----------------------------------------------------------
	inline MDHPrismaticJoint(double a = 0, double alpha = 0, double d0 = 0.0, double theta0 = 0.0) 
		: JointBase<MDHPrismaticJoint>(), _cal(cos(alpha)), _sal(sin(alpha)), _d0(d0)
	{
		//update(&q0);
		double sth = sin(theta0);
		double cth = cos(theta0);

		_Tadj.r() << a, -d0*_sal, d0*_cal;
		_Tadj.R() << cth, -sth, 0, sth*_cal, cth*_cal, -_sal, sth*_sal, cth*_sal, _cal;

		if (hasOffset())
			_Tadj = jointFrame().cascade(_Tadj);

		_E.col(0) = Vector6d::Unit(2);
		_E.col(1) = Vector6d::Unit(0);
		_E.col(2) = Vector6d::Unit(1);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets a modified DH parameters
	//! 
	//! \param a Link length 
	//! \param \alpha Link twist (in radian)
	//! \param d0 Joint offset
	//! \param theta Joint angle (in radian)
	//  ----------------------------------------------------------
	inline void set(double a, double alpha, double d0, double theta0)
	{
		_cal = cos(alpha);
		_sal = sin(alpha);
		_d0 = d0;

		double sth = sin(theta0);
		double cth = cos(theta0);

		_Tadj.r() << a, -d0*_sal, d0*_cal;
		_Tadj.R() << cth, -sth, 0, sth*_cal, cth*_cal, -_sal, sth*_sal, cth*_sal, _cal;

		if (hasOffset())
			_Tadj = jointFrame().cascade(_Tadj);
	}

	inline virtual void update(double const * const q)
	{
		double d = q[0] + _d0;
		_Tadj.r().tail<2>() << -d*_sal, d*_cal;

		if (hasOffset())
		{
			//_Tadj = jointFrame().cascade(_Tadj);
			_Tadj.r() = jointFrame().r() + jointFrame().R()*_Tadj.r();
		}
	}

	inline virtual void update(double const * const q, double const * const qdot) 
	{ 
		update(q);
		//_Vadj(2) = qdot[0];
		updateVelocity(q, qdot);
	}
	
	inline virtual void updateVelocity(double const * const q, double const * const qdot) 
	{ 
		_Vadj(2) = qdot[0];
	}

	inline virtual LieGroup::Twist adjTwist(double const * const qdot) const
	{
		return LieGroup::Twist(0, 0, qdot[0], 0, 0, 0);
	}

	inline virtual LieGroup::Twist adjAcc(double const * const qdot, double const * const qddot) const
	{
		return adjAcc(qddot);
	}	

	inline virtual LieGroup::Twist adjAcc(double const * const qddot) const
	{
		return LieGroup::Twist(0, 0, qddot ? qddot[0] : 0, 0, 0, 0);
	}

	inline virtual LieGroup::Twist biasAcc(LieGroup::Twist const & V) const
	{
		double qdot = _Vadj(2); 
		return LieGroup::Twist(V(4)*qdot, -V(3)*qdot, 0, 0, 0, 0); 
	}

	inline virtual void idyn(LieGroup::Wrench const & F, double * const torque) const
	{ 
		torque[0] = F[2];
	}

	inline virtual void project(Matrix6d const & A, double * const EA, double * const lambda) const
	{
		project(A, EA);

		lambda[0] = A(2, 2);
	}

	inline virtual void project(Matrix6d const & A, double * const EA) const 
	{		
		Eigen::Map<JointMatTr> ea(EA);
		ea = A.row(2);
	}

	inline double d0() const { return _d0; }

private:
	double _cal;
	double _sal;
	double _d0;
};

//  ---------------------- Doxygen info ----------------------
//! \class CylindricalJoint
//!
//! \brief
//! This implements a two dof cylindrical joint class.
//! Cylindrical joints have two-dimensional joint vectors consisting of 
//! joint distance and joint angles in this order.
//!
//! \sa JointBase
//  ----------------------------------------------------------
class CylindricalJoint : public JointBase<CylindricalJoint>
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a cylindrical joint having the initial joint angles
	//! 
	//! \param d0 Initial joint distance 
	//! \param q0 Initial joint angle (in radian)
	//  ----------------------------------------------------------
	inline CylindricalJoint(double d0 = 0.0, double q0 = 0.0) : JointBase<CylindricalJoint>()
	{
		update(&q0);

		Eigen::Matrix<int, 6, 1> indices;
		indices << 2, 5, 0, 1, 3, 4;
		_E = _E * indices.asPermutation();
	}

	inline virtual void update(double const * const q)
	{		
		//_HT.r().z() = q[0];
		_Tadj.r() = _Toffset.r() + q[0]*_Toffset.R().col(2);

		double c = cos(q[1]);
		double s = sin(q[1]);
		//_Tadj.R().topLeftCorner<2,2>() << c, -s, s, c;
		//_HT.R() << c, -s, 0, s, c, 0, 0, 0, 1;
		_Tadj.R().topLeftCorner<2,2>() = _Toffset.R().topLeftCorner<2,2>()*(Eigen::Matrix2d() << c, -s, s, c).finished();
	}

	inline virtual void update(double const * const q, double const * const qdot) 
	{ 
		update(q);
		updateVelocity(q, qdot);
		//_Vadj(2) = qdot[0];
		//_Vadj(5) = qdot[1];
	}

	inline virtual void updateVelocity(double const * const q, double const * const qdot) 
	{ 
		_Vadj(2) = qdot[0];
		_Vadj(5) = qdot[1];
	}

	inline virtual LieGroup::Twist adjTwist(double const * const qdot) const
	{
		return LieGroup::Twist(0, 0, qdot[0], 0, 0, qdot[1]);
	}

	inline virtual LieGroup::Twist adjAcc(double const * const qdot, double const * const qddot) const
	{
		return adjAcc(qddot);
	}
	
	inline virtual LieGroup::Twist adjAcc(double const * const qddot) const
	{
		if (qddot)
			return LieGroup::Twist(0, 0, qddot[0], 0, 0, qddot[1]);
		else
			return LieGroup::Twist();
	}

	inline virtual LieGroup::Twist biasAcc(LieGroup::Twist const & V) const
	{
		double qdot0 = _Vadj(2); 
		double qdot1 = _Vadj(5);

		return LieGroup::Twist(V(1)*qdot1 + V(4)*qdot0, -V(0)*qdot1 - V(3)*qdot0, 0, 
			V(4)*qdot1, -V(3)*qdot1, 0); 
	}

	inline virtual void idyn(LieGroup::Wrench const & F, double * const torque) const
	{ 
		torque[0] = F[2];
		torque[1] = F[5];
	}

	inline virtual void project(Matrix6d const & A, double * const EA, double * const lambda) const
	{
		project(A, EA);

		Eigen::Map<SqJointMat> Lambda(lambda);
		Lambda << A(2, 2), A(2, 5), A(5, 2), A(5, 5);
	}
	
	inline virtual void project(Matrix6d const & A, double * const EA) const 
	{		
		Eigen::Map<JointMatTr> ea(EA);
		ea << A.row(2), A.row(5);
	}
};

//  ---------------------- Doxygen info ----------------------
//! \class SphericalJoint
//!
//! \brief
//! This implements a three-dof spherical joint class.
//! The joint vector for the spherical joint is the lie algebra vector 
//! of the joint rotation matrix. 
//!
//! \sa JointBase
//  ----------------------------------------------------------
// FIXME JF transform should be taken into account in kinematics update.
class SphericalJoint : public JointBase<SphericalJoint>
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a default spherical joint 
	//  ----------------------------------------------------------
	inline SphericalJoint() : JointBase<SphericalJoint>() { }
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a spherical joint having the initial adjacent transformation
	//! 
	//! \param T0 Initial adjacent transformation
	//  ----------------------------------------------------------
	inline SphericalJoint(LieGroup::HTransform const & T0) : JointBase<SphericalJoint>() { _Tadj = T0; }

	inline virtual void update(double const * const q) 
	{
		LieGroup::Vector3D xi(q[0], q[1], q[2]);

		xi.compute_terms(&_alpha, &_beta, &_theta_sqr);
		_Tadj.R() = _Toffset.R()*xi.exp(_alpha, _beta, _theta_sqr);
		_Tadj.r() = _Toffset.r();

		// The property that the terms computed by -xi are the same as those by xi
		_E.topLeftCorner<3,3>().setZero();
		_E.topRightCorner<3, 3>().setIdentity();
		//LieGroup::Vector3D minus_xi = -xi;
		//_E.bottomLeftCorner<3, 3>() = minus_xi.dexp(alpha, beta, theta_sqr);
		_E.bottomLeftCorner<3, 3>() = xi.dexp(_alpha, _beta, _theta_sqr).transpose();
		_E.bottomRightCorner<3, 3>().setZero();
	}

	inline virtual void update(double const * const q, double const * const qdot) 
	{
		LieGroup::Vector3D xi(q[0], q[1], q[2]);

		xi.compute_terms(&_alpha, &_beta, &_theta_sqr);

		_Tadj.R() = xi.exp(_alpha, _beta, _theta_sqr);
		_Tadj.r() = _Toffset.r();

		// The property that the terms computed by -xi are the same as those by xi
		_E.topLeftCorner<3,3>().setZero();
		_E.topRightCorner<3, 3>().setIdentity();
		//LieGroup::Vector3D minus_xi = -xi;
		//_E.bottomLeftCorner<3, 3>() = minus_xi.dexp(alpha, beta, theta_sqr);
		_E.bottomLeftCorner<3, 3>() = xi.dexp(_alpha, _beta, _theta_sqr).transpose();
		_E.bottomRightCorner<3, 3>().setZero();

		// _Vadj is updated.
		LieGroup::Vector3D xidot(qdot[0], qdot[1], qdot[2]);
		_Vadj.w().noalias() = _E.bottomLeftCorner<3, 3>() * xidot;

		_Edot.topLeftCorner<3,3>().setZero();
		//_Edot.topRightCorner<3, 3>().setZero();
		//_Edot.bottomLeftCorner<3, 3>() = minus_xi.ddt_dexp(alpha, beta, theta_sqr, xi.dot(xidot), -xidot);
		_Edot.bottomLeftCorner<3, 3>() = xi.ddt_dexp(_alpha, _beta, _theta_sqr, xi.dot(xidot), xidot).transpose();
		//_Edot.bottomLeftCorner<3, 3>().setZero();
	}

	inline virtual void updateVelocity(double const * const q, double const * const qdot) 
	{
		LieGroup::Vector3D xi(q[0], q[1], q[2]);

		// _Vadj is updated.
		LieGroup::Vector3D xidot(qdot[0], qdot[1], qdot[2]);
		_Vadj.w().noalias() = _E.bottomLeftCorner<3, 3>() * xidot;

		_Edot.topLeftCorner<3,3>().setZero();
		//_Edot.topRightCorner<3, 3>().setZero();
		//_Edot.bottomLeftCorner<3, 3>() = minus_xi.ddt_dexp(alpha, beta, theta_sqr, xi.dot(xidot), -xidot);
		_Edot.bottomLeftCorner<3, 3>() = xi.ddt_dexp(_alpha, _beta, _theta_sqr, xi.dot(xidot), xidot).transpose();
		//_Edot.bottomLeftCorner<3, 3>().setZero();
	}

	inline virtual LieGroup::Twist adjTwist(double const * const qdot) const
	{
		LieGroup::Twist a;
		Eigen::Map<Eigen::Vector3d> w(a.w().derived().data());

		Eigen::Map<const Eigen::Vector3d> Qdot(qdot);
		w = _E.bottomLeftCorner<3, 3>()*Qdot;

		return a;
	}
	
	inline virtual LieGroup::Twist adjAcc(double const * const qdot, double const * const qddot) const
	{
		LieGroup::Twist a;
		Eigen::Map<Eigen::Vector3d> wdot(a.w().derived().data());

		Eigen::Map<const Eigen::Vector3d> Qddot(qddot);
		wdot = _E.bottomLeftCorner<3, 3>()*Qddot;

		//a.w() = _E.bottomLeftCorner<3, 3>()*LieGroup::Vector3D(qddot[0], qddot[1], qddot[2]);
		
		Eigen::Map<const Eigen::Vector3d> Qdot(qdot);
		wdot += _Edot.bottomLeftCorner<3, 3>()*Qdot;

		//a.w() += _Edot.bottomLeftCorner<3, 3>()*LieGroup::Vector3D(qdot[0], qdot[1], qdot[2]);

		return a;
	}
	
	inline virtual LieGroup::Twist adjAcc(double const * const qddot) const
	{
		LieGroup::Twist a;
		Eigen::Map<Eigen::Vector3d> wdot(a.w().derived().data());

		Eigen::Map<const Eigen::Vector3d> Qddot(qddot);
		wdot = _E.bottomLeftCorner<3, 3>()*Qddot;

		//a.w() = _E.bottomLeftCorner<3, 3>()*LieGroup::Vector3D(qddot[0], qddot[1], qddot[2]);
		
		return a;
	}

	inline virtual LieGroup::Twist biasAcc(LieGroup::Twist const & V) const
	{
		return LieGroup::Twist(V.v().cross(_Vadj.w()), V.w().cross(_Vadj.w()));
	}

	inline virtual void idyn(LieGroup::Wrench const & F, double * const torque) const
	{ 
		Eigen::Map<Eigen::Vector3d> tau(torque);
		tau = _E.bottomLeftCorner<3, 3>().transpose()*F.n();
	}

	inline virtual void project(Matrix6d const & A, double * const EA, double * const lambda) const
	{
		Eigen::Map<JointMatTr> ea(EA);
		ea.noalias() = _E.bottomLeftCorner<3, 3>().transpose()*A.bottomRows<3>();

		Eigen::Map<SqJointMat> Lambda(lambda);
		Lambda.noalias() = ea.rightCols<3>()*_E.bottomLeftCorner<3, 3>();
	}
	
	inline virtual void project(Matrix6d const & A, double * const EA) const 
	{		
		Eigen::Map<JointMatTr> ea(EA);
		ea.noalias() = _E.bottomLeftCorner<3, 3>().transpose()*A.bottomRows<3>();
	}

private:
	double _alpha;
	double _beta;
	double _theta_sqr;
};

//  ---------------------- Doxygen info ----------------------
//! \class FreeJoint
//!
//! \brief
//! This implements a six dof free joint class.
//! The joint vector for the free joint is the lie algebra vector 
//! of the joint transformation matrix. 
//!
//! \sa JointBase
//  ----------------------------------------------------------
class FreeJoint : public JointBase<FreeJoint>
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a default free joint 
	//  ----------------------------------------------------------
	inline FreeJoint() : JointBase<FreeJoint>() { }
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a free joint having the initial adjacent transformation
	//! 
	//! \param T0 Initial adjacent transformation
	//  ----------------------------------------------------------
	inline FreeJoint(LieGroup::HTransform const & T0) : JointBase<FreeJoint>() { _Tadj = T0; }

	inline virtual void update(double const * const q) 
	{
		LieGroup::Twist xi(q[0], q[1], q[2], q[3], q[4], q[5]);

		xi.compute_terms(&_alpha, &_beta, &_theta_sqr);
		_Tadj = xi.exp(_alpha, _beta, _theta_sqr);

		// FIXME: minus_xi is not used
		//LieGroup::Twist minus_xi = -xi;
		//_E = minus_xi.dexp(alpha, beta, theta_sqr);
		_E = xi.dexp(_alpha, _beta, _theta_sqr).transpose();
	}

	inline virtual void update(double const * const q, double const * const qdot) 
	{
		LieGroup::Vector3D eta(q[0], q[1], q[2]);
		LieGroup::Vector3D xi(q[3], q[4], q[5]);
		LieGroup::Twist lambda(eta, xi); 

		lambda.compute_terms(&_alpha, &_beta, &_theta_sqr);

		_Tadj = lambda.exp(_alpha, _beta, _theta_sqr);

		LieGroup::Twist minus_lambda = -lambda;
		_E = minus_lambda.dexp(_alpha, _beta, _theta_sqr);

		// _Vadj is updated.
		_Vadj.noalias() = _E * Eigen::Map<const Vector6d>(qdot);

		LieGroup::Vector3D etadot(qdot[0], qdot[1], qdot[2]);
		LieGroup::Vector3D xidot(qdot[3], qdot[4], qdot[5]);
		LieGroup::Twist lambdadot(etadot, xidot);
		_Edot = minus_lambda.ddt_dexp(_alpha, _beta, _theta_sqr, xi.dot(eta), xi.dot(xidot), xi.dot(etadot), xidot.dot(eta), -lambdadot); 
	}

	inline virtual void updateVelocity(double const * const q, double const * const qdot) 
	{
		LieGroup::Vector3D eta(q[0], q[1], q[2]);
		LieGroup::Vector3D xi(q[3], q[4], q[5]);
		LieGroup::Twist lambda(eta, xi); 
		
		LieGroup::Twist minus_lambda = -lambda;

		// _Vadj is updated.
		_Vadj.noalias() = _E * Eigen::Map<const Vector6d>(qdot);

		LieGroup::Vector3D etadot(qdot[0], qdot[1], qdot[2]);
		LieGroup::Vector3D xidot(qdot[3], qdot[4], qdot[5]);
		LieGroup::Twist lambdadot(etadot, xidot);
		_Edot = minus_lambda.ddt_dexp(_alpha, _beta, _theta_sqr, xi.dot(eta), xi.dot(xidot), xi.dot(etadot), xidot.dot(eta), -lambdadot); 
	}

	inline virtual LieGroup::Twist adjTwist(double const * const qdot) const
	{
		LieGroup::Twist V;
		Eigen::Map<Vector6d> v(V.data());

		Eigen::Map<const Vector6d> Qdot(qdot);
		v = _E*Qdot;

		return V;
	}

	inline virtual LieGroup::Twist adjAcc(double const * const qdot, double const * const qddot) const
	{
		LieGroup::Twist Vdot;
		Eigen::Map<Vector6d> vdot(Vdot.data());

		Eigen::Map<const Vector6d> Qddot(qddot);
		vdot = _E*Qddot;

		//Vdot = _E*LieGroup::Twist(qddot[0], qddot[1], qddot[2], qddot[3], qddot[4], qddot[5]);
		Eigen::Map<const Vector6d> Qdot(qdot);
		vdot += _Edot*Qdot;

		//Vdot += _Edot*LieGroup::Twist(qdot[0], qdot[1], qdot[2], qdot[3], qdot[4], qdot[5]));
		
		return Vdot;
	}

	inline virtual LieGroup::Twist adjAcc(double const * const qddot) const
	{
		LieGroup::Twist Vdot;
		Eigen::Map<Vector6d> vdot(Vdot.data());

		Eigen::Map<const Vector6d> Qddot(qddot);
		vdot = _E*Qddot;

		//Vdot = _E*LieGroup::Twist(qddot[0], qddot[1], qddot[2], qddot[3], qddot[4], qddot[5]);		
		
		return Vdot;
	}

	inline virtual LieGroup::Twist biasAcc(LieGroup::Twist const & V) const
	{
		return V.adjoint(_Vadj);
	}

	inline virtual void idyn(LieGroup::Wrench const & F, double * const torque) const
	{ 
		Eigen::Map<Vector6d> tau(torque);
		tau = _E.transpose()*F;
	}

	virtual void project(Matrix6d const & A, double * const EA, double * const lambda) const
	{
		Eigen::Map<JointMatTr> ea(EA);
		ea.noalias() = _E.transpose()*A;

		Eigen::Map<SqJointMat> Lambda(lambda);
		Lambda.noalias() = ea*_E;
	}

	inline virtual void project(Matrix6d const & A, double * const EA) const 
	{		
		Eigen::Map<JointMatTr> ea(EA);
		ea.noalias() = _E.transpose()*A;
	}

private:
	double _alpha;
	double _beta;
	double _theta_sqr;
};

} // namespace NRMKFoundation
