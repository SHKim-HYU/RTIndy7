//! \file Indy.h
//! \brief Header file for the class Indy (API of IndySDK)
//!
//! \details
//! This file implements an articulated multibody system class
//! to be used for the interface of NRMKFoundation library
//! \n
//! \copydetails Neuromeka Foundation Library
//! \n
//! Neuromeka \n
//! South Korea\n
//! \n
//! http://www.neuromeka.com\n
//!
//! \date January 2017
//! 
//! \version 2.0.0
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!
//! \note Copyright (C) 2013--2017 Neuromeka
//  ----------------------------------------------------------

#pragma once

#include "IndyBase.h"


#define INDY_NUM_BODIES	7
#define INDY_NUM_JOINTS	6
#define INDY_JOINT_DOF	6


namespace NRMKFoundation
{
namespace IndySDK
{
	class IndyBase6D : public SubsysBase<INDY_NUM_BODIES, INDY_NUM_JOINTS, INDY_JOINT_DOF>
	{
	public:
		//! \brief Provides compile-time constants of the subsys
		enum
		{
			NUM_BODIES = IndyBase6D::NUM_BODIES, //!< Number of bodies
			NUM_JOINTS = IndyBase6D::NUM_JOINTS, //!< Number of joints except the earthing joint
			NUM_TOTAL_JOINTS = IndyBase6D::NUM_TOTAL_JOINTS, //!< Number of total joints including the earthing joint
			JOINT_DOF = IndyBase6D::JOINT_DOF, //!< Total degrees-of-freedom of the system including the earthing joint
			SYSTEM_DOF = IndyBase6D::SYSTEM_DOF, //!< System degrees-of-freedom including the earthing joint dof
		};

		//! \brief Provides Typedefs 
		typedef SubsysBase<INDY_NUM_BODIES, INDY_NUM_JOINTS, INDY_JOINT_DOF> SUBSYSBASE;
		typedef IndyBase6D::JointVec JointVec; //!< Typedef of the vector having the dimension of the total joint dof
		typedef IndyBase6D::JointMat JointMat; //!< Typedef of the square matrix having the dimension of the total joint dof
		typedef IndyBase6D::JointJac JointJac; //!< Typedef of the jacobian matrix having the dimension of six-by-system dof

		typedef IndyBase6D::JointIndex JointIndex;	//!< Joint index vector
		typedef IndyBase6D::VarJointVec VarJointVec; //!< Typedef of the vector having the dimension of the total joint dof
	
	public:
		//! \brief constructs the subsys
		IndyBase6D();
	
		// kinematic update of every bodies in chain

		//! \brief updates the kinematic states of the subsys. 
		//! \param pos_only set to true if only position-relevant update is necessary
		//!
		//! \details
		//! It updates the transformation and the twist of every body owned by the system.
		//! Before calling this function the system states consisting of joint position and velocity vector 
		//! and base transform and twist (for floating system only) should have been updated
		//! using q(), qdot() and T() and Tdot().
		//! 
		//! \note
		//! It should be called after q(), qdot() and T() and Tdot() has been called.
		void update(bool pos_only = false); 
		void updateVelocity();

		//! \brief updates the jacobian of the system
		//! \param pos_only set to true if only position-relevant update is necessary
		//!
		//! \details
		//! It updates the jacobian of every body owned by the system.
		//! Before calling this function the system kinematics should have been updated
		//! using update().
		//! 
		//! \note
		//! It should be called after update() has been called.
		void updateJacobian(bool pos_only = false);

		//! \brief returns the body acceleration
		//! \return Body acceleration 
		LieGroup::Twist Vdot(int k) const;

		/*
		//! \brief helper function to get v = J*qdot()
		template <typename Derived, typename OtherDerived>
		void kinematics(Eigen::MatrixBase<Derived> const & J, Eigen::MatrixBase<OtherDerived> const & v)
		{
			//return internal::subsys_algorithm<internal::IsFloatingJoint<EarthJoint>::value>::kinematics(*this, J, v);
			Eigen::MatrixBase<OtherDerived> & v_ = const_cast<Eigen::MatrixBase<OtherDerived> &>(v);
			//v_.derived().resize(J.rows(), 1);

			v_.noalias() = J*subsys.qdot();
		}

		template <typename Derived1, typename Derived2, typename OtherDerived>
		void kinematics(Eigen::MatrixBase<Derived1> const & J, Eigen::MatrixBase<Derived2> const & Jdot, 
			Eigen::MatrixBase<OtherDerived> const & vdot) const
		{
			//return internal::subsys_algorithm<internal::IsFloatingJoint<EarthJoint>::value>::kinematics(*this, J, Jdot, vdot);
			Eigen::MatrixBase<OtherDerived> & vdot_ = const_cast<Eigen::MatrixBase<OtherDerived> &>(vdot);
			//vdot_.derived().resize(J.rows(), 1);

			vdot_.noalias() = J*subsys.qddot() + Jdot*subsys.qdot();
		}
		*/

		//! \brief
		//! implements the recursive Newton-Euler method 
		//! for a grounded system for a given joint velocity and acceleration vector
		//! \param qdot_ Array of the joint velocity values 
		//! \param qddot_ Array of the joint acceleration values 
		//! \param gacc Gravitation acceleration vector
		//!
		//! \note
		//! It should be called after update() has been called. 
		//! updateJacobian() is not necessary. When the caculation is completed, 
		//! the inverse dynamic torque is retrieved by tau(). 
		//! The gravitational acceleration vector (of type Vector3D) should be passed 
		//! as the last argument. It is represented with respect to the global reference frame.
		void idyn(JointVec const & qdot_, JointVec const & qddot_, LieGroup::Vector3D const & gacc);

		//! \brief
		//! This implements the recursive Newton-Euler method 
		//! for a floating system for a given joint velocity and acceleration array as well as body twist and acceleration 
		//!
		//! \note
		//! It should be called after update() has been called. 
		//! updateJacobian() is not necessary. When the caculation is completed, 
		//! the inverse dynamic torque is retrieved by tau() and the base wrench by F().
		//! The gravitational acceleration vector (of type Vector3D) should be passed 
		//! as the last argument. It is represented with respect to the global reference frame.
		//!
		//! \param V Base twist
		//! \param Vdot Base acceleration
		//! \param qdot_ Array of the joint velocity values
		//! \param qddot_ Array of the joint acceleration values
		//! \param gacc Gravitation acceleration vector
		void idyn(LieGroup::Twist const & V, LieGroup::Twist const & Vdot, JointVec const & qdot_, JointVec const & qddot_, LieGroup::Vector3D const & gacc);

		//! \brief
		//! implements the recursive passivity-based Newton-Euler method 
		//! for a grounded system for a given joint velocity and acceleration vector
		//! \param qdot_ Array of the joint reference velocity values 
		//! \param qddot_ Array of the joint acceleration values 
		//! \param gacc Gravitation acceleration vector
		//!
		//! \note
		//! It should be called after update() has been called. 
		//! updateJacobian() is not necessary. When the caculation is completed, 
		//! the inverse dynamic torque is retrieved by tau(). 
		//! The gravitational acceleration vector (of type Vector3D) should be passed 
		//! as the last argument. It is represented with respect to the global reference frame.
		void idyn_passive(JointVec const & qdot_, JointVec const & qddot_, LieGroup::Vector3D const & gacc);

		//! \brief
		//! This implements the recursive Newton-Euler method 
		//! for a floating system for a given joint velocity and acceleration array as well as body twist and acceleration 
		//! \param V Base reference twist
		//! \param Vdot Base acceleration
		//! \param qdot_ Array of the joint reference velocity values
		//! \param qddot_ Array of the joint acceleration values
		//! \param gacc Gravitation acceleration vector
		//!
		//! \note
		//! It should be called after update() has been called. 
		//! updateJacobian() is not necessary. When the caculation is completed, 
		//! the inverse dynamic torque is retrieved by tau() and the base wrench by F().
		//! The gravitational acceleration vector (of type Vector3D) should be passed 
		//! as the last argument. It is represented with respect to the global reference frame.
		void idyn_passive(LieGroup::Twist const & V, LieGroup::Twist const & Vdot,  JointVec const & qdot_, JointVec const & qddot_, LieGroup::Vector3D const & gacc);

		//! \brief
		//! implements the recursive Newton-Euler method for gravitation only
		//! for a grounded and floating system for a given joint position
		//! \param gacc Gravitation acceleration vector
		//!
		//! \note
		//! It should be called after update() has been called. 
		//! updateJacobian() is not necessary. When the caculation is completed, 
		//! the inverse dynamic torque is retrieved by tau(). 
		//! The gravitational acceleration vector (of type Vector3D) should be passed 
		//! as the last argument. It is represented with respect to the global reference frame.
		void idyn_gravity(LieGroup::Vector3D const & gacc);

		/// ADDED @20160413
	
		//! \brief
		//! implements the recursive Newton-Euler method without gravity
		//! for a grounded system for a given joint velocity and acceleration vector
		//! \param qdot_ Array of the joint velocity values 
		//! \param qddot_ Array of the joint acceleration values 
		//!
		//! \note
		//! It should be called after update() has been called. 
		//! updateJacobian() is not necessary. When the caculation is completed, 
		//! the inverse dynamic torque is retrieved by tau(). 
		void idyn(JointVec const & qdot_, JointVec const & qddot_);

		//! \brief
		//! This implements the recursive Newton-Euler method without gravity
		//! for a floating system for a given joint velocity and acceleration array as well as body twist and acceleration 
		//! \param V Base twist
		//! \param Vdot Base acceleration
		//! \param qdot_ Array of the joint velocity values
		//! \param qddot_ Array of the joint acceleration values
		//!
		//! \note
		//! It should be called after update() has been called. 
		//! updateJacobian() is not necessary. When the caculation is completed, 
		//! the inverse dynamic torque is retrieved by tau() and the base wrench by F().
		void idyn(LieGroup::Twist const & V, LieGroup::Twist const & Vdot, JointVec const & qdot_, JointVec const & qddot_);

		//! \brief
		//! implements the recursive passivity-based Newton-Euler method without gravity
		//! for a grounded system for a given joint velocity and acceleration vector
		//! \param qdot_ Array of the joint reference velocity values 
		//! \param qddot_ Array of the joint acceleration values 
		//!
		//! \note
		//! It should be called after update() has been called. 
		//! updateJacobian() is not necessary. When the caculation is completed, 
		//! the inverse dynamic torque is retrieved by tau(). 
		void idyn_passive(JointVec const & qdot_, JointVec const & qddot_);

		//! \brief
		//! This implements the recursive Newton-Euler method without gravity
		//! for a floating system for a given joint velocity and acceleration array as well as body twist and acceleration 
		//! \param V Base reference twist
		//! \param Vdot Base acceleration
		//! \param qdot_ Array of the joint reference velocity values
		//! \param qddot_ Array of the joint acceleration values
		//!
		//! \note
		//! It should be called after update() has been called. 
		//! updateJacobian() is not necessary. When the caculation is completed, 
		//! the inverse dynamic torque is retrieved by tau() and the base wrench by F().
		void idyn_passive(LieGroup::Twist const & V, LieGroup::Twist const & Vdot, JointVec const & qdot_, JointVec const & qddot_);
	
		//! \brief
		//! implements the recursive Newton-Euler method for momentum computation
		//! for a grounded system for a given joint acceleration vector
		//! (assuming zero joint velocity)
		//! \param qddot_ Array of the joint acceleration values 
		//!
		//! \note
		//! It should be called after update() has been called. 
		//! updateJacobian() is not necessary. When the caculation is completed, 
		//! the inverse dynamic torque is retrieved by tau(). 
		void idyn_momentum(JointVec const & qddot_);

		//! \brief
		//! This implements the recursive Newton-Euler method for momentum computation
		//! for a floating system for a given joint acceleration array and body acceleration 
		//! (assuming zero joint velocity and body twist)
		//! \param Vdot Base acceleration
		//! \param qddot_ Array of the joint acceleration values
		//!
		//! \note
		//! It should be called after update() has been called. 
		//! updateJacobian() is not necessary. When the caculation is completed, 
		//! the inverse dynamic torque is retrieved by tau() and the base wrench by F().
		void idyn_momentum(LieGroup::Twist const & Vdot, JointVec const & qddot_);



	private:
		void _updateFromBase(double const * const q, double const * const qdot); 
		void _updateFromBase(double const * const q);
		void _updateVelocityFromBase(double const * const q, double const * const qdot);
		void _updateJacobianFromBase(bool pos_only = false);
		void _updateJacobianFromBaseByVelocityReset(bool pos_only = false);

		void _propagateGravAccFromBase(LieGroup::Twist const & Vdot0, LieGroup::Wrench * const F) const;
		void _propagateAccFromBase(LieGroup::Twist const & Vdot0, double const * const qdot, double const * const qddot, LieGroup::Wrench * const F) const;
		void _propagateAccFromBase_stationary(LieGroup::Twist const & Vdot0, double const * const qddot, LieGroup::Wrench * const F) const;
		void _propagateAccFromBase_passive(LieGroup::Twist const & V0, LieGroup::Twist const & Vdot0, double const * const qdot, double const * const qddot, LieGroup::Wrench * const F) const;	
		void _propagateWrenchToBase(LieGroup::Wrench * const F, double * const torque) const;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		using SubsysBase<INDY_NUM_BODIES, INDY_NUM_JOINTS, INDY_JOINT_DOF>::Vdot; // To recover shadowed API in SubsysBase
	};

} // namespace IndySDK

} // namespace NRMKFoundation
