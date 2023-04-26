//  ---------------------- Doxygen info ----------------------
//! \file CompositeHTransformTask.h
//!
//! \brief
//! Header file for the class CompositeHTransformTask (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a composite transform task for six-dimensional spatial task 
//! to be used for the interface of NRMKFoundation library. 
//! It is different from HTransform task in that its task variables are represented by 
//! the composite transform task position, velocity and force, i.e. \f$ \{ R, r \} \f$, 
//! \f$ \{ \omega, \dot{r} \} \f$, \f$ \{ n, f \} \f$, instead of homogeneous transform, 
//! twist and wrench, \f$ T \f$,  \f$ V \f$, and \f$ F \f$.
//! The advantage using this kinematics is that the rotation and translation are decoupled, whereas 
//! they are coupled in HTranform task. This leads to clear understanding. 
//! 
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
//! \date March 2014
//! 
//! \version 1.7
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013-2014 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

// This file is part of NRMKFoundation, a lightweight C++ template library
// for robot motion control.
//
// Copyright (C) 2013-2014 Neuromeka <coolcat@neuromeka.com>

#pragma once 

#include "LieGroup/LieGroup.h"

#include "../Kinematics/Kinematics.h"
#include "../Kinematics/RotationKinematics.h"
#include "../Kinematics/DisplacementKinematics.h"

#include "../Controller/RotationController.h"
#include "../Controller/PositionController.h"

#include "../Interpolator/Interpolator.h"
#include "../Interpolator/RotationInterpolator.h"

namespace NRMKFoundation
{
//  ---------------------- Doxygen info ----------------------
//! \class CompositeHTransformTaskPosition
//!
//! \brief
//! This defines the position variable for composite transform task kinematics. 
//! 
//! \details 
//!	 Composite transform task positions are defined by the pair of rotation matrix and displacement vector. 
//  ----------------------------------------------------------
struct CompositeHTransformTaskPosition
{
	//! \brief constructs a zero task position 
	CompositeHTransformTaskPosition() : rot(), disp() 
	{
	}
	
	//! \brief constructs a task position from homogeneous transformation
	//! \sa LieGroup::HTransform
	CompositeHTransformTaskPosition(LieGroup::HTransform const & T) 
		: rot(T.R()), disp(T.r()) 
	{
	}

	//! \brief returns the identity element
	inline static  	CompositeHTransformTaskPosition const Zero() 
	{
		return CompositeHTransformTaskPosition();
	}

	// initialization is not necessary due to default construct
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//! \brief rotation component
	LieGroup::Rotation rot;
	//! \brief displacement component
	LieGroup::Displacement disp;

 	// for unified member access (same as class HTransform)
	
	//! \brief
	//! returns the rotation matrix 
	LieGroup::Rotation & R() { return rot; }
	//! \brief
	//! returns the displacement vector 
	LieGroup::Displacement & r() { return disp; }

	//! \brief
	//! returns the rotation matrix 
	LieGroup::Rotation const & R() const { return rot; }
	//! \brief
	//! returns the displacement vector 
	LieGroup::Displacement const & r() const { return disp; }

	//! \brief returns the vector form
	inline Vector6d asVector() const
	{
		Vector6d res;
		LieGroup::Vector3D rotAsVector;

		res.tail<3>() = disp;

		rotAsVector = rot.eulerAngles(2, 1, 0);

		res[0] = rotAsVector[2];
		res[1] = rotAsVector[1];
		res[2] = rotAsVector[0];

		return res;
	}
};

//  ---------------------- Doxygen info ----------------------
//! \class CompositeHTransformTaskVelocity
//!
//! \brief
//! This defines the velocity variable for composite transform task kinematics. 
//! 
//! \details 
//!	 Composite transform task velocity are defined by the pair of rotational velocity and translational velocity. 
//  ----------------------------------------------------------
struct CompositeHTransformTaskVelocity
{
	//! \brief constructs a zero task velocity 
	CompositeHTransformTaskVelocity() : rot(), disp() 
	{
	}

	//! \brief constructs a task velocity from twist
	//! \sa LieGroup::Twist
	CompositeHTransformTaskVelocity(LieGroup::Twist const & V) 
		: rot(V.w()), disp(V.v()) 
	{
	}

	//! \brief returns the identity element
	inline static  	CompositeHTransformTaskVelocity const Zero() 
	{
		return CompositeHTransformTaskVelocity();
	}

	//! \brief returns the vector form
	inline Vector6d asVector() const
	{
		Vector6d res;

		res.head<3>() = rot;
		res.tail<3>() = disp;

		return res;
	}

	// initialization is not necessary due to default construct
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//! \brief rotation component
	LieGroup::Vector3D rot;
	//! \brief displacement component
	LieGroup::Vector3D disp;

	//! \brief
	//! returns the rotational velocity
	LieGroup::Vector3D & w() { return rot; }
	//! \brief
	//! returns the translational velocity
	LieGroup::Vector3D & v() { return disp; }

	//! \brief
	//! returns the rotational velocity
	LieGroup::Vector3D const & w() const { return rot; }
	//! \brief
	//! returns the translational velocity
	LieGroup::Vector3D const & v() const { return disp; }

 	// for unified member access (same as class Twist)
// 	void setZero() { rot.setZero(); disp.setZero(); }
};

//  ---------------------- Doxygen info ----------------------
//! \class CompositeHTransformTaskForce
//!
//! \brief
//! This defines the force variable for composite transform task kinematics. 
//! 
//! \details 
//!	 Composite transform task force are defined by the pair of rotational moment and translational force. 
//  ----------------------------------------------------------
struct CompositeHTransformTaskForce
{
	//! \brief constructs a zero task force 
	CompositeHTransformTaskForce() : rot(), disp() 
	{
	}

	//! \brief constructs a task force from wrench
	//! \sa LieGroup::Wrench
	CompositeHTransformTaskForce(LieGroup::Wrench const & F) 
		: rot(F.n()), disp(F.f()) 
	{
	}

	//! \brief returns the identity element
	inline static  	CompositeHTransformTaskForce const Zero() 
	{
		return CompositeHTransformTaskForce();
	}

	//! \brief returns the vector form
	inline Vector6d asVector() const
	{
		Vector6d res;

		res.head<3>() = rot;
		res.tail<3>() = disp;

		return res;
	}

	// initialization is not necessary due to default construct
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//! \brief rotation component
	LieGroup::Vector3D rot;
	//! \brief dispalcement component
	LieGroup::Vector3D disp;

	//! \brief
	//! returns the rotational moment
	LieGroup::Vector3D & n() { return rot; }
	//! \brief
	//! returns the translational force
	LieGroup::Vector3D & f() { return disp; }

	//! \brief
	//! returns the rotational moment
	LieGroup::Vector3D const & n() const { return rot; }
	//! \brief
	//! returns the translational force
	LieGroup::Vector3D const & f() const { return disp; }
};

//  ---------------------- Doxygen info ----------------------
//! \class CompositeAbsHTransformTaskKinematics
//!
//! \brief
//! This defines the absolute composite transform task kinematics. 
//! 
//! \details 
//!	 Absolute composite transform task kinematics is defined by the absolute rotation matrix and 
//! absolute displacement vector of the target body with respect to the global reference frame. 
//! That is, it is defined by RotationAbsTaskKinematics and DisplacementAbsTaskKinematics.
//!
//! \tparam SubsysType Subsys type
//!
//! \sa RotationAbsTaskKinematics
//! \sa DisplacementAbsTaskKinematics
//! \sa HTransformAbsTaskKinematics
//  ----------------------------------------------------------
template <typename SubsysType>
class CompositeAbsHTransformTaskKinematics : public Kinematics<SubsysType, 6, CompositeAbsHTransformTaskKinematics<SubsysType>, CompositeHTransformTaskPosition, CompositeHTransformTaskVelocity, CompositeHTransformTaskVelocity, CompositeHTransformTaskForce>
{
public:
	typedef typename Kinematics<SubsysType, 6, CompositeAbsHTransformTaskKinematics<SubsysType>, CompositeHTransformTaskPosition, CompositeHTransformTaskVelocity>::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename Kinematics<SubsysType, 6, CompositeAbsHTransformTaskKinematics<SubsysType>, CompositeHTransformTaskPosition, CompositeHTransformTaskVelocity>::TaskJac TaskJac; //!< Typedef of task Jacobian

	typedef typename RotationAbsTaskKinematics<SubsysType>::TaskJac TaskJacRot;
	typedef typename DisplacementAbsTaskKinematics<SubsysType>::TaskJac TaskJacDisp;

public:
	inline CompositeAbsHTransformTaskKinematics(SubsysType const & subsys, 
		int target, LieGroup::HTransform const & Ttarget, LieGroup::HTransform const & Tref)
		: _rot(subsys, target, Ttarget.R(), Tref.R())
		, _disp(subsys, target, Ttarget.r(), Tref)
	{
	}
	
	inline CompositeAbsHTransformTaskKinematics(SubsysType const & subsys)
		: _rot(subsys)
		, _disp(subsys)
	{
	}

	inline void set(int target, LieGroup::HTransform const & Ttarget, LieGroup::HTransform const & Tref)
	{
		_rot.set(target, Ttarget.R(), Tref.R());
		_disp.set(target, Ttarget.r(), Tref);
	}

	inline void kinematics(SubsysType const & subsys, CompositeHTransformTaskPosition & p) const
	{
		_rot.kinematics(subsys, p.rot);
		_disp.kinematics(subsys, p.disp);
	}

	inline void kinematics(SubsysType const & subsys, CompositeHTransformTaskPosition & p, CompositeHTransformTaskVelocity & v) const
	{
		_rot.kinematics(subsys, p.rot, v.rot);
		_disp.kinematics(subsys, p.disp, v.disp);
	}

	inline void jacobian(SubsysType const & subsys, CompositeHTransformTaskPosition & p, CompositeHTransformTaskVelocity & v, TaskJac & J, TaskJac & Jdot) const
	{
		TaskJacRot J_rot;
		TaskJacRot Jdot_rot;
		_rot.jacobian(subsys, p.rot, v.rot, J_rot, Jdot_rot);

		TaskJacDisp J_disp;
		TaskJacDisp Jdot_disp;
		_disp.jacobian(subsys, p.disp, v.disp, J_disp, Jdot_disp);

		J << J_rot, J_disp;
		Jdot << Jdot_rot, Jdot_disp;
	}

	inline void jacobian(SubsysType const & subsys, CompositeHTransformTaskPosition & p, TaskJac & J) const
	{
		TaskJacRot J_rot;
		_rot.jacobian(subsys, p.rot, J_rot);

		TaskJacDisp J_disp;
		_disp.jacobian(subsys, p.disp, J_disp);

		J << J_rot, J_disp;
	}

private:
	RotationAbsTaskKinematics<SubsysType> _rot;
	DisplacementAbsTaskKinematics<SubsysType> _disp;
};

//  ---------------------- Doxygen info ----------------------
//! \class CompositeRelHTransformTaskKinematics
//!
//! \brief
//! This defines the relative composite transform task kinematics. 
//! 
//! \details 
//!	 Relative composite transform task kinematics is defined by the relative rotation matrix and 
//! relative displacement vector of the target body with respect to the reference body. 
//! That is, it is defined by RotationRelTaskKinematics and DisplacementRelTaskKinematics.
//!
//! \tparam SubsysType Subsys type
//!
//! \sa RotationRelTaskKinematics
//! \sa DisplacementRelTaskKinematics
//! \sa HTransformRelTaskKinematics
//  ----------------------------------------------------------
template <typename SubsysType>
class CompositeRelHTransformTaskKinematics : public Kinematics<SubsysType, 6, CompositeRelHTransformTaskKinematics<SubsysType>, CompositeHTransformTaskPosition, CompositeHTransformTaskVelocity>
{
public:
	typedef typename Kinematics<SubsysType, 6, CompositeRelHTransformTaskKinematics<SubsysType>, CompositeHTransformTaskPosition, CompositeHTransformTaskVelocity>::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename Kinematics<SubsysType, 6, CompositeRelHTransformTaskKinematics<SubsysType>, CompositeHTransformTaskPosition, CompositeHTransformTaskVelocity>::TaskJac TaskJac; //!< Typedef of task Jacobian

	typedef typename RotationRelTaskKinematics<SubsysType>::TaskJac TaskJacRot;
	typedef typename DisplacementRelTaskKinematics<SubsysType>::TaskJac TaskJacDisp;

public:
	inline CompositeRelHTransformTaskKinematics(SubsysType const & subsys, 
		int target, LieGroup::HTransform const & Ttarget, int ref, LieGroup::HTransform const & Tref)
		: _rot(subsys, target, Ttarget.R(), ref, Tref.R())
		, _disp(subsys, target, Ttarget.r(), ref, Tref)
	{
	}

	inline CompositeRelHTransformTaskKinematics(SubsysType const & subsys)
		: _rot(subsys)
		, _disp(subsys)
	{
	}

	inline void set(int target, LieGroup::HTransform const & Ttarget, int ref, LieGroup::HTransform const & Tref)
	{
		_rot.set(target, Ttarget.R(), ref, Tref.R());
		_disp.set(target, Ttarget.r(), ref, Tref);
	}

	inline void kinematics(SubsysType const & subsys, CompositeHTransformTaskPosition & p, CompositeHTransformTaskVelocity & v) const
	{
		_rot.kinematics(subsys, p.rot, v.rot);
		_disp.kinematics(subsys, p.disp, v.disp);
	}

	inline void kinematics(SubsysType const & subsys, CompositeHTransformTaskPosition & p) const
	{
		_rot.kinematics(subsys, p.rot);
		_disp.kinematics(subsys, p.disp);
	}

	inline void jacobian(SubsysType const & subsys, CompositeHTransformTaskPosition & p, CompositeHTransformTaskVelocity & v, TaskJac & J, TaskJac & Jdot) const
	{
		TaskJacRot J_rot;
		TaskJacRot Jdot_rot;
		_rot.jacobian(subsys, p.rot, v.rot, J_rot, Jdot_rot);

		TaskJacDisp J_disp;
		TaskJacDisp Jdot_disp;
		_disp.jacobian(subsys, p.disp, v.disp, J_disp, Jdot_disp);

		J << J_rot, J_disp;
		Jdot << Jdot_rot, Jdot_disp;
	}

	inline void jacobian(SubsysType const & subsys, CompositeHTransformTaskPosition & p, TaskJac & J) const
	{
		TaskJacRot J_rot;
		_rot.jacobian(subsys, p.rot, J_rot);

		TaskJacDisp J_disp;
		_disp.jacobian(subsys, p.disp, J_disp);

		J << J_rot, J_disp;
	}

private:
	RotationRelTaskKinematics<SubsysType> _rot;
	DisplacementRelTaskKinematics<SubsysType> _disp;
};

//  ---------------------- Doxygen info ----------------------
//! \class CompositeHTransformTaskController
//!
//! \brief
//! This defines the composite transform task controller. 
//! 
//! \details 
//!	 Composite transform task controller is defined by the rotation controller and 
//! displacement controller corresponding to composite transform task kinematics variables. 
//!
//! \sa RotationController
//! \sa PositionController
//! \sa HTransformController
//  ----------------------------------------------------------
class CompositeHTransformTaskController 
	: public Controller<6, CompositeHTransformTaskController, CompositeHTransformTaskPosition, CompositeHTransformTaskVelocity, CompositeHTransformTaskVelocity, CompositeHTransformTaskForce>
{
public:
	typedef Controller<6, CompositeHTransformTaskController, CompositeHTransformTaskPosition, CompositeHTransformTaskVelocity, CompositeHTransformTaskVelocity, CompositeHTransformTaskForce>::TaskVec TaskVec; //!< Typedef of task vector

	typedef RotationController::TaskVec TaskVecRot;
	typedef PositionController<3>::TaskVec TaskVecDisp;

public:
	// distribute the gains to the sub-task controllers
	inline void setSubGains() 
	{
		_rot.setPeriod(_dT);
		_rot.setPosGain(_kp.head<RotationController::DIM>());
		_rot.setVelGain(_kv.head<RotationController::DIM>());
		_rot.setIntGain(_ki.head<RotationController::DIM>());

		_rot.setForceGain(_kf.head<RotationController::DIM>());
		_rot.setForceIntGain(_kfi.head<RotationController::DIM>());
		_rot.setInvL2sqr(_invL2sqr.head<RotationController::DIM>());

		_rot.setSubGains();

		_disp.setPeriod(_dT);
		_disp.setPosGain(_kp.tail<PositionController<3>::DIM>());
		_disp.setVelGain(_kv.tail<PositionController<3>::DIM>());
		_disp.setIntGain(_ki.tail<PositionController<3>::DIM>());

		_disp.setForceGain(_kf.tail<PositionController<3>::DIM>());
		_disp.setForceIntGain(_kfi.tail<PositionController<3>::DIM>());
		_disp.setInvL2sqr(_invL2sqr.tail<PositionController<3>::DIM>());

		_disp.setSubGains();
	}

	inline void refAcceleration(PosType const & cur_pos, VelType const & cur_vel, 
		PosType const & des_pos, VelType const & des_vel, AccType const & des_acc, 
		TaskVec & ref_vel, TaskVec & ref_acc)
	{
		TaskVec e_pos;
		TaskVec e_vel;

		refAcceleration(cur_pos, cur_vel, des_pos, des_vel, des_acc, ref_vel, ref_acc, e_pos, e_vel);
	}

	inline void refAcceleration(PosType const & cur_pos, VelType const & cur_vel, 
		PosType const & des_pos, VelType const & des_vel, AccType const & des_acc, 
		TaskVec & ref_vel, TaskVec & ref_acc, TaskVec & e_pos, TaskVec & e_vel)
	{
		TaskVecRot ref_vel_rot;
		TaskVecRot ref_acc_rot;
		TaskVecRot e_pos_rot;
		TaskVecRot e_vel_rot;
		_rot.refAcceleration(cur_pos.rot, cur_vel.rot, des_pos.rot, des_vel.rot, des_acc.rot, ref_vel_rot, ref_acc_rot, e_pos_rot, e_vel_rot);
		
		TaskVecDisp ref_vel_disp;
		TaskVecDisp ref_acc_disp;
		TaskVecDisp e_pos_disp;
		TaskVecDisp e_vel_disp;
		_disp.refAcceleration(cur_pos.disp, cur_vel.disp, des_pos.disp, des_vel.disp, des_acc.disp, ref_vel_disp, ref_acc_disp, e_pos_disp, e_vel_disp);

		ref_vel << ref_vel_rot, ref_vel_disp;
		ref_acc << ref_acc_rot, ref_acc_disp;
		e_pos << e_pos_rot, e_pos_disp;
		e_vel << e_vel_rot, e_vel_disp;

		// FIXED @ 20131216
		_ref << _rot.refForce(), _disp.refForce();

		/// FIXME @20151015
		_int_error << _rot.int_error(), _disp.int_error(); 
	}

	inline void refImpedanceAcceleration(PosType const & cur_pos, VelType const & cur_vel, 
		PosType const & des_pos, VelType const & des_vel, AccType const & des_acc, 
		ForceType const & cur_force, ForceType const & des_force,
		TaskVec & ref_vel, TaskVec & ref_acc)
	{
		TaskVec e_pos;
		TaskVec e_vel;
		TaskVec e_force;

		refImpedanceAcceleration(cur_pos, cur_vel, des_pos, des_vel, des_acc, cur_force, des_force, ref_vel, ref_acc, e_pos, e_vel, e_force);
	}

	inline void refImpedanceAcceleration(PosType const & cur_pos, VelType const & cur_vel, 
		PosType const & des_pos, VelType const & des_vel, AccType const & des_acc, 
		ForceType const & cur_force, ForceType const & des_force,
		TaskVec & ref_vel, TaskVec & ref_acc, 
		TaskVec & e_pos, TaskVec & e_vel,
		TaskVec & e_force)
	{
		TaskVecRot ref_vel_rot;
		TaskVecRot ref_acc_rot;
		TaskVecRot e_pos_rot;
		TaskVecRot e_vel_rot;
		TaskVecRot e_force_rot;
		_rot.refImpedanceAcceleration(cur_pos.rot, cur_vel.rot, des_pos.rot, des_vel.rot, des_acc.rot, cur_force.rot, des_force.rot, ref_vel_rot, ref_acc_rot, e_pos_rot, e_vel_rot, e_force_rot);

		TaskVecDisp ref_vel_disp;
		TaskVecDisp ref_acc_disp;
		TaskVecDisp e_pos_disp;
		TaskVecDisp e_vel_disp;
		TaskVecDisp e_force_disp;
		_disp.refImpedanceAcceleration(cur_pos.disp, cur_vel.disp, des_pos.disp, des_vel.disp, des_acc.disp, cur_force.disp, des_force.disp, ref_vel_disp, ref_acc_disp, e_pos_disp, e_vel_disp, e_force_disp);

		ref_vel << ref_vel_rot, ref_vel_disp;
		ref_acc << ref_acc_rot, ref_acc_disp;
		e_pos << e_pos_rot, e_pos_disp;
		e_vel << e_vel_rot, e_vel_disp;
		e_force << e_force_rot, e_force_disp;

		_ref << _rot.refForce(), _disp.refForce();

		/// FIXME @20151015
		_int_error << _rot.int_error(), _disp.int_error(); 
		_int_ef << _rot.int_ef(), _disp.int_ef(); 
	}

	/// FIXME @20151015
	inline void refComplianceForce(PosType const & cur_pos, VelType const & cur_vel, 
		PosType const & des_pos, VelType const & des_vel, AccType const & des_acc, 
		ForceType const & cur_force, ForceType const & des_force,
		ForceType & ref_force)
	{
		TaskVec e_pos;
		TaskVec e_vel;
		TaskVec e_force;

		refComplianceForce(cur_pos, cur_vel, des_pos, des_vel, des_acc, cur_force, des_force, ref_force, e_pos, e_vel, e_force);
	}

	inline void refComplianceForce(PosType const & cur_pos, VelType const & cur_vel, 
		PosType const & des_pos, VelType const & des_vel, AccType const & des_acc, 
		ForceType const & cur_force, ForceType const & des_force,
		ForceType & ref_force, 
		TaskVec & e_pos, TaskVec & e_vel,
		TaskVec & e_force)
	{
		TaskVecRot ref_force_rot;
		TaskVecRot e_pos_rot;
		TaskVecRot e_vel_rot;
		TaskVecRot e_force_rot;
		_rot.refComplianceForce(cur_pos.rot, cur_vel.rot, des_pos.rot, des_vel.rot, des_acc.rot, cur_force.rot, des_force.rot, ref_force_rot, e_pos_rot, e_vel_rot, e_force_rot);

		TaskVecDisp ref_force_disp;
		TaskVecDisp e_pos_disp;
		TaskVecDisp e_vel_disp;
		TaskVecDisp e_force_disp;
		_disp.refComplianceForce(cur_pos.disp, cur_vel.disp, des_pos.disp, des_vel.disp, des_acc.disp, cur_force.disp, des_force.disp, ref_force_disp, e_pos_disp, e_vel_disp, e_force_disp);

		/// FIXME @20151103
		// Only valid for identity mass matrix
		_stiffness.topLeftCorner<3,3>() = _rot.stiffness();
		_stiffness.bottomRightCorner<3,3>() = _disp.stiffness();
		_damping.topLeftCorner<3,3>() = _rot.damping();
		_damping.bottomRightCorner<3,3>() = _disp.damping();
		/////////////////////////////////////////////////////////////////////////////

		_ref << _rot.refForce(), _disp.refForce();

		ref_force.rot = _rot.refForce();
		ref_force.disp = _disp.refForce(); 

		e_pos << e_pos_rot, e_pos_disp;
		e_vel << e_vel_rot, e_vel_disp;
		e_force << e_force_rot, e_force_disp;
		
		/// FIXME @20151015
		_int_error << _rot.int_error(), _disp.int_error(); 
		_int_ef << _rot.int_ef(), _disp.int_ef(); 
	}

	inline void refVelocity(PosType const & cur_pos, 
		PosType const & des_pos, VelType const & des_vel, 
		TaskVec & ref_vel)
	{
		TaskVec e_pos;		

		refVelocity(cur_pos, des_pos, des_vel, ref_vel, e_pos);
	}

	inline void refVelocity(PosType const & cur_pos, 
		PosType const & des_pos, VelType const & des_vel, 
		TaskVec & ref_vel, TaskVec & e_pos)
	{
		TaskVecRot ref_vel_rot;
		TaskVecRot e_pos_rot;
		_rot.refVelocity(cur_pos.rot, des_pos.rot, des_vel.rot, ref_vel_rot, e_pos_rot);

		TaskVecDisp ref_vel_disp;
		TaskVecDisp e_pos_disp;
		_disp.refVelocity(cur_pos.disp, des_pos.disp, des_vel.disp, ref_vel_disp, e_pos_disp);

		ref_vel << ref_vel_rot, ref_vel_disp;
		e_pos << e_pos_rot, e_pos_disp;
	}

	inline void error(PosType const & cur_pos, VelType const & cur_vel, 
		PosType const & des_pos, VelType const & des_vel, 
		TaskVec & e_pos, TaskVec & e_vel)
	{
		TaskVecRot e_pos_rot;
		TaskVecRot e_vel_rot;
		_rot.error(cur_pos.rot, cur_vel.rot, des_pos.rot, des_vel.rot, e_pos_rot, e_vel_rot);
		
		TaskVecDisp e_pos_disp;
		TaskVecDisp e_vel_disp;
		_disp.error(cur_pos.disp, cur_vel.disp, des_pos.disp, des_vel.disp, e_pos_disp, e_vel_disp);

		e_pos << e_pos_rot, e_pos_disp;
		e_vel << e_vel_rot, e_vel_disp;
	}

	inline void error(PosType const & cur_pos, PosType const & des_pos, TaskVec & e_pos)
	{
		TaskVecRot e_pos_rot;
		_rot.error(cur_pos.rot, des_pos.rot, e_pos_rot);
		
		TaskVecDisp e_pos_disp;
		_disp.error(cur_pos.disp, des_pos.disp, e_pos_disp);

		e_pos << e_pos_rot, e_pos_disp;
	}

	/// FIXME @20151015
	inline void resetIntError()
	{
		_rot.resetIntError();
		_disp.resetIntError();
	}

	/// ADDED @20161017
	inline void setImpedanceDir(int axis, int val = 1)
	{
		if (axis < 3)
			_rot.setImpedanceDir(axis, val);
		else
			_disp.setImpedanceDir(axis - 3, val);

		_impedanceDir[axis] = val;
	}

private:
	RotationController _rot;
	PositionController<3> _disp;
};

//  ---------------------- Doxygen info ----------------------
//! \class CompositeHTransformTaskInterpolator
//!
//! \brief
//! This defines the composite transform task controller. 
//! 
//! \details 
//!	 Composite transform task controller is defined by the rotation controller and 
//! displacement controller corresponding to composite transform task kinematics variables. 
//!
//! \tparam DispInterpolatorType Displacement interpolator type 
//! \tparam RotInterpolatorType Rotation interpolator type
//!
//! \sa DisplacementInterpolator
//! \sa RotationInterpolator
//! \sa HTransformInterpolator
//  ----------------------------------------------------------
template <typename DispInterpolatorType, typename RotInterpolatorType = RotationInterpolator<DispInterpolatorType> >
class CompositeHTransformTaskInterpolator
{
public:
	typedef CompositeHTransformTaskPosition PosType;
	typedef CompositeHTransformTaskVelocity VelType;
	typedef VelType AccType;

public:
	inline void setTraj(PosType const & pos, VelType const & vel = VelType::Zero(), AccType const & acc = AccType::Zero())
	{
		_rot.setTraj(pos.rot, vel.rot, acc.rot);
		_disp.setTraj(pos.disp, vel.disp, acc.disp);
	}

	inline void setInitialTraj(double t_ini, PosType const & pos_ini, VelType const & vel_ini = VelType::Zero(), AccType const & acc_ini = AccType::Zero())
	{
		_rot.setInitialTraj(t_ini, pos_ini.rot, vel_ini.rot, acc_ini.rot);
		_disp.setInitialTraj(t_ini, pos_ini.disp, vel_ini.disp, acc_ini.disp);
	}

	inline void setTargetTraj(double t_final, PosType const & pos_final, VelType const & vel_final = VelType::Zero(), AccType const & acc_final = AccType::Zero())
	{
		_rot.setTargetTraj(t_final, pos_final.rot, vel_final.rot, acc_final.rot);
		_disp.setTargetTraj(t_final, pos_final.disp, vel_final.disp, acc_final.disp);
	}

	inline void traj(double t, PosType & des_pos, VelType & des_vel, AccType & des_acc)
	{
		_rot.traj(t, des_pos.rot, des_vel.rot, des_acc.rot);
		_disp.traj(t, des_pos.disp, des_vel.disp, des_acc.disp);
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! This returns whether the robot reached target.
	//  ----------------------------------------------------------
	inline bool isTargetReached() { return (_rot.isTargetReached() && _disp.isTargetReached()); }

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the period for cyclic update of trajectory
	//!
	//! \param delT period (in sec) (default value = 0)
	//  ----------------------------------------------------------
	inline void setPeriod(double delT)
	{
		_rot.setPeriod(delT); 
		_disp.setPeriod(delT); 
	}

	int getCurSeg()
	{
		if( _disp.getCurSeg() > _rot.getCurSeg())
			return _disp.getCurSeg();
		else
			return _rot.getCurSeg();
	}

private:
	RotInterpolatorType _rot;
	DispInterpolatorType _disp; 
};

}

