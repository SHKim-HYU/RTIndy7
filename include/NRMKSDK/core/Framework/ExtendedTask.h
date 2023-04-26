//  ---------------------- Doxygen info ----------------------
//! \file ExtendedTask.h
//!
//! \brief
//! Header file for the class ExtendedTask (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a extended task kinematics for redundant manipulators 
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
//! \date March 2016
//! 
//! \version 1.9.4
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note v1.9.4 (20160323): added setWeightMatrix() for weighted pseudoinverse
//! \note v1.9.4 (20160426): added refComplianceForce() for compliance control
//! \note v1.9.4 (20160511): added resetIntError()
//!
//! \note Copyright (C) 2013-2016 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once 

#include "LieGroup/LieGroup.h"

#include "../Kinematics/Kinematics.h"
#include "../Kinematics/QuasiCoordKinematics.h"

#include "../Controller/VelocityController.h"

namespace NRMKFoundation
{
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
template<typename TaskKinematics>
struct ExtendedTaskPosition
{
	typedef typename TaskKinematics::SubsysType SubsysType;
	typedef NullVelocityKinematics<SubsysType, TaskKinematics> NullKinematics;
	typedef Eigen::Matrix<double, SubsysType::JOINT_DOF, 1> ExtendedTaskVec;

	ExtendedTaskPosition() : task(), null(NullKinematics::NullVec::Zero()) 
	{
	}

	/// ADDED @20160426
	//! \brief returns the identity element
	inline static ExtendedTaskPosition const Zero()
	{
		return ExtendedTaskPosition();
	}

	ExtendedTaskVec asVector()
	{
		Eigen::Matrix<double, 6, 1> taskvec;
		ExtendedTaskVec res;
		taskvec = task.asVector();

		for(int i = 0 ; i < 6 ; i++) res[i] = taskvec[i];

		return res;
	}

	LieGroup::Rotation & R() { return task.R(); }
	LieGroup::Displacement & r() { return task.r(); }

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typename TaskKinematics::PosType task;
	typename NullKinematics::NullVec null; 
};

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
template<typename TaskKinematics>
struct ExtendedTaskVelocity
{
	typedef typename TaskKinematics::SubsysType SubsysType;
	typedef NullVelocityKinematics<SubsysType, TaskKinematics> NullKinematics;
	typedef Eigen::Matrix<double, SubsysType::JOINT_DOF, 1> ExtendedTaskVec;


	ExtendedTaskVelocity() : task(), null(NullKinematics::NullVec::Zero()) 
	{
	}

	/// ADDED @20160426
	//! \brief returns the identity element
	inline static ExtendedTaskVelocity const Zero()
	{
		return ExtendedTaskVelocity();
	}

	ExtendedTaskVec asVector()
	{
		Eigen::Matrix<double, 6, 1> taskvec;
		ExtendedTaskVec res;
		taskvec = task.asVector();

		for(int i = 0 ; i < 6 ; i++) res[i] = taskvec[i];

		return res;
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typename TaskKinematics::VelType task;
	typename NullKinematics::NullVec null; 
};

//  ---------------------- Doxygen info ----------------------
//! \class ExtendedTaskVelocity
//!
//! \brief
//! This defines the acceleration variable for  extended task kinematics. 
//! 
//! \details 
//!	 Extended task accelerations are defined by extending the primary task acceleration variable 
//! with a null acceleration variable. 
//!
//! \tparam TaskKinematics Primary task kinematics
//  ----------------------------------------------------------
template<typename TaskKinematics>
struct ExtendedTaskAcceleration
{
	typedef typename TaskKinematics::SubsysType SubsysType;
	typedef NullVelocityKinematics<SubsysType, TaskKinematics> NullKinematics;
	typedef Eigen::Matrix<double, SubsysType::JOINT_DOF, 1> ExtendedTaskVec;

	// initialization is always a good habit.
	ExtendedTaskAcceleration() : task(), null(NullKinematics::NullVec::Zero()) 
	{
	}

	/// ADDED @20160426
	//! \brief returns the identity element
	inline static ExtendedTaskAcceleration const Zero()
	{
		return ExtendedTaskAcceleration();
	}

	ExtendedTaskVec asVector()
	{
		Eigen::Matrix<double, 6, 1> taskvec;
		ExtendedTaskVec res;
		taskvec = task.asVector();

		for(int i = 0 ; i < 6 ; i++) res[i] = taskvec[i];

		return res;
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typename TaskKinematics::AccType task;
	typename NullKinematics::NullVec null; 
};

//  ---------------------- Doxygen info ----------------------
//! \class ExtendedTaskForce
//!
//! \brief
//! This defines the force variable for  extended task kinematics. 
//! 
//! \details 
//!	 Extended task forces are defined by extending the primary task force variable 
//! with a null force variable. 
//!
//! \tparam TaskKinematics Primary task kinematics
//  ----------------------------------------------------------
template<typename TaskKinematics>
struct ExtendedTaskForce
{
	typedef typename TaskKinematics::SubsysType SubsysType;
	typedef NullVelocityKinematics<SubsysType, TaskKinematics> NullKinematics;

	// initialization is always a good habit.
	ExtendedTaskForce() : task(), null(NullKinematics::NullVec::Zero()) 
	{
	}

	/// ADDED @20160426
	//! \brief returns the identity element
	inline static ExtendedTaskForce const Zero()
	{
		return ExtendedTaskForce();
	}

	LieGroup::Vector3D & f(){ return task.f(); }
	LieGroup::Vector3D & n(){ return task.n(); }

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typename TaskKinematics::ForceType task;
	typename NullKinematics::NullVec null; 
};

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
template<typename TaskKinematics>
class ExtendedTaskKinematics
	: public Kinematics<typename TaskKinematics::SubsysType, TaskKinematics::SubsysType::JOINT_DOF, ExtendedTaskKinematics<TaskKinematics>, 
			ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >
{
public:
	enum 
	{
		JOINT_DOF = TaskKinematics::SubsysType::JOINT_DOF,
	};

	typedef typename TaskKinematics::SubsysType SubsysType;
	typedef typename SubsysType::JointMat JointMat;

	typedef NullVelocityKinematics<SubsysType, TaskKinematics> NullKinematics;

	typedef typename Kinematics<SubsysType, JOINT_DOF, ExtendedTaskKinematics<TaskKinematics>, 
			ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename Kinematics<SubsysType, JOINT_DOF, ExtendedTaskKinematics<TaskKinematics>, 
			ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >::TaskJac TaskJac; //!< Typedef of task Jacobian

public:
	inline ExtendedTaskKinematics(SubsysType const & subsys)
		: _task(subsys)
		, _null(subsys)
	{
	}

	inline void kinematics(SubsysType const & subsys, ExtendedTaskPosition<TaskKinematics> & p, ExtendedTaskVelocity<TaskKinematics> & v) const
	{		
		_task.kinematics(subsys, p.task, v.task);
	}

	inline void kinematics(SubsysType const & subsys, ExtendedTaskPosition<TaskKinematics> & p) const
	{		
		_task.kinematics(subsys, p.task);
	}

	inline void jacobian(SubsysType const & subsys, ExtendedTaskPosition<TaskKinematics> & p, ExtendedTaskVelocity<TaskKinematics> & v, TaskJac & extJ, TaskJac & extJdot) const
	{		
		typename TaskKinematics::TaskJac J;
		typename TaskKinematics::TaskJac Jdot;

		_task.jacobian(subsys, p.task, v.task, J, Jdot);

		typename NullKinematics::NullJac Z;
		typename NullKinematics::NullJac Zdot;

		//_null.computeNullSpaceBases(J, Jdot, Z, Zdot, _Jinv);
		_null.computeNullSpaceBases(J, Jdot, Z, Zdot);
		_null.jacobian(subsys, p.null, v.null, Z, Zdot);

		extJ << J, Z;
		extJdot << Jdot, Zdot;
	}

	inline void jacobian(SubsysType const & subsys, ExtendedTaskPosition<TaskKinematics> & p, TaskJac & extJ) const
	{		
		typename TaskKinematics::TaskJac J;

		_task.jacobian(subsys, p.task, J);

		typename NullKinematics::NullJac Z;

		//_null.computeNullSpaceBases(J, Z, _Jinv);
		_null.computeNullSpaceBases(J, Z);
		_null.jacobian(subsys, p.null, Z);

		extJ << J, Z;
	}

	TaskKinematics & taskKinematics() { return _task; }
	NullKinematics & nullKinematics() { return _null; }

	//inline Eigen::Matrix<double, JOINT_DOF, TaskKinematics::DIM> const & Jinv() const 
	//{
	//	return _null.Jinv();
	//}

	/// ADDED @20160323 
	inline void setWeightMatrix(JointMat const & W, JointMat const & Wdot) 
	{
		_null.setWeightMatrix(W, Wdot);
	}

	inline void setWeightMatrix(JointMat const & W) 
	{
		_null.setWeightMatrix(W);
	}

	inline void set(int target, LieGroup::HTransform const & Ttarget, LieGroup::HTransform const & Tref)
	{
		_task.set(target, Ttarget, Tref);
	}

private:
	TaskKinematics _task;
	NullKinematics _null; 

	//  ---------------------- Doxygen info ----------------------
	//! \brief pseudoinverse of the task Jacobian
	//  ----------------------------------------------------------
	//mutable Eigen::Matrix<double, JOINT_DOF, TaskKinematics::DIM> _Jinv;
};

//  ---------------------- Doxygen info ----------------------
//! \class ExtendedTaskController
//!
//! \brief
//! This defines the extended task controller. 
//! 
//! \details 
//!	 Extended task controller is defined by extending the primary task controller 
//! with a null velocity controller for the quasi-coordinate null velocity variables. 
//!
//! \tparam TaskKinematics Primary task kinematics
//! \tparam TaskController Primary task controller
//!
//! \sa VelocityController
//  ----------------------------------------------------------
template<typename TaskKinematics, typename TaskController>
class ExtendedTaskController
	: public Controller<TaskKinematics::SubsysType::JOINT_DOF, ExtendedTaskController<TaskKinematics, TaskController>, 
			ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >
{
public:
	typedef typename TaskKinematics::SubsysType SubsysType;
	typedef NullVelocityKinematics<SubsysType, TaskKinematics> NullKinematics;

	typedef VelocityController<NullKinematics::DIM> NullController;

	typedef typename Controller<TaskKinematics::SubsysType::JOINT_DOF, ExtendedTaskController<TaskKinematics, TaskController>, 
		ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >::TaskVec TaskVec; //!< Typedef of task vector

	typedef typename Controller<TaskKinematics::SubsysType::JOINT_DOF, ExtendedTaskController<TaskKinematics, TaskController>, 
		ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >::PosType PosType; //!< Typedef of task position
	typedef typename Controller<TaskKinematics::SubsysType::JOINT_DOF, ExtendedTaskController<TaskKinematics, TaskController>, 
		ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >::VelType VelType; //!< Typedef of task velocity
	typedef typename Controller<TaskKinematics::SubsysType::JOINT_DOF, ExtendedTaskController<TaskKinematics, TaskController>, 
		ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >::AccType AccType; //!< Typedef of task acceleration
	typedef typename Controller<TaskKinematics::SubsysType::JOINT_DOF, ExtendedTaskController<TaskKinematics, TaskController>, 
		ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >::ForceType ForceType; //!< Typedef of task force 

	typedef typename TaskController::TaskVec TaskVecTask;
	typedef typename NullController::TaskVec TaskVecNull;

public:
	// distribute the gains to the sub-task controllers
	inline void setSubGains() 
	{
		task.setPeriod(_dT);
#ifdef __GNUC__
		task.setPosGain(_kp.template head<TaskController::DIM>());
		task.setVelGain(_kv.template head<TaskController::DIM>());
		task.setIntGain(_ki.template head<TaskController::DIM>());

		task.setForceGain(_kf.template head<TaskController::DIM>());
		task.setForceIntGain(_kfi.template head<TaskController::DIM>());
		task.setInvL2sqr(_invL2sqr.template head<TaskController::DIM>());
#else
		task.setPosGain(_kp.head<TaskController::DIM>());
		task.setVelGain(_kv.head<TaskController::DIM>());
		task.setIntGain(_ki.head<TaskController::DIM>());

		task.setForceGain(_kf.head<TaskController::DIM>());
		task.setForceIntGain(_kfi.head<TaskController::DIM>());
		task.setInvL2sqr(_invL2sqr.head<TaskController::DIM>());
#endif

		task.setSubGains();

		null.setPeriod(_dT);
#ifdef __GNUC__
		null.setPosGain(_kp.template tail<NullController::DIM>());
		null.setVelGain(_kv.template tail<NullController::DIM>());
		null.setIntGain(_ki.template tail<NullController::DIM>());

		null.setForceGain(_kf.template tail<NullController::DIM>());
		null.setForceIntGain(_kfi.template tail<NullController::DIM>());
		null.setInvL2sqr(_invL2sqr.template tail<NullController::DIM>());
#else
		null.setPosGain(_kp.tail<NullController::DIM>());
		null.setVelGain(_kv.tail<NullController::DIM>());
		null.setIntGain(_ki.tail<NullController::DIM>());

		null.setForceGain(_kf.tail<NullController::DIM>());
		null.setForceIntGain(_kfi.tail<NullController::DIM>());
		null.setInvL2sqr(_invL2sqr.tail<NullController::DIM>());
#endif

		null.setSubGains();
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
		TaskVecTask ref_vel_task;
		TaskVecTask ref_acc_task;
		TaskVecTask e_pos_task;
		TaskVecTask e_vel_task;
		task.refAcceleration(cur_pos.task, cur_vel.task, des_pos.task, des_vel.task, des_acc.task, ref_vel_task, ref_acc_task, e_pos_task, e_vel_task);

		TaskVecNull ref_vel_null;
		TaskVecNull ref_acc_null;
		TaskVecNull e_pos_null;
		TaskVecNull e_vel_null;
		null.refAcceleration(cur_pos.null, cur_vel.null, des_pos.null, des_vel.null, des_acc.null, ref_vel_null, ref_acc_null, e_pos_null, e_vel_null);

		ref_vel << ref_vel_task, ref_vel_null;
		ref_acc << ref_acc_task, ref_acc_null;
		e_pos << e_pos_task, e_pos_null;
		e_vel << e_vel_task, e_vel_null;

		// FIXED @ 20131216
		_ref << task.refForce(), null.refForce();
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
		TaskVecTask ref_vel_task;
		TaskVecTask ref_acc_task;
		TaskVecTask e_pos_task;
		TaskVecTask e_vel_task;
		TaskVecTask e_force_task;
		task.refImpedanceAcceleration(cur_pos.task, cur_vel.task, des_pos.task, des_vel.task, des_acc.task, cur_force.task, des_force.task, ref_vel_task, ref_acc_task, e_pos_task, e_vel_task, e_force_task);

		TaskVecNull ref_vel_null;
		TaskVecNull ref_acc_null;
		TaskVecNull e_pos_null;
		TaskVecNull e_vel_null;
		TaskVecNull e_force_null;
		null.refImpedanceAcceleration(cur_pos.null, cur_vel.null, des_pos.null, des_vel.null, des_acc.null, cur_force.null, des_force.null, ref_vel_null, ref_acc_null, e_pos_null, e_vel_null, e_force_null);

		ref_vel << ref_vel_task, ref_vel_null;
		ref_acc << ref_acc_task, ref_acc_null;
		e_pos << e_pos_task, e_pos_null;
		e_vel << e_vel_task, e_vel_null;
		e_force << e_force_task, e_force_null;

		// FIXED @ 20131216
		_ref << task.refForce(), null.refForce();
	}

	/// ADDED @20160426
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
		TaskVecTask e_pos_task;
		TaskVecTask e_vel_task;
		TaskVecTask e_force_task;
		task.refComplianceForce(cur_pos.task, cur_vel.task, des_pos.task, des_vel.task, des_acc.task, cur_force.task, des_force.task, ref_force.task, e_pos_task, e_vel_task, e_force_task);

		TaskVecNull e_pos_null;
		TaskVecNull e_vel_null;
		TaskVecNull e_force_null;
		null.refComplianceForce(cur_pos.null, cur_vel.null, des_pos.null, des_vel.null, des_acc.null, cur_force.null, des_force.null, ref_force.null, e_pos_null, e_vel_null, e_force_null);

		e_pos << e_pos_task, e_pos_null;
		e_vel << e_vel_task, e_vel_null;
		e_force << e_force_task, e_force_null;

		_ref << task.refForce(), null.refForce();
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
		TaskVecTask ref_vel_task;
		TaskVecTask e_pos_task;
		task.refVelocity(cur_pos.task, des_pos.task, des_vel.task, ref_vel_task, e_pos_task);

		TaskVecNull ref_vel_null;
		TaskVecNull e_pos_null;
		null.refVelocity(cur_pos.null, des_pos.null, des_vel.null, ref_vel_null, e_pos_null);

		ref_vel << ref_vel_task, ref_vel_null;
		e_pos << e_pos_task, e_pos_null;
	}

	inline void error(PosType const & cur_pos, VelType const & cur_vel, 
		PosType const & des_pos, VelType const & des_vel, 
		TaskVec & e_pos, TaskVec & e_vel)
	{
		TaskVecTask e_pos_task;
		TaskVecTask e_vel_task;
		task.error(cur_pos.task, cur_vel.task, des_pos.task, des_vel.task, e_pos_task, e_vel_task);

		TaskVecNull e_pos_null;
		TaskVecNull e_vel_null;
		null.error(cur_pos.null, cur_vel.null, des_pos.null, des_vel.null, e_pos_null, e_vel_null);

		e_pos << e_pos_task, e_pos_null;
		e_vel << e_vel_task, e_vel_null;
	}

	inline void error(PosType const & cur_pos, PosType const & des_pos, TaskVec & e_pos)
	{
		TaskVecTask e_pos_task;
		task.error(cur_pos.task, des_pos.task, e_pos_task);

		TaskVecNull e_pos_null;
		null.error(cur_pos.null, des_pos.null, e_pos_null);

		e_pos << e_pos_task, e_pos_null;
	}
	
	/// FIXME @20160511
	inline void resetIntError()
	{
		task.resetIntError();
		null.resetIntError();
	}

	inline void setPosGain(TaskVecTask const & k)
	{
		task.setPosGain(k);
	}

	inline void setVelGain(TaskVecTask const & k)
	{
		task.setVelGain(k);
	}

	inline void setInvL2sqr(TaskVecTask const & k)
	{
		task.setInvL2sqr(k);
	}

	inline void setForceGain(TaskVecTask const & k)
	{
		task.setForceGain(k);
	}

	inline void setSubGainTask()
	{
		task.setSubGains();
	}

private:
	using Controller<TaskKinematics::SubsysType::JOINT_DOF, ExtendedTaskController<TaskKinematics, TaskController>, 
			ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >::_kp;
	using Controller<TaskKinematics::SubsysType::JOINT_DOF, ExtendedTaskController<TaskKinematics, TaskController>, 
			ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >::_kv;
	using Controller<TaskKinematics::SubsysType::JOINT_DOF, ExtendedTaskController<TaskKinematics, TaskController>, 
			ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >::_ki;
	using Controller<TaskKinematics::SubsysType::JOINT_DOF, ExtendedTaskController<TaskKinematics, TaskController>, 
			ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >::_int_error;
	using Controller<TaskKinematics::SubsysType::JOINT_DOF, ExtendedTaskController<TaskKinematics, TaskController>, 
			ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >::_ref;
	using Controller<TaskKinematics::SubsysType::JOINT_DOF, ExtendedTaskController<TaskKinematics, TaskController>, 
			ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >::_dT;
	using Controller<TaskKinematics::SubsysType::JOINT_DOF, ExtendedTaskController<TaskKinematics, TaskController>, 
		ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >::_kf;
	using Controller<TaskKinematics::SubsysType::JOINT_DOF, ExtendedTaskController<TaskKinematics, TaskController>, 
		ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >::_kfi;
	using Controller<TaskKinematics::SubsysType::JOINT_DOF, ExtendedTaskController<TaskKinematics, TaskController>, 
		ExtendedTaskPosition<TaskKinematics>, ExtendedTaskVelocity<TaskKinematics>, ExtendedTaskAcceleration<TaskKinematics>, ExtendedTaskForce<TaskKinematics> >::_invL2sqr;

public:
	TaskController task;
	NullController null; 
};

} // namespace NRMKFoundation
