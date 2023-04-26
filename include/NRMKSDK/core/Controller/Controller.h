//  ---------------------- Doxygen info ----------------------
//! \file Controller.h
//!
//! \brief
//! Header file for the class Controller (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a controller base class
//! to be used for the interface of NRMKFoundation library.
//!  
//! \n
//! \n
//! \n
//! \copydetails Neuromeka Foundation Library
//! \n
//! \n
//! \n
//! Neuromeka Co., Ltd. \n
//! South KoreaY\n
//! \n
//! http://www.neuromeka.com\n
//!
//! \date December 2013
//! 
//! \version 1.9.4
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note v1.9.4 (20160511): changed resetIntError() by CRTP
//!
//! \note Copyright (C) 2013-2016 Neuromeka Co., Ltd.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#include <Eigen/Eigen>

namespace NRMKFoundation
{
namespace internal
{
}

//  ---------------------- Doxygen info ----------------------
//! \class Controller
//!
//! \brief
//! This implements a base controller class. 
//!
//! \details
//! Class Controller models any type of controllers which computes the acceleration-level reference 
//! and the velocity-level reference in terms of the position-level error and the velocity-level error for any kind of tasks.
//! They are called the task reference acceleration, task reference velocity, task position error, and task velocity error, respectively.
//! These errors are computed by the difference of the current position-level value (called task position) from the position-level desired value (called desired task position)
//! and the difference of the current velocity-level value (called task velocity) from the velocity level desired value (called desired task velocity).
//! In many cases, the acceleration-level desired value (a.k.a. desired task acceleration) is necessary to compute the acceleration-level reference value. 
//! Since there may exists many types to represent task position, task velocity, and task acceleration,
//! they are specified as the template arguments. 
//! 
//! Since ver.1.6, reference acceleration computation induced by impedance control is also generated in order to accommodate
//! possible force error. 
//!
//! In order to enforce the static polymorphism the derived class should inherit the class by the CRTP pattern. 
//! See http://en.wikipedia.org/wiki/Curiously_recurring_template_pattern.
//!  
//! \note 
//! Although task position is of type PosType, task velocity of type VelType, task position error and velocity error as well as task reference velocity
//! and task reference acceleration should be of type TaskVec. This is the vector of dimension same as the task dimension.
//!
//! \tparam _DIM Dimension of the task variable
//! \tparam ControllerType Derived class
//! \tparam _PosType Type of the position-level task variable. 
//!		The default type is Eigen::Matrix<double, _DIM, 1>.
//! \tparam _VelType Type of the velocity-level task variable.
//!		The default type is Eigen::Matrix<double, _DIM, 1>.
//! \tparam _AccType Type of the acceleration-level task variable.
//!		The default type is same as _VelType.
//! \tparam _ForceType Type of the force-level task variable.
//!		The default type is same as _VelType.
//!
//! \sa PositionController
//! \sa VelocityController
//! \sa RotationController
//! \sa HTransformController
//  ----------------------------------------------------------
template<int _DIM, typename ControllerType, typename _PosType = Eigen::Matrix<double, _DIM, 1>, typename _VelType = Eigen::Matrix<double, _DIM, 1>, typename _AccType = _VelType, typename _ForceType = _VelType>
class Controller
{	
public:
	//  ---------------------- Doxygen info ----------------------
	//! \enum 
	//! 
	//! \brief Provides compile-time constants 
	//  ----------------------------------------------------------
	enum 
	{	
		DIM = _DIM, //!< Task dimension
	};

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides Typedefs 
	//  ----------------------------------------------------------
	typedef _PosType PosType; //!< Typedef of the type for position variable
	typedef _VelType VelType; //!< Typedef of the type for velocity variable
	typedef _AccType AccType; //!< Typedef of the type for acceleration variable
	typedef _ForceType ForceType; //!< Typedef of the type for force variable
	typedef Eigen::Matrix<double, DIM, 1> TaskVec; //!< Typedef of task vector
	typedef Eigen::Matrix<double, DIM, DIM> TaskMat; //!< Typedef of task matrix
	/// ADDED @20161017
	//typedef Eigen::Matrix<int, DIM, 1> TaskSelVec; //!< Typedef of task selection vector

	//! \returns Reference to the derived object 
	inline ControllerType& derived() { return *static_cast<ControllerType*>(this); }
	//! \returns Constant reference to the derived object 
	inline const ControllerType& derived() const { return *static_cast<const ControllerType*>(this); }

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief constructs the default controller 
	//  ----------------------------------------------------------
	Controller()
	{
		_int_error.setZero();
		_int_ef.setZero();
		_ref.setZero();

		_kp.setZero();
		_kv.setZero();
		_ki.setZero();
		_invL2sqr.setZero();
		_kf.setZero();
		_kfi.setZero();

		/// FIXME @20151103
		_mass.setIdentity();
		_damping.setZero();
		_stiffness.setZero();

		/// ADDED @20161017
		_impedanceDir.setConstant(1);

		/// ADDED @20170118
#ifdef _PASSIVITY_BASED_CONTROL
		_passivity_based = false;
#else
		_passivity_based = true;
#endif
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity and acceleration 
	//!
	//! \details
	//! One may compute _ref, or reference error feedback, inside this function
	//! in order to call refErrorFeedback as well as update the integral errors.
	//! 
	//! \param cur_pos Current task position
	//! \param cur_vel Current task velocity 
	//! \param des_pos Desired task position 
	//! \param des_vel Desired task velocity 
	//! \param des_acc Desired task acceleration 
	//! \param ref_vel Task reference velocity 
	//! \param ref_acc Task reference acceleration 
	//  ----------------------------------------------------------
	inline void refAcceleration(PosType const & cur_pos, VelType const & cur_vel, 
					PosType const & des_pos, VelType const & des_vel, AccType const & des_acc, 
					TaskVec & ref_vel, TaskVec & ref_acc)
	{
		derived().refAcceleration(cur_pos, cur_vel, des_pos, des_vel, des_acc, ref_vel, ref_acc);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity and acceleration as well as the task position and velocity error
	//!
	//! \details
	//! One may compute _ref, or reference error feedback, inside this function
	//! in order to call refErrorFeedback as well as update the integral errors.
	//!
	//! \note
	//! The task position error is of type VelType (not PosType), and 
	//! the task velocity error is of type AccType (not VelType).
	//! 
	//! \param cur_pos Current task position
	//! \param cur_vel Current task velocity 
	//! \param des_pos Desired task position 
	//! \param des_vel Desired task velocity 
	//! \param des_acc Desired task acceleration 
	//! \param ref_vel Task reference velocity 
	//! \param ref_acc Task reference acceleration 
	//! \param e_pos Task position error 
	//! \param e_vel Task velocity error 
	//  ----------------------------------------------------------
	inline void refAcceleration(PosType const & cur_pos, VelType const & cur_vel, 
		PosType const & des_pos, VelType const & des_vel, AccType const & des_acc, 
		TaskVec & ref_vel, TaskVec & ref_acc, 
		TaskVec & e_pos, TaskVec & e_vel)
	{
		derived().refAcceleration(cur_pos, cur_vel, des_pos, des_vel, des_acc, ref_vel, ref_acc, e_pos, e_vel);
	}

	/// FIXME @20151015	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference force 
	//!
	//! \details
	//! One may need to compute _ref, or reference error, as well as integral errors inside this function.
	//!
	//! \note
	//! The task position error is of type VelType (not PosType),  
	//! the task velocity error is of type AccType (not VelType), and 
	//! the task force error is of type AccType (not VelType), and 
	//!
	//! \param cur_pos Current task position
	//! \param cur_vel Current task velocity 
	//! \param des_pos Desired task position 
	//! \param des_vel Desired task velocity 
	//! \param des_acc Desired task acceleration
	//! \param cur_force Current task force
	//! \param des_force Desired task force  
	//! \param ref_force Task reference force
	//! \param e_pos Task position error 
	//! \param e_vel Task velocity error 
	//! \param e_force Task force error 
	//  ----------------------------------------------------------
	inline void refComplianceForce(PosType const & cur_pos, VelType const & cur_vel,
			PosType const & des_pos, VelType const & des_vel, AccType const & des_acc,
			ForceType const & cur_force, ForceType const & des_force,
			ForceType & ref_force)
	{
		derived().refComplianceForce(cur_pos, cur_vel, des_pos, des_vel, des_acc, cur_force, des_force, ref_force);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference force 
	//!
	//! \details
	//! One may need to compute _ref, or reference error, as well as integral errors inside this function.
	//!
	//! \note
	//! The task position error is of type VelType (not PosType),  
	//! the task velocity error is of type AccType (not VelType), and 
	//! the task force error is of type AccType (not VelType), and 
	//!
	//! \param cur_pos Current task position
	//! \param cur_vel Current task velocity 
	//! \param des_pos Desired task position 
	//! \param des_vel Desired task velocity 
	//! \param des_acc Desired task acceleration
	//! \param cur_force Current task force
	//! \param des_force Desired task force  
	//! \param ref_force Task reference force
	//! \param e_pos Task position error 
	//! \param e_vel Task velocity error 
	//! \param e_force Task force error 
	//  ----------------------------------------------------------
	inline void refComplianceForce(PosType const & cur_pos, VelType const & cur_vel,
			PosType const & des_pos, VelType const & des_vel, AccType const & des_acc,
			ForceType const & cur_force, ForceType const & des_force,
			ForceType & ref_force,
			TaskVec & e_pos, TaskVec & e_vel,
			TaskVec & e_force)
	{
		derived().refComplianceForce(cur_pos, cur_vel, des_pos, des_vel, des_acc, cur_force, des_force, ref_force, e_pos, e_vel, e_force);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity and acceleration accommodating the task force error
	//!
	//! \details
	//! One may need to compute _ref, or reference error,  as well as integral errors inside this function.
	//!
	//! \note
	//! The task position error is of type VelType (not PosType),  
	//! the task velocity error is of type AccType (not VelType), and 
	//! the task force error is of type AccType (not VelType), and 
	//! 
	//! \param cur_pos Current task position
	//! \param cur_vel Current task velocity 
	//! \param des_pos Desired task position 
	//! \param des_vel Desired task velocity 
	//! \param des_acc Desired task acceleration
	//! \param cur_force Current task force
	//! \param des_force Desired task force  
	//! \param ref_vel Task reference velocity 
	//! \param ref_acc Task reference acceleration 
	//  ----------------------------------------------------------
	inline void refImpedanceAcceleration(PosType const & cur_pos, VelType const & cur_vel, 
		PosType const & des_pos, VelType const & des_vel, AccType const & des_acc, 
		ForceType const & cur_force, ForceType const & des_force,
		TaskVec & ref_vel, TaskVec & ref_acc)
	{
		derived().refImpedanceAcceleration(cur_pos, cur_vel, des_pos, des_vel, des_acc, cur_force, des_force, ref_vel, ref_acc);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity and acceleration as well as the task position and velocity error
	//! accommodating the task force error
	//!
	//! \details
	//! One may need to compute _ref, or reference error,  as well as integral errors inside this function.
	//!
	//! \note
	//! The task position error is of type VelType (not PosType),  
	//! the task velocity error is of type AccType (not VelType), and 
	//! the task force error is of type AccType (not VelType), and 
	//! 
	//! \param cur_pos Current task position
	//! \param cur_vel Current task velocity 
	//! \param des_pos Desired task position 
	//! \param des_vel Desired task velocity 
	//! \param des_acc Desired task acceleration
	//! \param cur_force Current task force
	//! \param des_force Desired task force  
	//! \param ref_vel Task reference velocity 
	//! \param ref_acc Task reference acceleration 
	//! \param e_pos Task position error 
	//! \param e_vel Task velocity error 
	//! \param e_force Task force error 
	//  ----------------------------------------------------------
	inline void refImpedanceAcceleration(PosType const & cur_pos, VelType const & cur_vel, 
		PosType const & des_pos, VelType const & des_vel, AccType const & des_acc, 
		ForceType const & cur_force, ForceType const & des_force,
		TaskVec & ref_vel, TaskVec & ref_acc, 
		TaskVec & e_pos, TaskVec & e_vel,
		TaskVec & e_force)
	{
		derived().refImpedanceAcceleration(cur_pos, cur_vel, des_pos, des_vel, des_acc, cur_force, des_force, ref_vel, ref_acc, e_pos, e_vel, e_force);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the reference velocity only
	//! 
	//! \param cur_pos Current task position
	//! \param des_pos Desired task position 
	//! \param des_vel Desired task velocity 
	//! \param ref_vel Task reference velocity 
	//  ----------------------------------------------------------
	inline void refVelocity(PosType const & cur_pos, 
		PosType const & des_pos, VelType const & des_vel, 
		TaskVec & ref_vel)
	{
		derived().refVelocity(cur_pos, des_pos, des_vel, ref_vel);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the reference velocity as well as task position error
	//! 
	//! \note
	//! The task position error is of type VelType (not PosType).
	//!
	//! \param cur_pos Current task position
	//! \param des_pos Desired task position 
	//! \param des_vel Desired task velocity 
	//! \param ref_vel Task reference velocity 
	//! \param e_pos Task position error
	//  ----------------------------------------------------------
	inline void refVelocity(PosType const & cur_pos, 
		PosType const & des_pos, VelType const & des_vel, 
		TaskVec & ref_vel, 
		TaskVec & e_pos)
	{
		derived().refVelocity(cur_pos, des_pos, des_vel, ref_vel, e_pos);
	}
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task position and velocity error
	//!
	//! \param cur_pos Current task position
	//! \param cur_vel Current task velocity 
	//! \param des_pos Desired task position 
	//! \param des_vel Desired task velocity 
	//! \param e_pos Task position error 
	//! \param e_vel Task velocity error 
	//  ----------------------------------------------------------
	inline void error(PosType const & cur_pos, VelType const & cur_vel, 
		PosType const & des_pos, VelType const & des_vel, 
		TaskVec & e_pos, TaskVec & e_vel)
	{
		derived().error(cur_pos, cur_vel, des_pos, des_vel, e_pos, e_vel);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task position error only
	//!
	//! \param cur_pos Current task position 
	//! \param des_pos Desired task position 
	//! \param e_pos Task position error 
	//  ----------------------------------------------------------
	inline void error(PosType const & cur_pos, PosType const & des_pos, TaskVec & e_pos)
	{
		derived().error(cur_pos,des_pos, e_pos);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference force
	//!
	//! \details
	//! Task reference force is defined by users.
	//! In case of passivity-based inverse dynamics, it includes \f$ (1/\gamma^2)* e_{ref} \f$ where 
	//! \f$ e_{ref} \f$ is the task reference error, which is the difference between task reference velocity and task velocity. 
	//!	 In case of external force compensation, it should include the external force component.
	//!
	//! \note 
	//! Task reference force is of type TaskVec.
	//! 
	//! \return Reference force
	//  ----------------------------------------------------------
	inline TaskVec const & refForce() const
	{
		return _ref;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the control period to compute integral error
	//! 
	//! \param dT Control period
	//  ----------------------------------------------------------
	inline void setPeriod(double dT)
	{
		_dT = dT; 
	}

	/// ADDED @20161017
	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task position error feedback gain uniformly
	//!
	//! \param axis index
	//! \param k Position error feedback gain
	//  ----------------------------------------------------------
	inline void setPosGain(int axis, double k)
	{
		_kp[axis] = k;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task velocity error feedback gain uniformly
	//!
	//! \param axis index
	//! \param k Velocity error feedback gain
	//  ----------------------------------------------------------
	inline void setVelGain(int axis, double k)
	{
		_kv[axis] = k;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task integral error feedback gain uniformly
	//!
	//! \param axis index
	//! \param k Integral error feedback gain
	//  ----------------------------------------------------------
	inline void setIntGain(int axis, double k)
	{
		_ki[axis] = k;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task reference error feedback gain uniformly
	//!
	//! \param axis index
	//! \param k Reference error feedback gain
	//  ----------------------------------------------------------
	inline void setInvL2sqr(int axis, double k)
	{
		_invL2sqr[axis] = k;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task force error feedback gain uniformly
	//!
	//! \param axis index
	//! \param k Force error feedback gain
	//  ----------------------------------------------------------
	inline void setForceGain(int axis, double k)
	{
		_kf[axis] = k;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the integral force error feedback gain uniformly
	//!
	//! \param axis index
	//! \param k Integral force error feedback gain
	//  ----------------------------------------------------------
	inline void setForceIntGain(int axis, double k)
	{
		_kfi[axis] = k;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task position error feedback gain uniformly
	//! 
	//! \param k Position error feedback gain
	//  ----------------------------------------------------------
	inline void setPosGain(double k)
	{
		_kp.setConstant(k);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task velocity error feedback gain uniformly
	//! 
	//! \param k Velocity error feedback gain
	//  ----------------------------------------------------------
	inline void setVelGain(double k)
	{
		_kv.setConstant(k);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task integral error feedback gain uniformly
	//! 
	//! \param k Integral error feedback gain
	//  ----------------------------------------------------------
	inline void setIntGain(double k)
	{
		_ki.setConstant(k);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task reference error feedback gain uniformly
	//! 
	//! \param k Reference error feedback gain
	//  ----------------------------------------------------------
	inline void setInvL2sqr(double k)
	{
		_invL2sqr.setConstant(k);
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task force error feedback gain uniformly
	//! 
	//! \param k Force error feedback gain
	//  ----------------------------------------------------------
	inline void setForceGain(double k)
	{
		_kf.setConstant(k);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the integral force error feedback gain uniformly
	//! 
	//! \param k Integral force error feedback gain
	//  ----------------------------------------------------------
	inline void setForceIntGain(double k)
	{
		_kfi.setConstant(k);
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task position error feedback gain 
	//! 
	//! \param k Position error feedback gain vector
	//  ----------------------------------------------------------
	inline void setPosGain(TaskVec const & k)
	{
		_kp = k;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task velocity error feedback gain 
	//! 
	//! \param k Velocity error feedback gain vector
	//  ----------------------------------------------------------
	inline void setVelGain(TaskVec const & k)
	{
		_kv = k;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task integral error feedback gain 
	//! 
	//! \param k Integral error feedback gain vector
	//  ----------------------------------------------------------
	inline void setIntGain(TaskVec const & k)
	{
		_ki = k;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task reference error feedback gain 
	//! 
	//! \param k Reference error feedback gain vector
	//  ----------------------------------------------------------
	inline void setInvL2sqr(TaskVec const & k)
	{
		_invL2sqr = k;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task force error feedback gain 
	//! 
	//! \param k Force error feedback gain vector
	//  ----------------------------------------------------------
	inline void setForceGain(TaskVec const & k)
	{
		_kf = k;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the integral force error feedback gain 
	//! 
	//! \param k Integral force error feedback gain vector
	//  ----------------------------------------------------------
	inline void setForceIntGain(TaskVec const & k)
	{
		_kfi = k;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief returns the task position error feedback gain 
	//  ----------------------------------------------------------
	inline TaskVec const &  kp() const
	{
		return _kp;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief returns the task velocity error feedback gain 
	//  ----------------------------------------------------------
	inline TaskVec const &  kv() const
	{
		return _kv;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task integral error feedback gain 
	//! 
	//! \param k Integral error feedback gain vector
	//  ----------------------------------------------------------
	inline TaskVec const &  ki() const
	{
		return _ki;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the task reference error feedback gain 
	//! 
	//! \param k Reference error feedback gain vector
	//  ----------------------------------------------------------
	inline TaskVec const &  invL2sqr() const
	{
		return _invL2sqr;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief returns the task force error feedback gain 
	//  ----------------------------------------------------------
	inline TaskVec const & kf() const
	{
		return _kf;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief returns the integral force error feedback gain 
	//  ----------------------------------------------------------
	inline TaskVec const & kfi() const
	{
		return _kfi;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief sets the gains of sub-task controllers (if any) 
	//! in terms of the specified gains, _kp, _kv, _ki, and _delT
	//! as well as _invL2sqr if needed.
	//! 
	//! \note
	//! When the derived controller has no subcontrollers,
	//! it is really important to define this function 
	//! to have null statement explicitly. This prevents recursively defining this function. 
	//  ----------------------------------------------------------
	inline void setSubGains()
	{
		derived().setSubGains();
	}

	/// FIXME @20160511
	inline void resetIntError()
	{
		//_int_error.setZero();
		//_int_ef.setZero();
		derived().resetIntError();
	}

	/// FIXME @20151015
	inline TaskVec const & int_error() const
	{
		return _int_error;
	}
	
	inline TaskVec const & int_ef() const
	{
		return _int_ef;
	}

	/// FIXME @201511103
	inline TaskMat const & stiffness() const
	{
		return _stiffness;
	}

	inline TaskMat const & damping() const
	{
		return _damping;
	}

	inline TaskMat const & mass() const
	{
		return _mass;
	}

	inline void setImpedanceDir(int axis, int val = 1) 
	{
		_impedanceDir[axis] = val;
	}

	inline void setPassivityBased(bool flag = true) 
	{
		_passivity_based = flag; 
	}

	inline bool getPassivityBased() 
	{
		return _passivity_based; 
	}
	
public:
 	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Position error feedback gain vector
	//  ----------------------------------------------------------
	TaskVec _kp;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Velocity error feedback gain vector
	//  ----------------------------------------------------------
	TaskVec _kv;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Integral error feedback gain vector
	//  ----------------------------------------------------------
	TaskVec _ki;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Reference error feedback gain vector
	//  ----------------------------------------------------------
	TaskVec _invL2sqr;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Force error feedback gain vector
	//  ----------------------------------------------------------
	TaskVec _kf;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Integral force error feedback gain vector
	//  ----------------------------------------------------------
	TaskVec _kfi;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Control period
	//  ----------------------------------------------------------
	double _dT;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Task integral error vector
	//  ----------------------------------------------------------
	TaskVec _int_error;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Integral task force error vector
	//  ----------------------------------------------------------
	TaskVec _int_ef;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Task reference error vector
	//  ----------------------------------------------------------
	TaskVec _ref;

	/// FIXME @20151103
	// Not fully implemented yet..
	//  ---------------------- Doxygen info ----------------------
	//! \brief Task inertia (in impedance context)
	//  ----------------------------------------------------------
	TaskMat _mass;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Task inertia (in impedance context)
	//  ----------------------------------------------------------
	TaskMat _damping;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Task inertia (in impedance context)
	//  ----------------------------------------------------------
	TaskMat _stiffness;

	/// ADDED @20161017
	//! \brief Task selection for impedance control
	// TaskSelVec _impedanceDir;
	TaskVec _impedanceDir;

	/// ADDED @20170118
	//! \brief Flag for activating passivity_based control
	bool _passivity_based;
};	// class Controller

//  ---------------------- Doxygen info ----------------------
//! \class Impedance
//!
//! \brief
//! This implements an impedance class which generates a set of gains 
//! from the specified impedance parameters.
//!
//! \tparam DIM Dimension of the task variable
//  ----------------------------------------------------------
template <int DIM>
class Impedance
{
public:
	typedef Eigen::Matrix<double, DIM, 1> TaskVec;

	void setMass(double m)
	{
		_m.setConstant(m);
	}

	void setDamping(double d)
	{
		_d.setConstant(d);
	}

	void setStiffness(double k)
	{
		_k.setConstant(k);
	}

	void setMass(TaskVec const & m)
	{
		_m = m;
	}

	void setDamping(TaskVec const & d)
	{
		_d = d;
	}

	void setStiffness(TaskVec const & k)
	{
		_k = k;
	}

	void generateGain(TaskVec & kp, TaskVec & kv, TaskVec & kf)
	{
		kv = _d.cwiseQuotient(_m);
		kp = _k.cwiseQuotient(_m);
		kf = _m.cwiseInverse();
	}

private:
	TaskVec _m;
	TaskVec _d;
	TaskVec _k;
};

} // namespace NRMKFoundation
