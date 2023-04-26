//  ---------------------- Doxygen info ----------------------
//! \file VelocityController.h
//!
//! \brief
//! Header file for the class VelocityController (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a position controller
//! to be used for the interface of NRMKFoundation library.
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
//! \date April 2016
//! 
//! \version 1.9.4
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note v1.9.4 (20160426): fixed refComplianceForce() for compliance control under Hinf optimal control
//! \note v1.9.4 (20160511): added resetIntError()
//!
//!
//! \note Copyright (C) 2013--2016 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#include "Controller.h"

namespace NRMKFoundation
{
namespace internal
{
}

//  ---------------------- Doxygen info ----------------------
//! \class VelocityController
//!
//! \brief
//! This implements a velocity controller class.
//! 
//! \details 
//! The velocity controller is used for the task whose velocity is described by vectors and the task position 
//! can not be defined. Such tasks are also called the quasi-coordinate task. 
//! In particular, task velocity are denoted by \f$ v \in \mathbb{R}^m \f$.
//! The desired task velocity, and acceleration are denoted by \f$ v_{des} \f$ and \f$ \dot{v}_{des} \f$,
//! and the reference velocity and acceleration by \f$ v_{ref} \f$, and \f$ \dot{v}_{ref} \f$, 
//! all vectors belonging to \f$ \mathbb{R}^m \f$. 
//!
//! \tparam DIM Task dimension
//!
//! \sa Controller
//  ----------------------------------------------------------
template <int DIM>
class VelocityController : public Controller<DIM, VelocityController<DIM> >
{	
public:
	typedef typename Controller<DIM, VelocityController<DIM> >::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename Controller<DIM, VelocityController<DIM> >::TaskMat TaskMat; //!< Typedef of task matrix

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a default velocity controller
	//  ----------------------------------------------------------
	VelocityController() : Controller<DIM, VelocityController>() 
	{
		_int_int_error.setZero();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief 
	//! sets the gains of sub-task controllers (if any) 
	//! in terms of the specified gains, _kp, _kv, _ki, and _delT
	//! as well as _invL2sqr if needed.
	//! 
	//! \note
	//! It is really important to define this function 
	//! to have null statement explicitly because it has no subcontrollers.
	//! This prevents recursively defining this function. 
	//  ----------------------------------------------------------
	inline void setSubGains()
	{		
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity and acceleration
	//!
	//! \details
	//! They are computed by \f$ \dot{v}_{ref} = \dot{v}_{des} + k_v e_v + k_p \int e_v dt \f$ and  
	//!  \f$ v_{ref} = v_{des} + k_v \int e_v dt + kp \int \int e_v dt dt \f$, where 
	//! the velocity error is defined by \f$ e_v = v_{des} - v \f$.
	//! 
	//! \note 
	//! The first argument for the current task position vector is neglected. 
	//!
	//! \param v Current task velocity vector or \f$ v \f$
	//! \param vd Desired task velocity vector or \f$ v_{des} \f$
	//! \param vdotd Desired task acceleration vector or \f$ \dot{v}_{des} \f$
	//! \param vref Task reference velocity vector or \f$ v_{ref} \f$
	//! \param vdotref Task reference acceleration vector or \f$ \dot{v}_{ref} \f$
	//  ----------------------------------------------------------
	inline void refAcceleration(TaskVec const &x, TaskVec const & v, 
						TaskVec const &xd, TaskVec const & vd, TaskVec const & vdotd, 
						TaskVec & vref, TaskVec & vdotref)
	{
		TaskVec ex; // don't care
		TaskVec ev;

		refAcceleration(x, v, xd, vd, vdotd, vref, vdotref, ex, ev);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity and acceleration as well as task position and velocity error
	//!
	//! \details
	//! They are computed by \f$ \dot{v}_{ref} = \dot{v}_{des} + k_v e_v + k_p \int e_v dt \f$ and  
	//!  \f$ v_{ref} = v_{des} + k_v \int e_v dt + kp \int \int e_v dt dt \f$, where 
	//! the velocity error is defined by \f$ e_v = v_{des} - v \f$.
	//! 
	//! \note 
	//! The first argument for the current task position vector 
	//! and the eighth argument for the task position error are neglected.
	//!
	//! \param v Current task velocity vector or \f$ v \f$
	//! \param vd Desired task velocity vector or \f$ v_{des} \f$
	//! \param vdotd Desired task acceleration vector or \f$ \dot{v}_{des} \f$
	//! \param vref Task reference velocity vector or \f$ v_{ref} \f$
	//! \param vdotref Task reference acceleration vector or \f$ \dot{v}_{ref} \f$
	//! \param ev Task velocity error vector or \f$ e_v \f$
	//  ----------------------------------------------------------
	inline void refAcceleration(TaskVec const &, TaskVec const & v, 
		TaskVec const &, TaskVec const & vd, TaskVec const & vdotd, 
		TaskVec & vref, TaskVec & vdotref, TaskVec & , TaskVec &ev)
	{
		ev = vd - v;

		vref = vd + _kv.cwiseProduct(_int_error) + _kp.cwiseProduct(_int_int_error);
		vdotref = vdotd + _kv.cwiseProduct(ev) + _kp.cwiseProduct(_int_error);

		// For reference error feedback
		/// FIXED @20170118
		if (_passivity_based)
			_ref.noalias() = _invL2sqr.cwiseProduct(vref - v);

		_int_error += _dT*ev;
		_int_int_error += _dT*_int_error;
	}
	
	/// FIXME @20151015
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference force by compliance
	//!
	//! \details
	//! They are computed by \f$ f_{ref} = \dot{v}_{des} + k_v e_v + k_p \int e_v dt - k_f e_f\f$, where 
	//! the position and velocity errors are defined by \f$ e = p_{des} - p \f$, \f$ \dot{e} = \dot{p}_{des} - \dot{p} \f$,
	//! and the force error by \f$ e_f = f_{des} - f \f$
	//!
	//! \param v Current task velocity vector or \f$ \dot{p} \f$
	//! \param vd Desired task velocity vector or \f$ \dot{p}_{des} \f$
	//! \param vdotd Desired task acceleration vector or \f$ \ddot{p}_{des} \f$
	//! \param f Current task force vector or \f$ f \f$
	//! \param fd Desired task force vector or \f$ f_{des} \f$
	//! \param fref Task reference force vector or \f$ f_{ref} \f$
	//  ----------------------------------------------------------
	inline void refComplianceForce(TaskVec const &, TaskVec const & v, 
		TaskVec const &, TaskVec const & vd, TaskVec const & vdotd, 
		TaskVec const & f, TaskVec const & fd, 
		TaskVec & fref)
	{
		TaskVec x;
		TaskVec xd;
		
		TaskVec ex; // don't care
		TaskVec ev;
		TaskVec ef;

		// FIXME 20160107
		refComplianceForce(x, v, xd, vd, vdotd, fref, ex, ev, ef);		
		
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference force by compliance
	//!
	//! \details
	//! They are computed by \f$ f_{ref} = \dot{v}_{des} + k_v e_v + k_p \int e_v dt - k_f e_f\f$, where 
	//! the position and velocity errors are defined by \f$ e = p_{des} - p \f$, \f$ \dot{e} = \dot{p}_{des} - \dot{p} \f$,
	//! and the force error by \f$ e_f = f_{des} - f \f$
	//!
	//! \param p Current task position vector or \f$ p \f$
	//! \param pdot Current task velocity vector or \f$ \dot{p} \f$
	//! \param pd Desired task position vector or \f$ p_{des} \f$
	//! \param pdotd Desired task velocity vector or \f$ \dot{p}_{des} \f$
	//! \param pddotd Desired task acceleration vector or \f$ \ddot{p}_{des} \f$
	//! \param f Current task force vector or \f$ f \f$
	//! \param fd Desired task force vector or \f$ f_{des} \f$
	//! \param fref Task reference force vector or \f$ f_{ref} \f$
	//! \param ev Task velocity error vector or \f$ e_v \f$
	//! \param ef Task force error vector or \f$ e_f \f$
	//  ----------------------------------------------------------
	inline void refComplianceForce(TaskVec const &, TaskVec const & v, 
		TaskVec const &, TaskVec const & vd, TaskVec const & vdotd, 
		TaskVec const & f, TaskVec const & fd, 
		TaskVec & fref, 
		TaskVec &, TaskVec &ev, 
		TaskVec &ef)
	{
		// FIXME 20160107
		ev = vd - v;
		ef = fd - f;				

		/// FIXME @20151103
		/// FIXME @20151103
		// Only valid for identity mass matrix
		_stiffness.diagonal() = _kp;
		_damping.diagonal() = _kv;
		/////////////////////////////////////////////////////////////////////////////

		// For force feedback
		/// FIXED @20160428 Can add external force compensation
		_ref = -f;
		// FIXME 20160107
		// For compliance control
		//fref = vdotd + _kv.cwiseProduct(ev) + _kp.cwiseProduct(_int_error) - _kf.cwiseProduct(ef);
		fref = vdotd + _kv.cwiseProduct(ev) + _kp.cwiseProduct(_int_error) - ef;
		_ref += fref;

		_int_error += _dT*ev;
		// For additional reference error feedback
		/// FIXED @20170118
		if (_passivity_based)
		{
			//TaskVec evref = ev + _kv.cwiseProduct(_int_error) + _kp.cwiseProduct(_int_int_error) - _kf.cwiseProduct(_int_ef);
			TaskVec evref = ev + _kv.cwiseProduct(_int_error) + _kp.cwiseProduct(_int_int_error) - _int_ef;
			_ref.noalias() += _invL2sqr.cwiseProduct(evref);

			_int_int_error += _dT*_int_error;
			_int_ef += _dT*ef;
		}
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity and acceleration by impedance
	//!
	//! \details
	//! They are computed by \f$ \dot{v}_{ref} = \dot{v}_{des} + k_v e_v + k_p \int e_v - k_f e_f\f$ and  
	//!  \f$ v_{ref} = v_{des} + k_v \int e_v + kp \int \int e_v dt dt - k_f \int e_f dt \f$, where 
	//! the position and velocity errors are defined by \f$ e_v = v_{des} - v \f$,
	//! and the force error by \f$ e_f = f_{des} - f \f$.
	//! 
	//! \note 
	//! The first argument for the current task position vector is neglected. 
	//!
	//! \param v Current task velocity vector or \f$ v \f$
	//! \param vd Desired task velocity vector or \f$ v_{des} \f$
	//! \param vdotd Desired task acceleration vector or \f$ \dot{v}_{des} \f$
	//! \param f Current task force vector or \f$ f \f$
	//! \param fd Desired task force vector or \f$ f_{des} \f$
	//! \param vref Task reference velocity vector or \f$ v_{ref} \f$
	//! \param vdotref Task reference acceleration vector or \f$ \dot{v}_{ref} \f$
	//  ----------------------------------------------------------
	inline void refImpedanceAcceleration(TaskVec const &x, TaskVec const & v, 
		TaskVec const &xd, TaskVec const & vd, TaskVec const & vdotd, 
		TaskVec const & f, TaskVec const & fd, 
		TaskVec & vref, TaskVec & vdotref)
	{
		TaskVec ex; // don't care
		TaskVec ev;
		TaskVec ef;

		refImpedanceAcceleration(x, v, xd, vd, vdotd, vref, vdotref, ex, ev, ef);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity and acceleration as well as task position and velocity error  by impedance
	//!
	//! \details
	//! They are computed by \f$ \dot{v}_{ref} = \dot{v}_{des} + k_v e_v + k_p \int e_v - k_f e_f\f$ and  
	//!  \f$ v_{ref} = v_{des} + k_v \int e_v + kp \int \int e_v dt dt - k_f \int e_f dt \f$, where 
	//! the position and velocity errors are defined by \f$ e_v = v_{des} - v \f$,
	//! and the force error by \f$ e_f = f_{des} - f \f$.
	//! 
	//! \note 
	//! The first argument for the current task position vector 
	//! and the eighth argument for the task position error are neglected.
	//!
	//! \param v Current task velocity vector or \f$ v \f$
	//! \param vd Desired task velocity vector or \f$ v_{des} \f$
	//! \param vdotd Desired task acceleration vector or \f$ \dot{v}_{des} \f$
	//! \param f Current task force vector or \f$ f \f$
	//! \param fd Desired task force vector or \f$ f_{des} \f$
	//! \param vref Task reference velocity vector or \f$ v_{ref} \f$
	//! \param vdotref Task reference acceleration vector or \f$ \dot{v}_{ref} \f$
	//! \param ev Task velocity error vector or \f$ e_v \f$
	//! \param ef Task force error vector or \f$ e_f \f$
	//  ----------------------------------------------------------
	inline void refImpedanceAcceleration(TaskVec const &, TaskVec const & v, 
		TaskVec const &, TaskVec const & vd, TaskVec const & vdotd, 
		TaskVec const & f, TaskVec const & fd, 
		TaskVec & vref, TaskVec & vdotref, 
		TaskVec & , TaskVec &ev, TaskVec &ef)
	{
		ev = vd - v;
		
		/// FIXME @20161017  (Tried but not work as intended)
		//ef = _impedanceDir.cwiseProduct(fd - f);
		ef = fd - f;

		//vref = vd + _kv.cwiseProduct(_int_error) + _kp.cwiseProduct(_int_int_error) - _impedanceDir.cwiseProduct(_kf.cwiseProduct(_int_ef));
		//vdotref = vdotd + _kv.cwiseProduct(ev) + _kp.cwiseProduct(_int_error) - _impedanceDir.cwiseProduct(_kf.cwiseProduct(ef));
		vref = vd + _kv.cwiseProduct(_int_error) + _kp.cwiseProduct(_int_int_error) - _kf.cwiseProduct(_int_ef);
		vdotref = vdotd + _kv.cwiseProduct(ev) + _kp.cwiseProduct(_int_error) - _kf.cwiseProduct(ef);

		// For force feedback
		//_ref = -_impedanceDir.cwiseProduct(f);
		_ref = -f;

		// For additional reference error feedback
		/// FIXED @20170118
		if (_passivity_based)
			_ref.noalias() += _invL2sqr.cwiseProduct(vref - v);

		_int_error += _dT*ev;
		_int_int_error += _dT*_int_error;
		_int_ef += _dT*ef;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity 
	//!
	//! \details
	//! Task reference velocity is computed simply by \f$ v_{ref} = v_{des} \f$.
	//! 
	//! \note 
	//! The first two arguments for the current task position and the desired task position vector are neglected.
	//! 
	//! \param vd
	//! Desired task velocity vector
	//! \param vref
	//! Task reference velocity vector
	//  ----------------------------------------------------------
	inline void refVelocity(TaskVec const &, 
		TaskVec const &, TaskVec const & vd,
		TaskVec & vref)
	{
		vref = vd; // + _kp.cwiseProduct(_int_error) + _ki.cwiseProduct(_int_int_error);

// 		_int_error += _dT*vd;
// 		_int_int_error += _dT*_int_error;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity with as well as task position error
	//!
	//! \details
	//! Task reference velocity is computed simply by \f$ v_{ref} = v_{des} \f$.
	//! 
	//! \note 
	//! The first two arguments for the current task position and the desired task position vector 
	//! as well as the last argument for task position error are neglected.
	//! 
	//! \param vd
	//! Desired task velocity vector
	//! \param vref
	//! Task reference velocity vector
	//  ----------------------------------------------------------
	inline void refVelocity(TaskVec const &, 
		TaskVec const &, TaskVec const & vd,
		TaskVec & vref, TaskVec &)
	{
		vref = vd;// + _kp.cwiseProduct(_int_error) + _ki.cwiseProduct(_int_int_error);

// 		_int_error += _dT*vd;
// 		_int_int_error += _dT*_int_error;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task errors
	//!
	//! \details
	//! The velocity error is defined by \f$ e_v = v_{des} - v \f$, 
	//! whereas the position error is not relevant within the context of velocity controller. 
	//! 
	//! \note 
	//! The first argument for the current task position vector, the third one for the desired task position vector, 
	//! and the fifth for task position error are neglected.
	//!
	//! \param v
	//! Current task velocity vector
	//! \param vd
	//! Desired task velocity vector
	//! \param ev
	//! Task velocity error vector
	//  ----------------------------------------------------------
	inline void error(TaskVec const &, TaskVec const & v, 
		TaskVec const &, TaskVec const & vd,
		TaskVec & ep, TaskVec & ev)
	{
		ep.setZero(); 
		ev = vd - v;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task position error
	//!
	//! \details
	//! The position error is not relevant within the context of velocity controller. 
	//! Hence, this function does nothing.
	//!
	//! \note 
	//! The first argument for the current task position vector, the third one for the desired task position vector, 
	//! and the fifth for task position error are neglected.
	//!
	//  ----------------------------------------------------------
	inline void error(TaskVec const &, TaskVec const &, TaskVec & e)
	{
		e.setZero();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! reset all integral errors
	//  ----------------------------------------------------------
	/// ADDED @20160511
	inline void resetIntError()
	{
		_int_int_error.setZero();
		_int_error.setZero();
		_int_ef.setZero();
	}

public:
 	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	using Controller<DIM, VelocityController<DIM> >::_kp;
	using Controller<DIM, VelocityController<DIM> >::_kv;
	using Controller<DIM, VelocityController<DIM> >::_ki;
	using Controller<DIM, VelocityController<DIM> >::_invL2sqr;
	using Controller<DIM, VelocityController<DIM> >::_kf;
	using Controller<DIM, VelocityController<DIM> >::_kfi;
	using Controller<DIM, VelocityController<DIM> >::_int_error;
	using Controller<DIM, VelocityController<DIM> >::_int_ef;
	using Controller<DIM, VelocityController<DIM> >::_ref;
	using Controller<DIM, VelocityController<DIM> >::_dT;
	using Controller<DIM, VelocityController<DIM> >::_passivity_based;
	
	/// FIXME @20151103
	using Controller<DIM, VelocityController<DIM> >::_stiffness;
	using Controller<DIM, VelocityController<DIM> >::_damping;
	using Controller<DIM, VelocityController<DIM> >::_mass;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Double integral of velocity error
	//  ----------------------------------------------------------
	TaskVec _int_int_error;

	/// ADDED @20161017
	using Controller<DIM, VelocityController<DIM> >::_impedanceDir;
};	// class VelocityController

} // namespace NRMKFoundation
