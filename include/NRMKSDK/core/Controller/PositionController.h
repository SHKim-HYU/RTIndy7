//  ---------------------- Doxygen info ----------------------
//! \file PositionController.h
//!
//! \brief
//! Header file for the class PositionController (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a position controller
//! to be used for the interface of NRMKFoundation library.
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
//! \date April 2016
//! 
//! \version 1.9.4
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note v1.9.4 (20160426): fixed refComplianceForce() for compliance control under Hinf optimal control
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
//! \class PositionController
//!
//! \brief
//! This implements a position controller class.
//!
//! \details 
//! When the task is described by vectors, it can be controlled by this control. 
//! In particular, task position, velocity are denoted by \f$ p \in \mathbb{R}^m \f$ and \f$ \dot{p} \in \mathbb{R}^m \f$.
//! The desired task position, velocity, and acceleration are denoted by \f$ p_{des} \f$, \f$ \dot{p}_{des} \f$, and \f$ \ddot{p}_{des} \f$,
//! and the reference velocity and acceleration by \f$ \dot{p}_{ref} \f$, and \f$ \ddot{p}_{ref} \f$, all vectors belonging to \f$ \mathbb{R}^m \f$. 
//!  
//! \tparam DIM Task dimension
//!
//! \sa Controller
//  ----------------------------------------------------------
template <int DIM>
class PositionController : public Controller<DIM, PositionController<DIM> >
{	
public:
	typedef typename Controller<DIM, PositionController<DIM> >::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename Controller<DIM, PositionController<DIM> >::TaskMat TaskMat; //!< Typedef of task matrix

public:
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
	//! They are computed by \f$ \ddot{p}_{ref} = \ddot{p}_{des} + k_v \dot{e} + k_p e \f$ and  
	//!  \f$ \dot{p}_{ref} = \dot{p}_{des} + k_ve + kp\int e dt \f$, where 
	//! the position and velocity errors are defined by \f$ e = p_{des} - p \f$ and \f$ \dot{e} = \dot{p}_{des} - \dot{p} \f$.
	//!
	//! \param p Current task position vector or \f$ p \f$
	//! \param pdot Current task velocity vector or \f$ \dot{p} \f$
	//! \param pd Desired task position vector or \f$ p_{des} \f$
	//! \param pdotd Desired task velocity vector or \f$ \dot{p}_{des} \f$
	//! \param pddotd Desired task acceleration vector or \f$ \ddot{p}_{des} \f$
	//! \param pdotref Task reference velocity vector or \f$ \dot{p}_{ref} \f$
	//! \param pddotref Task reference acceleration vector or \f$ \ddot{p}_{ref} \f$
	//  ----------------------------------------------------------
	inline void refAcceleration(TaskVec const & p, TaskVec const & pdot, 
						TaskVec const & pd, TaskVec const & pdotd, TaskVec const & pddotd, 
						TaskVec & pdotref, TaskVec & pddotref)
	{
		TaskVec e;
		TaskVec edot;

		refAcceleration(p, pdot, pd, pdotd, pddotd, pdotref, pddotref, e, edot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity and acceleration as well as task position and velocity error
	//!
	//! \details
	//! They are computed by \f$ \ddot{p}_{ref} = \ddot{p}_{des} + k_v \dot{e} + k_p e \f$ and  
	//!  \f$ \dot{p}_{ref} = \dot{p}_{des} + k_ve + kp\int e dt \f$, where 
	//! the position and velocity errors are defined by \f$ e = p_{des} - p \f$ and \f$ \dot{e} = \dot{p}_{des} - \dot{p} \f$.
	//!
	//! \param p Current task position vector or \f$ p \f$
	//! \param pdot Current task velocity vector or \f$ \dot{p} \f$
	//! \param pd Desired task position vector or \f$ p_{des} \f$
	//! \param pdotd Desired task velocity vector or \f$ \dot{p}_{des} \f$
	//! \param pddotd Desired task acceleration vector or \f$ \ddot{p}_{des} \f$
	//! \param pdotref Task reference velocity vector or \f$ \dot{p}_{ref} \f$
	//! \param pddotref Task reference acceleration vector or \f$ \ddot{p}_{ref} \f$
	//! \param e Task position error vector or \f$ e \f$
	//! \param edot Task velocity error vector or \f$ \dot{e} \f$
	//  ----------------------------------------------------------
	inline void refAcceleration(TaskVec const & p, TaskVec const & pdot, 
		TaskVec const & pd, TaskVec const & pdotd, TaskVec const & pddotd, 
		TaskVec & pdotref, TaskVec & pddotref, 
		TaskVec & e, TaskVec & edot)
	{
		e = pd - p;
		edot = pdotd - pdot;

		pdotref = pdotd + _kv.cwiseProduct(e) + _kp.cwiseProduct(_int_error);
		pddotref = pddotd + _kv.cwiseProduct(edot) + _kp.cwiseProduct(e);

		// For reference error feedback
		/// FIXED @20170118
		if (_passivity_based)
			_ref.noalias() = _invL2sqr.cwiseProduct(pdotref - pdot);

		_int_error += _dT*e;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity and acceleration by impedance
	//!
	//! \details
	//! They are computed by \f$ \ddot{p}_{ref} = \ddot{p}_{des} + k_v \dot{e} + k_p e - k_f e_f\f$ and  
	//!  \f$ \dot{p}_{ref} = \dot{p}_{des} + k_ve + kp\int e dt - k_f \int e_f dt \f$, where 
	//! the position and velocity errors are defined by \f$ e = p_{des} - p \f$, \f$ \dot{e} = \dot{p}_{des} - \dot{p} \f$,
	//! and the force error by \f$ e_f = f_{des} - f \f$.
	//!
	//! \param p Current task position vector or \f$ p \f$
	//! \param pdot Current task velocity vector or \f$ \dot{p} \f$
	//! \param pd Desired task position vector or \f$ p_{des} \f$
	//! \param pdotd Desired task velocity vector or \f$ \dot{p}_{des} \f$
	//! \param pddotd Desired task acceleration vector or \f$ \ddot{p}_{des} \f$
	//! \param f Current task force vector or \f$ f \f$
	//! \param fd Desired task force vector or \f$ f_{des} \f$
	//! \param pdotref Task reference velocity vector or \f$ \dot{p}_{ref} \f$
	//! \param pddotref Task reference acceleration vector or \f$ \ddot{p}_{ref} \f$
	//  ----------------------------------------------------------
	inline void refImpedanceAcceleration(TaskVec const & p, TaskVec const & pdot, 
		TaskVec const & pd, TaskVec const & pdotd, TaskVec const & pddotd, 
		TaskVec const & f, TaskVec const & fd, 
		TaskVec & pdotref, TaskVec & pddotref)
	{
		TaskVec e;
		TaskVec edot;
		TaskVec ef;

		refImpedanceAcceleration(p, pdot, pd, pdotd, pddotd, f, fd, pdotref, pddotref, e, edot, ef);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity and acceleration  as well as task position and velocity error by impedance
	//!
	//! \details
	//! They are computed by \f$ \ddot{p}_{ref} = \ddot{p}_{des} + k_v \dot{e} + k_p e - k_f e_f\f$ and  
	//!  \f$ \dot{p}_{ref} = \dot{p}_{des} + k_ve + kp\int e dt - k_f \int e_f dt \f$, where 
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
	//! \param pdotref Task reference velocity vector or \f$ \dot{p}_{ref} \f$
	//! \param pddotref Task reference acceleration vector or \f$ \ddot{p}_{ref} \f$
	//! \param e Task position error vector or \f$ e \f$
	//! \param edot Task velocity error vector or \f$ \dot{e} \f$
	//! \param ef Task force error vector or \f$ e_f \f$
	//  ----------------------------------------------------------
	inline void refImpedanceAcceleration(TaskVec const & p, TaskVec const & pdot, 
		TaskVec const & pd, TaskVec const & pdotd, TaskVec const & pddotd, 
		TaskVec const & f, TaskVec const & fd, 
		TaskVec & pdotref, TaskVec & pddotref, 
		TaskVec & e, TaskVec & edot,
		TaskVec & ef)
	{
		e = pd - p;
		edot = pdotd - pdot;

		/// FIXME @20161017 (Tried but not work as intended)
		//ef = _impedanceDir.cwiseProduct(fd - f);
		ef = fd - f;

		//pdotref = pdotd + _kv.cwiseProduct(e) + _kp.cwiseProduct(_int_error) - _impedanceDir.cwiseProduct(_kf.cwiseProduct(_int_ef));
		//pddotref = pddotd + _kv.cwiseProduct(edot) + _kp.cwiseProduct(e) - _impedanceDir.cwiseProduct(_kf.cwiseProduct(ef));

		pdotref = pdotd + _kv.cwiseProduct(e) + _kp.cwiseProduct(_int_error) - _kf.cwiseProduct(_int_ef);
		pddotref = pddotd + _kv.cwiseProduct(edot) + _kp.cwiseProduct(e) - _kf.cwiseProduct(ef);

		// For force feedback
		/// FIXME @20161017
		//_ref = -_impedanceDir.cwiseProduct(f);
		_ref = -f;

		// For additional reference error feedback
		/// FIXED @20170118
		if (_passivity_based)
			_ref.noalias() += _invL2sqr.cwiseProduct(pdotref - pdot);
		
		_int_error += _dT*e;
		_int_ef += _dT*ef;
	}

	/// FIXME @20151015
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference force by compliance
	//!
	//! \details
	//! They are computed by \f$ f_{ref} = \ddot{p}_{des} + k_v \dot{e} + k_p e - k_f e_f\f$, where 
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
	//  ----------------------------------------------------------
	inline void refComplianceForce(TaskVec const & p, TaskVec const & pdot, 
		TaskVec const & pd, TaskVec const & pdotd, TaskVec const & pddotd, 
		TaskVec const & f, TaskVec const & fd, 
		TaskVec & fref)
	{
		TaskVec e;
		TaskVec edot;
		TaskVec ef;

		refComplianceForce(p, pdot, pd, pdotd, pddotd, f, fd, fref, e, edot, ef);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference force by compliance
	//!
	//! \details
	//! They are computed by \f$ f_{ref} = \ddot{p}_{des} + k_v \dot{e} + k_p e - k_f e_f\f$, where 
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
	//! \param e Task position error vector or \f$ e \f$
	//! \param edot Task velocity error vector or \f$ \dot{e} \f$
	//! \param ef Task force error vector or \f$ e_f \f$
	//  ----------------------------------------------------------
	inline void refComplianceForce(TaskVec const & p, TaskVec const & pdot, 
		TaskVec const & pd, TaskVec const & pdotd, TaskVec const & pddotd, 
		TaskVec const & f, TaskVec const & fd, 
		TaskVec & fref, 
		TaskVec & e, TaskVec & edot,
		TaskVec & ef)
	{
		e = pd - p;
		edot = pdotd - pdot;
		ef = fd - f;

		/// FIXME @20151103
		// Only valid for identity mass matrix
		_stiffness.diagonal() = _kp;
		_damping.diagonal() = _kv;
		/////////////////////////////////////////////////////////////////////////////

		// For force feedback
		/// FIXED @20160428 Can add external force compensation
		_ref = -f;

		// For compliance control
		//fref = pddotd + _kv.cwiseProduct(edot) + _kp.cwiseProduct(e) - _kf.cwiseProduct(ef);
		fref = pddotd + _kv.cwiseProduct(edot) + _kp.cwiseProduct(e) - ef;
		_ref += fref;

		// For additional reference error feedback
		/// FIXED @20170118
		if (_passivity_based)
		{
			//TaskVec edotref = edot + _kv.cwiseProduct(e) + _kp.cwiseProduct(_int_error) - _kf.cwiseProduct(_int_ef);
			TaskVec edotref = edot + _kv.cwiseProduct(e) + _kp.cwiseProduct(_int_error) - _int_ef;
			_ref.noalias() += _invL2sqr.cwiseProduct(edotref);

			_int_error += _dT*e;
			_int_ef += _dT*ef;
		}
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity
	//!
	//! \details
	//! It is computed by \f$ \dot{p}_{ref} = \dot{p}_{des} + k_v*e + kp*\int e dt \f$, where 
	//! the position error is defined by \f$ e = p_{des} - p \f$.
	//!
	//! \param p Current task position vector or \f$ p \f$
	//! \param pd Desired task position vector or \f$ p_{des} \f$
	//! \param pdotd Desired task velocity vector or \f$ \dot{p}_{des} \f$
	//! \param pdotref Task reference velocity vector or \f$ \dot{p}_{ref} \f$
	//  ----------------------------------------------------------
	inline void refVelocity(TaskVec const & p, 
		TaskVec const & pd, TaskVec const & pdotd,
		TaskVec & pdotref)
	{
		TaskVec e;

		refVelocity(p, pd, pdotd, pdotref, e);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task reference velocity as well as task position error
	//!
	//! \details
	//! It is computed by \f$ \dot{p}_{ref} = \dot{p}_{des} + k_v*e + kp*\int e dt \f$, where 
	//! the position error is defined by \f$ e = p_{des} - p \f$.
	//!
	//! \param p Current task position vector or \f$ p \f$
	//! \param pd Desired task position vector or \f$ p_{des} \f$
	//! \param pdotd Desired task velocity vector or \f$ \dot{p}_{des} \f$
	//! \param pdotref Task reference velocity vector or \f$ \dot{p}_{ref} \f$
	//! \param e Task position error vector or \f$ e \f$
	//  ----------------------------------------------------------
	inline void refVelocity(TaskVec const & p, 
		TaskVec const & pd, TaskVec const & pdotd,
		TaskVec & pdotref, 
		TaskVec & e)
	{
		e = pd - p;

		pdotref = pdotd + _kv.cwiseProduct(e) + _kp.cwiseProduct(_int_error);

		_int_error += _dT*e;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task errors
	//!
	//! \details
	//! The position and velocity errors are defined by \f$ e = p_{des} - p \f$ 
	//! and \f$ \dot{e} = \dot{p}_{des} - \dot{p} \f$.
	//! 
	//! \param p Current task position vector or \f$ p \f$
	//! \param pdot Current task velocity vector or \f$ \dot{p} \f$
	//! \param pd Desired task position vector or \f$ p_{des} \f$
	//! \param pdotd Desired task velocity vector or \f$ \dot{p}_{des} \f$
	//! \param e Task position error vector or \f$ e \f$
	//! \param edot Task velocity error vector or \f$ \dot{e} \f$
	//  ----------------------------------------------------------
	inline void error(TaskVec const & p, TaskVec const & pdot, 
		TaskVec const & pd, TaskVec const & pdotd,
		TaskVec & e, TaskVec & edot)
	{
		e = pd - p;
		edot = pdotd - pdot;
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the task position error
	//!
	//! \details
	//! The position error is defined by \f$ e = p_{des} - p \f$.
	//! 
	//! \param p Current task position vector or \f$ p \f$
	//! \param pd Desired task position vector or \f$ p_{des} \f$
	//! \param e Task position error vector or \f$ e \f$
	//  ----------------------------------------------------------
	inline void error(TaskVec const & p, TaskVec const & pd, TaskVec & e)
	{
		e = pd - p;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! reset all integral errors
	//  ----------------------------------------------------------
	/// ADDED @20160511
	inline void resetIntError()
	{
		_int_error.setZero();
		_int_ef.setZero();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! reset a integral error of selected joint
	//  ----------------------------------------------------------
	/// ADDED @20180423
	inline void resetIntError(int idx)
	{
		_int_error[idx] = 0;
		_int_ef[idx] = 0;
	}

private:
	using Controller<DIM, PositionController<DIM> >::_kp;
	using Controller<DIM, PositionController<DIM> >::_kv;
	using Controller<DIM, PositionController<DIM> >::_ki;
	using Controller<DIM, PositionController<DIM> >::_invL2sqr;
	using Controller<DIM, PositionController<DIM> >::_kf;
	using Controller<DIM, PositionController<DIM> >::_kfi;
	using Controller<DIM, PositionController<DIM> >::_dT;
	using Controller<DIM, PositionController<DIM> >::_int_error;
	using Controller<DIM, PositionController<DIM> >::_int_ef;
	using Controller<DIM, PositionController<DIM> >::_ref;	
	using Controller<DIM, PositionController<DIM> >::_passivity_based;	

	/// FIXME @20151103
	using Controller<DIM, PositionController<DIM> >::_stiffness;
	using Controller<DIM, PositionController<DIM> >::_damping;
	using Controller<DIM, PositionController<DIM> >::_mass;

	/// ADDED @20161017
	using Controller<DIM, PositionController<DIM> >::_impedanceDir;
};	// class PositionController

} // namespace NRMKFoundation
