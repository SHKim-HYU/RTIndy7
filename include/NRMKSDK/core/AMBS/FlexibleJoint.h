//  ---------------------- Doxygen info ----------------------
//! \file FlexibleJoint.h
//!
//! \brief
//! Header file for the class FlecibleJoint (API of the NRMKFoundation Libraries)
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
//! \date August 2015
//! 
//! \version 1.9
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013-2015 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

//#include "Joint.h"
#include "../NRMKCommon.h"

#include "Integrator.h"

namespace NRMKFoundation
{

//  ---------------------- Doxygen info ----------------------
//! \class FlexinleJoint
//!
//! \brief
//! This implements a flexible joint class.
//! 
//  ----------------------------------------------------------
class FlexibleJoint
{
public:
	inline FlexibleJoint() 
		: _I(0), _D(0), _K(std::numeric_limits<double>::infinity())
		, _th(0), _thdot(0), _thddot(0)
		, _h(0)
		/// FIXME @20151022
		,  _thdot_stationary(1e-5)
		, _stiction(0)
		, _viscosity(0)
		, _mu(0)
	{
	}

	//! \return joint stiffness values
	inline double stiffness() const
	{
		return _K;
	}

	//! \return  joint damping values
	inline double damping() const
	{
		return _D;
	}

	//! \return  joint damping values
	inline double inerita() const
	{
		return _I;
	}

	//! \return joint position
	inline double pos() const
	{
		return _th;
	}

	//! \return joint velocity
	inline double vel() const
	{
		return _thdot;
	}

	//! \return joint acceleration
	inline double acc() const
	{
		return _thddot;
	}

	//! \brief sets the joint damping
	//! \param d joint damping value
	inline void setDamping(double d)
	{
		_D = d; 
	}

	//! \brief sets the joint stiffness
	//! \param k joint stiffness value
	inline void setStiffness(double k)
	{
		_K = k; 
	}

	//! \brief sets the joint inertia
	//! \param I joint inertia value
	inline void setInertia(double I)
	{
		_I = I;
	}

	//! \brief sets the joint position
	//! \param th joint position value
	inline void setPos(double th)
	{
		_th = th; 
	}

	//! \brief sets the joint velocity
	//! \param thdot joint velocity value
	inline void setVel(double thdot)
	{
		_thdot = thdot;
	}

	//! \brief sets the integration period
	//! \param h integration period
	inline void setPeriod(double h)
	{
		_h = h; 
	}

	//! \brief computes joint acceleration by implicit stiffness and damping
	//! \param q link position
	//! \param qdot link velocity
	//! \param qddot link acceleration
	//! \param u joint torque
	//!
	//! \note qddot should be computed appropriately 
	inline void computeAcc(double q, double qdot, double qddot, double u)
	{
		_thddot = (u - _K*(_th - q) - (_D + _h*_K)*(_thdot - qdot) + _h*_D*qddot)/(_I + _h*_D);
	}

	//! \brief computes joint acceleration by explicit stiffness and damping
	//! \param q link position
	//! \param qdot link velocity
	//! \param u joint torque
	inline void computeAcc(double q, double qdot, double u)
	{
		_thddot = (u - _K*(_th - q) - (_D + _h*_K)*(_thdot - qdot)) / _I;
	}

	//! \brief integrates joint acceleration and velocity
	//! \note the joint position and velocity are updated internally
	inline void integrate()
	{
		Integrate(_th, _thdot, _thddot, _h);
	}

	//! \return added inertia to link
	double addedInertia() const
	{
		return (1 - _h*_D/(_I + _h*_D))*_h*_D;
	}

	//! \return generated joint torque 
	double addedTorque(double q, double qdot, double u) const
	{
		return (1 - _h*_D/(_I + _h*_D))*(_K*(_th - q) + (_D + _h*_K)*(_thdot - qdot)) + _h*_D/(_I + _h*_D)*u;
	}

	//! \return torque sensor value 
	double measureTorque(double q) const
	{
		return _K*(_th - q);
	}

	/// FIXME @20151022
	
	//! \brief sets the joint stiction
	//! \param stiction joint stiction value
	//! \param qdot_stationary joint stationary velocity threshold
	inline void setStiction(double stiction, double qdot_stationary = 1e-5)
	{
		_stiction = stiction;
		_thdot_stationary = qdot_stationary; 
	}

	//! \brief sets the joint viscosity
	//! \param viscosity joint viscosity value
	inline void setViscosity(double viscosity)
	{
		_viscosity = viscosity;
	}

	//! \brief sets the joint viscosity
	//! \param mu joint Coulomb coefficient
	inline void setCoulomb(double mu)
	{
		_mu = mu;
	}

	//! \return generated friction torque
	//! \param q joint position
	//! \param qdot joint velocity
	//! \param u motor torque
	double frictionTorque(double q, double qdot, double u) const
	{
		double tau = _K*(_th - q) + _D*(_thdot - qdot);

		if (fabs(_thdot) <= _thdot_stationary)
		{
			double u_f_max = (_stiction + _mu*std::fabs(tau)*NRMKFoundation::signum(u));
					
			if (fabs(u) < fabs(u_f_max))
				return u;
			else
				return u_f_max;
		}
		else
		{
			return (_stiction + _mu*std::fabs(tau))*NRMKFoundation::signum(_thdot) + _viscosity*_thdot ;
		}
	}

protected:
	//! \brief Joint inertia
	double	_I;

	//! \brief Joint damping 
	double	_D;

	//! \brief Joint stiffness 
	double	_K;

	//! \brief Join position
	double	_th;

	//! \brief Join velocity
	double	_thdot;

	//! \brief Join acceleration
	double	_thddot;

	//! \brief Integration period
	double	_h;

	/// FIXME @20151022
	//! \brief threshold joint stationary velocity
	double _thdot_stationary;

	//! \brief joint stiction
	double _stiction;

	//! \brief joint viscosity
	double _viscosity;

	//! \brief joint coulomb coefficient
	double _mu;
};

} // namespace NRMKFoundation
