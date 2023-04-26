//! \file ProfileVelocityInterpolator.h
//!
//! \brief
//! Header file for the class ProfileVelocityInterpolator (Internal API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a class for profile velocity interpolator 
//! to be used for the interface of NRMKFoundation library. 
//! The algorithm is adapted from Sec.3.10.1 of [Trajectory Planning for Automatic Machines and Robots]. 
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
//! \date March 2016
//! 
//! \version 1.9.3
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013--2016 Neuromeka Co., Ltd.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

namespace NRMKFoundation
{

namespace internal
{
}

//  ---------------------- Doxygen info ----------------------
//! \class ProfileVelocityInterpolator
//!
//! \brief
//! This class implements a constant velocity trajectory with cycloidal blends algorithm 
//! to be used for the interface of NRMKFoundation library
//!
//! \details 
//! This class implements the profile velocity interpolator class 
//! for interpolating a velocity vector variable for a given maximum velocity. 
//  ----------------------------------------------------------
template<int DIM>
class ProfileVelocityInterpolator
{
public:
	typedef Eigen::Matrix<double, DIM, 1> VecType; //!< Typedef of task vector

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the initial condition of the interpolated trajectory
	//!
	//! \note
	//! This should be called before setProfileSpecs(). 
	//! 
	//! \param t0 Initial time
	//! \param p0 Initial position vector
	//! \param v0 Initial velocity vector
	//! \param a0 Initial acceleration vector
	//  ----------------------------------------------------------
	inline void setInitialTraj(double t0, VecType const & p0, VecType const & v0 = VecType::Zero(), VecType const & a0 = VecType::Zero())
	{
		_t0 = t0;
		_p0 = p0;
		_v0 = v0;
		_a0 = a0;
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the velocity profile parameters
	//!
	//! \note
	//! This should be called before traj(). 
	//! 
	//! \param tacc Time to reach the maximum velocity
	//! \param vmax Maximum velocity vector
	//! \param amax Maximum acceleration vector
	//  ----------------------------------------------------------
	inline void setProfileCond(double tacc, VecType const & vmax, VecType const & amax = VecType::Zero())
	{
		_tacc = tacc;
		_vmax = vmax;
		_amax = amax;

		_pacc = (_vmax + _v0)/2*(_tacc - _t0) + _p0;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the desired trajectory for the time instant
	//! 
	//! \note
	//! This should be called after setTargetTraj(). 
	//!
	//! \param t Desired time
	//! \param pd Desired position vector
	//! \param vd Desired velocity vector 
	//! \param ad Desired acceleration vector 
	//  ----------------------------------------------------------
	inline void traj(double t, VecType & pd, VecType & vd, VecType & ad)
	{
		if (t < _t0)
		{
			pd = _p0;
			vd = _v0;
			ad = _a0;

			return;
		}

		VecType Vk, Kk;
		double wk; 

		if (t < _tacc)
		{
			Vk = (_vmax - _v0)/2;
			Kk = (_vmax + _v0)/2;
			wk = M_PI/(_tacc - _t0);

			double h = t - _t0;

			pd = -(sin(wk*h)/wk)*Vk + h*Kk + _p0;
			vd = -cos(wk*h)*Vk + Kk;
			ad = (wk*sin(wk*h))*Vk;
		} 
		else // if (t > _tacc)
		{
			Vk.setZero(); 
			Kk = _vmax;
			wk = 0;

			pd = (t - _tacc)*Kk + _pacc;
			vd = Kk;
			ad.setZero();
		}		
	}

private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Initial time
	//  ----------------------------------------------------------
	double _t0;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Initial task position vector
	//  ----------------------------------------------------------
	VecType _p0;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Initial task velocity vector
	//  ----------------------------------------------------------
	VecType _v0;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Initial task acceleration vector
	//  ----------------------------------------------------------
	VecType _a0;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Time to reach max vel
	//  ----------------------------------------------------------
	double _tacc;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Position at tacc
	//  ----------------------------------------------------------
	VecType _pacc;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Max Velocity
	//  ----------------------------------------------------------
	VecType _vmax;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Max Acceleration
	//  ----------------------------------------------------------
	VecType _amax;
};

}