//  ---------------------- Doxygen info ----------------------
//! \file PolynomialAlgorithm.h
//!
//! \brief
//! Header file for the class _PolynomialInterpolator (Internal API of the NRMKFoundation Libraries)
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
//! \date April 2013
//! 
//! \version 0.1
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013 Neuromeka Co., Ltd.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#include <list>
#include <Eigen/Eigen>

#include "InterpolatorConstants.h"

namespace NRMKFoundation
{
namespace internal
{
//  ---------------------- Doxygen info ----------------------
//! \class _PolynomialInterpolator
//!
//! \brief
//! This implements a base polynomial interpolator class
//! to be used for the interface of NRMKFoundation library
//!
//! \details 
//! This class implements a base class for interpolating a scalar variable
//! given initial and final conditions with optional boundary conditions, such as 
//! maximum velocity, acceleration, and jerk values. The specific behavior of the interpolator 
//! is defined by deriving this class (by static polymorphism by the CRTP) by 
//! implementing setTargetTraj(), setBoundaryCond(), and traj(). 
//!  
//! \tparam _TYPE is the interpolator type among enum PolynomialInterpolatorType.
//! \tparam InterpolatorType is the derived class of this class.
//!
//! \sa _ZeroOrderPolynomialInterpolator
//! \sa _LinearPolynomialInterpolator
//! \sa _ParabolicPolynomialInterpolator
//! \sa _CubicPolynomialInterpolator
//! \sa _QuinticPolynomialInterpolator
//! \sa _SepticPolynomialInterpolator
//! \sa _TrapezoidalInterpolator
//  ----------------------------------------------------------
 template<PolynomialInterpolatorMode _TYPE, typename InterpolatorType>
class _PolynomialInterpolator
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \enum
	//! 
	//! \brief Provides polynomial interpolator type
	//  ----------------------------------------------------------
	enum 
	{	
		//! \details The interpolator type among enum PolynomialInterpolatorType
		TYPE = _TYPE,
	};

	//! \returns 
	//	a reference to the derived object 
	inline InterpolatorType& derived() { return *static_cast<InterpolatorType*>(this); }
	//! \returns 
	//! a const reference to the derived object 
	inline const InterpolatorType& derived() const { return *static_cast<const InterpolatorType*>(this); }

public:
	_PolynomialInterpolator() : _duration(0), _delT(0)
		//, _p0(0), _v0(0), _a0(0), _j0(0), _pf(0), _vf(0), _af(0), _jf(0)
		, _vmax(0), _vmin(0), _amax(0), _amin(0), _jmax(0), _jmin(0)
	{			
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the period for cyclic update of trajectory
	//!
	//! \param delT period (in sec) (default value = 0)
	//  ----------------------------------------------------------
	inline void setPeriod(double delT)
	{
		_delT = delT;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the initial condition of the interpolated trajectory
	//!
	//! \details
	//! This should be called first. 
	//! 
	//! \param t0 Initial time
	//! \param p0 Initial position
	//! \param v0 Initial velocity (default value = 0)
	//! \param a0 Initial acceleration (default value = 0)
	//! \param j0 Initial jerk (default value = 0)
	//  ----------------------------------------------------------
	inline void setInitialTraj(double t0, double p0, double v0 = 0, double a0 = 0, double j0 = 0)
	{
		derived().setInitialTraj(t0, p0, v0, a0, j0);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the final condition of the interpolated trajectory
	//!
	//! \details
	//! This should be called after setInitialTraj() before calling traj(). 
	//! 
	//! \param tf Final time
	//! \param pf Final position
	//! \param vf Final velocity (default value = 0)
	//! \param af Final acceleration (default value = 0)
	//! \param jf Final jerk (default value = 0)
	//  ----------------------------------------------------------
	inline void setTargetTraj(double tf, double pf, double vf = 0, double af = 0, double jf = 0)
	{
		derived().setTargetTraj(tf, pf, vf, af, jf);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the boundary condition of the interpolated trajectory
	//!
	//! \details
	//! Only symmetric bounds are allowed. That is, minimum velocity is the 
	//! negative of the maximum velocity.
	//! This may not be called but, if it is called, 
	//! it should be called after setTargetTraj() before calling traj(). 
	//! 
	//! \param vmax Maximal velocity 
	//! \param amax Maximal acceleration
	//! \param jmax Maximal jerk (default value = 0)
	//  ----------------------------------------------------------
	inline void setBoundaryCond(double vmax, double amax, double jmax = 0)
	{
		derived().setBoundaryCond(vmax, amax, jmax);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the duration of the interpolated trajectory
	//! 
	//! \param duration duration
	//  ----------------------------------------------------------
	inline void setDuration(double duration)
	{
		derived().setDuration(duration);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the desired trajectory at a given time. 
	//! 
	//! \param t the time
	//! \param pd desired position
	//! \param vd desired velocity 
	//! \param ad desired acceleration
	//  ----------------------------------------------------------
	inline void traj(double t, double & pd, double & vd, double & ad)
	{
		derived().traj(t, pd, vd, ad);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! This returns the final trajectory time.
	//  ----------------------------------------------------------
	inline double tf() const { return _tf; }

	//  ---------------------- Doxygen info ----------------------
	//! \brief This returns the trajectory duration.
	//  ----------------------------------------------------------
	inline double duration() const { return _duration; }

	//! \brief This returns the trajectory sampling period
	inline double period() const { return _delT; }

	//! \brief This returns the trajectory finished
	inline bool isArrived() const { return _isArrived; }

protected:
	//  ---------------------- Doxygen info ----------------------
	//! \var double _t0
	//! 
	//! \brief initial time
	//  ----------------------------------------------------------
	double _t0;

	//  ---------------------- Doxygen info ----------------------
	//! \var double _tf
	//! 
	//! \brief final time
	//  ----------------------------------------------------------
	double _tf;

	//  ---------------------- Doxygen info ----------------------
	//! \var double _duration
	//! 
	//! \brief trajectory duration
	//  ----------------------------------------------------------
	double _duration;

	//  ---------------------- Doxygen info ----------------------
	//! \var double _delT
	//! 
	//! \brief update period
	//  ----------------------------------------------------------
	double _delT;

	//  ---------------------- Doxygen info ----------------------
	//! \var double _p0
	//! 
	//! \brief initial position
	//  ----------------------------------------------------------
	double _p0;

	//  ---------------------- Doxygen info ----------------------
	//! \var double _v0
	//! 
	//! \brief initial velocity
	//  ----------------------------------------------------------
	double _v0;

	//  ---------------------- Doxygen info ----------------------
	//! \var double _a0
	//! 
	//! \brief initial acceleration
	//  ----------------------------------------------------------
	double _a0;

	//  ---------------------- Doxygen info ----------------------
	//! \var double _j0
	//! 
	//! \brief initial jerk
	//  ----------------------------------------------------------
	double _j0;

	//  ---------------------- Doxygen info ----------------------
	//! \var double _pf
	//! 
	//! \brief final position
	//  ----------------------------------------------------------
	double _pf;

	//  ---------------------- Doxygen info ----------------------
	//! \var double _vf
	//! 
	//! \brief final velocity
	//  ----------------------------------------------------------
	double _vf;

	//  ---------------------- Doxygen info ----------------------
	//! \var double _af
	//! 
	//! \brief final acceleration
	//  ----------------------------------------------------------
	double _af;

	//  ---------------------- Doxygen info ----------------------
	//! \var double _jf
	//! 
	//! \brief final jerk
	//  ----------------------------------------------------------
	double _jf;

	
	/// ADDED @20160512
	//! \brief maximum veloicty
	double _vmax;
	
	//! \brief minimum veloicty
	double _vmin;
	
	//! \brief maximum acceleration
	double _amax;

	//! \brief minimum acceleration
	double _amin;

	//! \brief maximum jerk
	double _jmax;	

	//! \brief minimum jerk
	double _jmin;

	//! \brief isArrived
	bool _isArrived;
};

//  ---------------------- Doxygen info ----------------------
//! \class _ZeroOrderPolynomialInterpolator
//!
//! \brief
//! This implements a linear interpolator class
//! to be used for the interface of NRMKFoundation library
//!
//! \details 
//! This class implements the linear interpolator class for interpolating a scalar variable
//! for a given pair of initial and final position. No boundary conditions as well as
//! initial and final velocities and accelerations are not taken into account.
//!
//! \sa _PolynomialInterpolator
//  ----------------------------------------------------------
class _ZeroOrderPolynomialInterpolator : public _PolynomialInterpolator<POLYNOMIAL_INTERPOLATOR_NONE, _ZeroOrderPolynomialInterpolator>
{
public:
	inline void setInitialTraj(double t0, double = 0, double = 0, double = 0, double = 0)
	{
		_t0 = t0;
	}

	//! \details This calculates the coefficients of the first-order polynomial.
	inline void setTargetTraj(double tf, double pf, double = 0, double = 0, double = 0)
	{
		_tf = tf;
		_duration = _tf - _t0;

		_pf = pf;		
		_pd = pf; 
	}

	//! \details Boundary conditions are not respected.
	inline void setBoundaryCond(double vmax, double amax, double = 0)
	{
	}

	//! \details sets the duration
	inline void setDuration(double duration)
	{
	}

	//! \details This calculates the desired trajectory at a given time.
	inline void traj(double t, double & pd, double & vd, double & ad)
	{
		pd = _pd;
		vd = 0;
		ad = 0;
	}

private:
	//  ---------------------- Doxygen info ----------------------
	//! \var double _pd;
	//! 
	//! \brief set point
	//  ----------------------------------------------------------
	double _pd;
};

//  ---------------------- Doxygen info ----------------------
//! \class _LinearPolynomialInterpolator
//!
//! \brief
//! This implements a linear interpolator class
//! to be used for the interface of NRMKFoundation library
//!
//! \details 
//! This class implements the linear interpolator class for interpolating a scalar variable
//! for a given pair of initial and final position. No boundary conditions as well as
//! initial and final velocities and accelerations are not taken into account.
//!
//! \sa _PolynomialInterpolator
//  ----------------------------------------------------------
class _LinearPolynomialInterpolator : public _PolynomialInterpolator<POLYNOMIAL_INTERPOLATOR_LINEAR, _LinearPolynomialInterpolator>
{
public:
	inline void setInitialTraj(double t0, double p0, double = 0, double = 0, double = 0)
	{
		_t0 = t0;
		_p0 = p0;
	}

	//! \details This calculates the coefficients of the first-order polynomial.
	inline void setTargetTraj(double tf, double pf, double = 0, double = 0, double = 0)
	{
		_tf = tf;
		_duration = _tf - _t0;

		_pf = pf;

		_a[0] = _p0;
		_a[1] = (_pf - _p0)/_duration; 
	}

	//! \details Boundary conditions are not respected.
	inline void setBoundaryCond(double vmax, double amax, double = 0)
	{
	}

	//! \details sets the duration
	inline void setDuration(double duration)
	{
	}

	//! \details This calculates the desired trajectory at a given time.
	inline void traj(double t, double & pd, double & vd, double & ad)
	{
		pd = _a[0] + _a[1]*(t - _t0);
		vd = _a[1];
		ad = 0;
	}

private:
	//  ---------------------- Doxygen info ----------------------
	//! \var double _a[2];
	//! 
	//! \brief polynomial coefficients
	//  ----------------------------------------------------------
	double _a[2];
	//double _a[PolynomialInterpolatorCoeffSize[POLYNOMIAL_INTERPOLATOR_LINEAR]];
};

//  ---------------------- Doxygen info ----------------------
//! \class _ParabolicPolynomialInterpolator
//!
//! \brief
//! This implements a two-segment parabolic interpolator class
//! to be used for the interface of NRMKFoundation library
//!
//! \details 
//! This class implements the two-segment parabolic interpolator class 
//! for interpolating a scalar variable for a given pair of initial 
//! and final position and velocity. No boundary conditions as well as
//! initial and final accelerations are not taken into account.
//!
//! \sa _PolynomialInterpolator
//  ----------------------------------------------------------
class _ParabolicPolynomialInterpolator : public _PolynomialInterpolator<POLYNOMIAL_INTERPOLATOR_PARABOLIC, _ParabolicPolynomialInterpolator>
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \fn _ParabolicPolynomialInterpolator(double tm_portion)
	//! 
	//! \brief
	//! Constructor of the class with the portion to fix the inflection
	//!
	//! \details
	//! When tm_portion is given, the first tm_potion * 100 (%) of the 
	//! total duration will be the acceleration (or deceleration) phase
	//! being followed by the deceleration (or acceleration) phase.
	//!
	//! \param tm_portion the duration ration of the initial segment
	//!	 It should be between 0 and 1.
	//  ----------------------------------------------------------
	_ParabolicPolynomialInterpolator(double tm_portion = 0.5)
		: _tm_portion(tm_portion)
	{
	}


	inline void setInitialTraj(double t0, double p0, double v0 = 0, double = 0, double = 0)
	{
		_t0 = t0;
		_p0 = p0;
		_v0 = v0;
	}

	//! \details This calculates the coefficients of the two quadratic-order polynomial.
	//! The first three elements defines the first quadratic polynomial, 
	//! while the last three the second polynomial.
	inline void setTargetTraj(double tf, double pf, double vf = 0, double = 0, double = 0)
	{
		_tf = tf;
		_duration = _tf - _t0;		

		_pf = pf;
		_vf = vf;

		double h = _pf - _p0;
		
		
#if 0	// if v0 = -_vf
		double T2 = _duration*_duration;
		_tm = (_t0 + tf)/2;

		_a[0] = _p0;
		_a[1] = _v0;		
		_a[2] = (4*h - _duration*(3*_v0 + _vf))/(2*T2);
				
		_a[3] = (4*(_p0 + _pf) + _duration*(_v0 - _vf))/8;
		_a[4] = (4*h - _duration*(_v0 + _vf))/(2*_duration);
		_a[5] = (-4*h + _duration*(_v0 + 3*_vf))/(2*T2);
#endif

		_tm = _t0 + _tm_portion*h;
		double Ta = _tm - _t0;
		double Td = tf - _tm;

		_a[0] = _p0;
		_a[1] = _v0;
		_a[2] = (2*h - _v0*(_duration + Ta) - _vf*Td)/(2*_duration*Ta);

		_a[3] = (2*_pf*Ta + Td*(2*_p0 + Ta*(_v0 - _vf)))/(2*_duration);
		_a[4] = (2*h - _v0*Ta - _vf*Td)/_duration;
		_a[5] = (2*h - _v0*Ta - _vf*(_duration + Ta))/(-2*_duration*Td);
	}

	//! \details Boundary conditions are not respected.
	inline void setBoundaryCond(double vmax = 0, double amax = 0, double = 0)
	{
	}

	//! \details sets the duration
	inline void setDuration(double duration)
	{
		_duration = duration;
	}

	//! \details This calculates the desired trajectory at a given time.
	inline void traj(double t, double & pd, double & vd, double & ad)
	{
		if (t < _tm)
		{
			double T = t - _t0;

			pd = _a[0] + _a[1]*T + _a[2]*T*T; 
			vd = _a[1] + 2*_a[2]*T;
			ad = 2*_a[2];
		}
		else
		{
			double T = t - _tm;

			pd = _a[3] + _a[4]*T + _a[5]*T*T; 
			vd = _a[4] + 2*_a[5]*T;
			ad = 2*_a[5];
		}
	}

private:
	//  ---------------------- Doxygen info ----------------------
	//! \var double _a[6];
	//! 
	//! \brief polynomial coefficients
	//  ----------------------------------------------------------
	double _a[6];

	//  ---------------------- Doxygen info ----------------------
	//! \var double _tm
	//! 
	//! \brief the time where inflection occurs
	//  ----------------------------------------------------------
	double _tm;
	
	//  ---------------------- Doxygen info ----------------------
	//! \var double _tm_portion
	//! 
	//! \brief the duration ration of the first phase
	//  ----------------------------------------------------------
	double _tm_portion;
};

//  ---------------------- Doxygen info ----------------------
//! \class _CubicPolynomialInterpolator
//!
//! \brief
//! This implements a cubic interpolator class
//! to be used for the interface of NRMKFoundation library
//!
//! \details 
//! This class implements the cubic interpolator class 
//! for interpolating a scalar variable for a given pair of initial 
//! and final position and velocity. No boundary conditions as well as
//! initial and final accelerations are not taken into account.
//!
//! \sa _PolynomialInterpolator
//  ----------------------------------------------------------
class _CubicPolynomialInterpolator : public _PolynomialInterpolator<POLYNOMIAL_INTERPOLATOR_CUBIC, _CubicPolynomialInterpolator>
{
public:
	inline void setInitialTraj(double t0, double p0, double v0 = 0, double = 0, double = 0)
	{
		_t0 = t0;
		_p0 = p0;
		_v0 = v0;
	}

	//! \details This calculates the coefficients of the third-order polynomial.
	inline void setTargetTraj(double tf, double pf, double vf = 0, double = 0, double = 0)
	{		
		_tf = tf;
		_duration = _tf - _t0;		

		_pf = pf;
		_vf = vf;

		_a[0] = _p0;
		_a[1] = _v0;

		double h = _pf - _p0;
		double T2 = _duration*_duration;

		_a[2] = (3*h - (2*_v0 + _vf)*_duration)/T2;
		_a[3] = (-2*h + (_v0 + _vf)*_duration)/(T2*_duration);
	}

	//! \details Boundary conditions are not respected.
	inline void setBoundaryCond(double vmax = 0, double amax = 0, double = 0)
	{
	}

	//! \details sets the duration
	inline void setDuration(double duration)
	{
	}

	//! \details This calculates the desired trajectory at a given time.
	inline void traj(double t, double & pd, double & vd, double & ad)
	{
		double T = t - _t0;
		double T2 = T*T;

		pd = _a[0] + _a[1]*T + _a[2]*T2 + _a[3]*T2*T;
		vd = _a[1] + 2*_a[2]*T + 3*_a[3]*T2;
		ad = 2*_a[2] + 6*_a[3]*T;
	}

private:
	//  ---------------------- Doxygen info ----------------------
	//! \var double _a[4];
	//! 
	//! \brief polynomial coefficients
	//  ----------------------------------------------------------
	double _a[4];
};

//  ---------------------- Doxygen info ----------------------
//! \class _QuinticPolynomialInterpolator
//!
//! \brief
//! This implements a quintic interpolator class
//! to be used for the interface of NRMKFoundation library
//!
//! \details 
//! This class implements the quintic interpolator class 
//! for interpolating a scalar variable for a given pair of initial 
//! and final position, velocity, and acceleration. No boundary conditions 
//! are not taken into account.
//!
//! \sa _PolynomialInterpolator
//  ----------------------------------------------------------
class _QuinticPolynomialInterpolator : public _PolynomialInterpolator<POLYNOMIAL_INTERPOLATOR_QUINTIC, _QuinticPolynomialInterpolator>
{
public:
	inline void setInitialTraj(double t0, double p0, double v0 = 0, double a0 = 0, double = 0)
	{
		_isArrived = false;
		_t0 = t0;
		_p0 = p0;
		_v0 = v0;
		_a0 = a0;
	}

	//! \details This calculates the coefficients of the fifth-order polynomial.
	inline void setTargetTraj(double tf, double pf, double vf = 0, double af = 0, double = 0)
	{		
		_tf = tf;
		_duration = _tf - _t0;		

		_pf = pf;
		_vf = vf;
		_af = af;

		_a[0] = _p0;
		_a[1] = _v0;
		_a[2] = _a0/2;

		double h = _pf - _p0;
		double T2 = _duration*_duration;
		double T3 = T2*_duration;
		double T4 = T3*_duration;

		if ( _duration < 1.0e-6 )
		{
			_a[3] = 0.0;
			_a[4] = 0.0;
			_a[5] = 0.0;
		}
		else
		{
			_a[3] = (20*h -(8*_vf + 12*_v0)*_duration - (3*_a0 - _af)*T2)/(2*T3);
			_a[4] = (-30*h +(14*_vf + 16*_v0)*_duration + (3*_a0 - 2*_af)*T2)/(2*T4);
			_a[5] = (12*h - (6*_vf + 6*_v0)*_duration + (_af - _a0)*T2)/(2*T4*_duration);
		}
	}

	//! \details Boundary conditions are not respected.
	inline void setBoundaryCond(double vmax = 0, double amax = 0, double = 0)
	{
	}

	//! \details sets the duration
	inline void setDuration(double duration)
	{
	}

	//! \details This calculates the desired trajectory at a given time.
	inline void traj(double t, double & pd, double & vd, double & ad)
	{
		double T = t - _t0;
		if (T>_duration)
		{
			pd = _pf;
			vd = _vf;
			ad = _af;
			_isArrived = true;
		}
		else
		{
			double T2 = T*T;
			double T3 = T2*T;
			double T4 = T3*T;

			pd = _a[0] + _a[1]*T + _a[2]*T2 + _a[3]*T3 + _a[4]*T4 + _a[5]*T4*T;
			vd = _a[1] + 2*_a[2]*T + 3*_a[3]*T2 + 4*_a[4]*T3 + 5*_a[5]*T4;
			ad = 2*_a[2] + 6*_a[3]*T + 12*_a[4]*T2 + 20*_a[5]*T3;
		}
	}

private:
	//  ---------------------- Doxygen info ----------------------
	//! \var double _a[6];
	//! 
	//! \brief polynomial coefficients
	//  ----------------------------------------------------------
	double _a[6];
};

//  ---------------------- Doxygen info ----------------------
//! \class _SepticPolynomialInterpolator
//!
//! \brief
//! This implements a septic interpolator class
//! to be used for the interface of NRMKFoundation library
//!
//! \details 
//! This class implements the septic interpolator class 
//! for interpolating a scalar variable for a given pair of initial 
//! and final jerk as well as position, velocity, and acceleration. 
//! No boundary conditions are not taken into account.
//!
//! \sa _PolynomialInterpolator
//  ----------------------------------------------------------
class _SepticPolynomialInterpolator : public _PolynomialInterpolator<POLYNOMIAL_INTERPOLATOR_SEPTIC, _SepticPolynomialInterpolator>
{
public:
	inline void setInitialTraj(double t0, double p0, double v0 = 0, double a0 = 0, double j0 = 0)
	{
		_t0 = t0;
		_p0 = p0;
		_v0 = v0;
		_a0 = a0;
		_j0 = j0;
	}

	//! \details This calculates the coefficients of the seventh-order polynomial.
	inline void setTargetTraj(double tf, double pf, double vf = 0, double af = 0, double jf = 0)
	{
		_tf = tf;
		_duration = _tf - _t0;		

		_pf = pf;
		_vf = vf;
		_af = af;
		_jf = jf;

		_a[0] = _p0;
		_a[1] = _v0;
		_a[2] = _a0/2;
		_a[3] = _j0/6;

		double h = pf - _p0;
		
		double T2 = _duration*_duration;
		double T3 = T2*_duration;
		double T4 = T3*_duration;
		double T5 = T4*_duration;
		double T6 = T5*_duration;

		_a[4] = (210*h - _duration*((30*_a0 - 15*_af)*_duration + (4*_j0 + _jf)*T2 + 120*_v0 + 90*_vf))/(6*T4);
		_a[5] = (-168*h + _duration*((20*_a0 - 14*_af)*_duration + (2*_j0 + _jf)*T2 + 90*_v0 + 78*_vf))/(2*T5);
		_a[6] = (420*h - _duration*((45*_a0 - 39*_af)*_duration + (4*_j0 + 3*_jf)*T2 + 216*_v0 + 204*_vf))/(6*T6);
		_a[7] = (-120*h + _duration*((12*_a0 - 12*_af)*_duration + (_j0 + _jf)*T2 + 60*_v0 + 60*_vf))/(6*T6*_duration);
	}

	//! \details Boundary conditions are not respected.
	inline void setBoundaryCond(double vmax = 0, double amax = 0, double = 0)
	{
	}

	//! \details sets the duration
	inline void setDuration(double duration)
	{
	}

	//! \details This calculates the desired trajectory at a given time.
	inline void traj(double t, double & pd, double & vd, double & ad)
	{
		double T = t - _t0;
		double T2 = T*T;
		double T3 = T2*T;
		double T4 = T3*T;
		double T5 = T4*T;
		double T6 = T5*T;

		pd = _a[0] + _a[1]*T + _a[2]*T2 + _a[3]*T3 + _a[4]*T4 + _a[5]*T5 + _a[6]*T6 + _a[7]*T6*T;
		vd = _a[1] + 2*_a[2]*T + 3*_a[3]*T2 + 4*_a[4]*T3 + 5*_a[5]*T4 + 6*_a[6]*T5 + 7*_a[7]*T6;
		ad = 2*_a[2] + 6*_a[3]*T + 12*_a[4]*T2 + 20*_a[5]*T3 + 30*_a[6]*T4 + 42*_a[7]*T5;
	}

private:
	//  ---------------------- Doxygen info ----------------------
	//! \var double _a[8];
	//! 
	//! \brief polynomial coefficients
	//  ----------------------------------------------------------
	double _a[8];
};

template<PolynomialInterpolatorMode MODE>
struct _PolynomialInterpolatorType;

template<>
struct _PolynomialInterpolatorType<POLYNOMIAL_INTERPOLATOR_NONE>
{
	typedef _ZeroOrderPolynomialInterpolator Type;
};

template<>
struct _PolynomialInterpolatorType<POLYNOMIAL_INTERPOLATOR_LINEAR>
{
	typedef _LinearPolynomialInterpolator Type;
};

template<>
struct _PolynomialInterpolatorType<POLYNOMIAL_INTERPOLATOR_PARABOLIC>
{
	typedef _ParabolicPolynomialInterpolator Type;
};

template<>
struct _PolynomialInterpolatorType<POLYNOMIAL_INTERPOLATOR_CUBIC>
{
	typedef _CubicPolynomialInterpolator Type;
};

template<>
struct _PolynomialInterpolatorType<POLYNOMIAL_INTERPOLATOR_QUINTIC>
{
	typedef _QuinticPolynomialInterpolator Type;
};

template<>
struct _PolynomialInterpolatorType<POLYNOMIAL_INTERPOLATOR_SEPTIC>
{
	typedef _SepticPolynomialInterpolator Type;
};

}

} // namespace NRMKFoundation
