//  ---------------------- Doxygen info ----------------------
//! \file BlendedPolynomialAlgorithm.h
//!
//! \brief
//! Header file for the class BlendedPolynomialAlgorithm (Internal API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a set of classes for blended polynomai interpolator 
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
//! \date February 2016
//! 
//! \version 1.9.3
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//!
//! \note v1.9.4 (20160513): Added synchronized trapezoidal interpolator
//!
//! \note Copyright (C) 2013--2016 Neuromeka Co., Ltd.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#if defined(_DEBUG_PRINT)
#include <iostream>
#endif

// #ifdef __RTX__
// #include <float.h>
// #else
#include <limits>
//#endif

/// ADDED @20160503
#include <algorithm>

#include "PolynomialAlgorithm.h"

namespace NRMKFoundation
{
namespace internal
{

template<typename T>
inline int _sign(T const & z)
{
	return (z > T(0)) ? 1 : ((z < T(0)) ? -1 : 0);
}

template<typename T>
inline T _sat(T const & z)
{
	return (z > T(1)) ? 1 : ((z < T(-1)) ? -1 : z);
}

template <class T> 
const T& _max (const T & a, const T & b) 
{
	return (a < b) ? b : a;     
}

template <class T> 
const T& _min (const T & a, const T & b) 
{
	return (a > b) ? b : a;     
}

//  ---------------------- Doxygen info ----------------------
//! \class _TrapezoidalInterpolator
//!
//! \brief
//! This implements a trapezoidal interpolator (a.k.a. linear interpolator 
//! with parabolic blends) class to be used for the interface of NRMKFoundation library
//!
//! \details 
//! This class implements the trapezoidal interpolator class 
//! for interpolating a scalar variable for a given pair of initial 
//! and final position and velocity. 
//! The default interpolator when the boundary conditions are not specified 
//! is defined by two _duration ratios of the first and last acceleration (or deceleration) phase.
//! Given the boundary conditions, the assigned maximum acceleration will be achieved if feasible.
//! When the maximum velocity cannot be respected, the time _duration is modified accordingly.
//! That means the total _duration may change.
//!
//! \sa _PolynomialInterpolator
//  ----------------------------------------------------------
class _TrapezoidalInterpolator : public _PolynomialInterpolator<POLYNOMIAL_INTERPOLATOR_LINEAR_PARABOLIC_BLEND, _TrapezoidalInterpolator>
{
public:
// 	//  ---------------------- Doxygen info ----------------------
// 	//! \fn _TrapezoidalInterpolator(double acc_portion, double dec_portion)
// 	//! 
// 	//! \brief
// 	//! Constructor of the class with the portion to fix the two acceleration/deceleration phase
// 	//!
// 	//! \details
// 	//! When acc_portion and dec_portion is given, the first acc_potion * 100 (%) of the 
// 	//! total _duration will be the first acceleration (or deceleration) phase
// 	//! being followed by the constant velocity segment and the deceleration (or acceleration) phase
// 	//! spanning the dec_potion * 100 (%) of the total _duration at the end of the _duration.
// 	//!
// 	//! \note two parameters should be between 0 and 1, and their sum should not be greater than 1.
// 	//!
// 	//! \param acc_portion the _duration ratio of the initial acceleration (or deceleration) segment
// 	//!	 It should be between 0 and 1.
// 	//! \param dec_portion the _duration ratio of the last deceleration (or acceleration) segment
// 	//!	 It should be between 0 and 1.
// 	//  ----------------------------------------------------------
// 	_TrapezoidalInterpolator(double acc_portion = -1, double dec_portion = -1) 
// 		: _Tacc(acc_portion), _Tdec(dec_portion)
// 	{
// 	}

	inline void setInitialTraj(double t0, double p0, double v0 = 0, double = 0, double = 0)
	{
		_t0 = t0;
		_p0 = p0;
		_v0 = v0;
	}

	//! \details This calculates the coefficients of the default trapezoidal interpolator.
	//!
	//! \note Only when the initial portions of two acceleration phases have been properly assigned,
	//! they are computed. When setBoundaryCond() is called later, these can be modified.
	//! 
	inline void setTargetTraj(double tf, double pf, double vf = 0, double = 0, double = 0)
	{
		_tf = tf;
		_pf = pf;
		_vf = vf;

		//_vmax = 0;
		//_amax = 0;

		_direction();

		/// ADDED @20160512
		_setupTraj();
	}

	//! \details This sets the maximum velocity and acceleration for the default trapezoidal interpolator.
	//!
	//! \note In general, they can modify the segment durations. 
	//! Depending on the maximum acceleration, the trapezoidal trajectory can not be realized.
	//! feasible(amax) can detect this feasibility. If not feasible, the boundary conditions are neglected.
	//! The _duration may not be consistent with the maximum velocity condition. 
	//! In this case the _duration time is modified to attain the boundary conditions.
	//! 
	//! Zero (or negative) vmax means that vmax is computed internally.
	//! Zero (or negative) amax means that amax is computed internally. 
	//! If this function is to be called, it should be called before setDuration().
	//! 
	//! For example, 
	//!  Case 1: setBoundaryCondition(+, +) --> Ttraj (also tf) & Ta computed. 
	//!  Case 2: Given _tf > _t0, SetDuration(alpha) --> vmax & amax computed. 
	//!  Case 3: Given _tf > _t0, SetBoundaryCondition(0, +), SetDuration(alpha) --> vmax computed. (In some cases, amax may be increased.)
	//!  Case 4: Given _tf > _t0, SetBoundaryCondition(+, 0), SetDuration(alpha) --> amax computed.
	inline void setBoundaryCond(double vmax, double amax, double = 0)
	{
		_vmax = vmax;
		_amax = amax;
	}

	//! \details sets the duration
	inline void setDuration(double duration)
	{
	}

	//! \details This sets the maximum velocity and acceleration for the default trapezoidal interpolator.
	//!
	//! \note In general, they can modify the segment durations. 
	//! Depending on the maximum acceleration, the trapezoidal trajectory can not be realized.
	//! feasible(amax) can detect this feasibility. If not feasible, the boundary conditions are neglected.
	//! The _duration may not be consistent with the maximum velocity condition. 
	//! In this case the _duration time is modified to attain the boundary conditions.
	//! 
	//! \param acc_portion the _duration ratio of the initial acceleration (or deceleration) segment
	//!	 It should be between 0 and 1.
	//! \param dec_portion the _duration ratio of the last deceleration (or acceleration) segment
	//!	 It should be between 0 and 1.
	inline void setDurations(double alpha, double = 0) 
	{
 		assert(0 < alpha && alpha <= 0.5);

		if (_vmax == 0 && _amax == 0) // means setBoundaryCond() has not been called
		{
			assert(_v0 == 0 && _vf == 0);

			double h = _pf - _p0;

			_duration = _tf - _t0;

			_Ta = _Td = _duration*alpha;

			//_construct_bc();

			_vmax = h/(_duration - _Ta);
			_amax = _vmax/_Ta;

			_setCoefficients_amax_zero_vel_tc();
		}
		else
		{
			assert(_vmax > 0 && _amax <= 0);

			_Ta = _Td = _duration*alpha;
			_vv = _vmax;

			if (_v0 == 0 && _vf == 0)
			{
				_setCoefficients_vv_zero_vel_tc();
			}
			else
			{
				_setCoefficients_vv();
			}			
		}
	}

	//! \details This calculates the desired trajectory at a given time.
	inline void traj(double t, double & pd, double & vd, double & ad)
	{
		if (t < _t0)
		{
			pd = _p0;
			vd = 0;
			ad = 0;
		}
		else if (t < _t0 + _Ta)
		{
			double dt = t - _t0;

			pd = _a[0] + _a[1]*dt + _a[2]*dt*dt;
			vd = _a[1] + 2*_a[2]*dt;
			ad = 2*_a[2];
		}
		else if (t < _tf - _Td)
		{
			pd = _b[0] + _b[1]*(t - _t0 - _Ta/2);
			vd = _b[1];
			ad = 0;
		}
		else if (t < _tf)
		{
			double dt = _tf - t;

			pd = _c[0] + _c[1]*dt + _c[2]*dt*dt;
			vd = -_c[1] - 2*_c[2]*dt;
			ad = 2*_c[2];
		}
		else
		{
			pd = _pf;
			vd = 0;
			ad = 0;
		}

		// modify the direction
		if (_dir == -1)
		{
			ad = -ad;
			vd = -vd;
			pd = -pd;
		}
	}

private:
	//! \details This sets the maximum velocity and acceleration for the default trapezoidal interpolator.
	//!
	//! \note In general, they can modify the segment durations. 
	//! Depending on the maximum acceleration, the trapezoidal trajectory can not be realized.
	//! feasible(amax) can detect this feasibility. If not feasible, the boundary conditions are neglected.
	//! The _duration may not be consistent with the maximum velocity condition. 
	//! In this case the _duration time is modified to attain the boundary conditions.
	//! 
	//! Zero (or negative) vmax means that vmax is computed internally.
	//! Zero (or negative) amax means that amax is computed internally. 
	//! If this function is to be called, it should be called before setDuration().
	//! 
	//! For example, 
	//!  Case 1: setBoundaryCondition(+, +) --> Ttraj (also tf) & Ta computed. 
	//!  Case 2: Given _tf > _t0, SetDuration(alpha) --> vmax & amax computed. 
	//!  Case 3: Given _tf > _t0, SetBoundaryCondition(0, +), SetDuration(alpha) --> vmax computed. (In some cases, amax may be increased.)
	//!  Case 4: Given _tf > _t0, SetBoundaryCondition(+, 0), SetDuration(alpha) --> amax computed.
	inline void _setupTraj()
	{
		if (_amax > 0)
		{
			if (!_feasible())
			{
				assert(false);
				return;
			}

			double h = _pf - _p0;

			if (_vmax > 0)	// vmax and amax are respected
			{

				// construct time constants
				if (_v0 == 0 && _vf == 0) // with zero velocity bcs, alwayd feasible
				{
					if (_amax*h > _vmax*_vmax) // vmax reached
					{
						_Ta = _Td = _vmax/_amax;
						_duration = h/_vmax + _vmax/_amax;
					}
					else
					{
						_Ta = _Td = ::sqrt(h/_amax); 
						_duration = _Ta + _Td;
					}

					_setCoefficients_amax_zero_vel_tc();
				}
				else
				{
					double vlim2 = _amax*h + (_v0*_v0 + _vf*_vf)/2;
					//if (_amax*h > (_vmax*_vmax- (_v0*_v0 + _vf*_vf)/2)) // vmax reached
					if (vlim2 > _vmax*_vmax) // vmax reached
					{
						_vv = _vmax;
						_Ta = (_vmax - _v0)/_amax;
						_Td = (_vmax - _vf)/_amax;

						double u0 = 1 - _v0/_vmax;
						double uf = 1 - _vf/_vmax;

						_duration = h/_vmax + _vmax/_amax*(u0*u0 + uf*uf)/2;
					}
					else
					{
						//_vlim = ::sqrt(h*amax + (v02 + vf2)/2);
						_vv = _vlim = ::sqrt(vlim2);
						_Ta = (_vlim - _v0)/_amax;
						_Td = (_vlim - _vf)/_amax;

						_duration = _Ta + _Td;
					}

					_setCoefficients_vv();
				}

				_tf = _t0 + _duration;
			}
			else // _duration and amax are respected
			{
				_duration = _tf - _t0;

				double T2 = _duration*_duration;

				// construct time constants
				if (_v0 == 0 && _vf == 0) // with zero velocity bcs, always feasible
				{
					double alim = 4*h/T2;

					if (_amax >= alim) // vmax reached
					{
						double vdiff = _amax*_duration;
						_vmax = _vv = (vdiff - ::sqrt(vdiff*vdiff - 4*_amax*h))/2;
					}
					else // modifying amax for feasibility
					{
						_amax = alim;
						_vmax = _vv = (_amax*_duration)/2;
					}

					_Ta = _Td = _vv/_amax;

					_setCoefficients_amax_zero_vel_tc();
				}
				else
				{
					// preassigned acceleration and _duration
					double vsum = _v0 + _vf;
					//double b = _duration*vsum - 2*h;
					//double c = (_v0 - _vf);

					double v2sum = _v0*_v0 + _vf*_vf;
					double alim = (2*h - _duration*vsum + ::sqrt(4*h*(h - vsum*_duration) + 2*v2sum*T2))/T2;

					if (_amax >= alim)	// max velocity exists
					{
						double vdiff = vsum + _amax*_duration;
						_vmax = _vv = (vdiff - ::sqrt(vdiff*vdiff - 2*v2sum - 4*_amax*h))/2;
					}
					else	// amax reset to alim
					{
						_amax = alim;
						_vmax = _vv = (vsum + _amax*_duration)/2;
					}

					_Ta = (_vv - _v0)/_amax;
					_Td = (_vv - _vf)/_amax; 		

					_setCoefficients_vv();	
				}
			}
		}
		else // amax <= 0 --> vmax and Ta will be respected
		{
			assert(_vmax > 0);

			_duration = _tf - _t0;
		}
	}

	//! \details This checks whether the maximum acceleration is feasible 
	//! with the initial and final position and velocity conditions.
	//! 
	inline bool _feasible() const 
	{
		if (_v0 == 0 && _vf == 0) // Zero velocity trajectory conditions always leads to feasible trajectory.
			return true;
		else
			return 2*_amax*::fabs(_pf - _p0) >= ::fabs(_vf*_vf - _v0*_v0);
	}

	inline void _setCoefficients_vv()
	{
		_a[0] = _p0;
		_a[1] = _v0;
		_a[2] = (_vv - _v0)/(2*_Ta);

		_b[0] = _p0 + _v0*_Ta/2;
		_b[1] = _vv;

		_c[0] = _pf;
		_c[1] = -_vf;
		_c[2] = -(_vv - _vf)/(2*_Td);
	}

	inline void _setCoefficients_amax_zero_vel_tc()
	{
		_a[0] = _p0;
		_a[1] = 0;
		_a[2] = _amax/2;

		_b[0] = _p0;
		_b[1] = _amax*_Ta;

		_c[0] = _pf;
		_c[1] = 0;
		_c[2] = -_a[2]; // = -_amax/2; 
	}

	inline void _setCoefficients_vv_zero_vel_tc()
	{
		_a[0] = _p0;
		_a[1] = 0;
		_a[2] = _vv/(2*_Ta);

		_b[0] = _p0;
		_b[1] = _vv;

		_c[0] = _pf;
		_c[1] = 0;
		_c[2] = -_vv/(2*_Td);
	}

	inline void _direction()
	{
		_dir = 1;
		if (_p0 > _pf)
		{
			_dir = -1;

			_p0 *= _dir;
			_pf *= _dir;
			_v0 *= _dir;
			_vf *= _dir;
		}			
	}

private:
	//  ---------------------- Doxygen info ----------------------
	//! \var double _a[6];
	//! 
	//! \brief polynomial coefficients
	//  ----------------------------------------------------------
	double _a[3];
	double _b[2];
	double _c[3];

	// time parameters
	double _Ta;	// acceleration _duration
	double _Td;	// deceleration _duration

	double _vv;
	double _vlim;

	int _dir;	
};


//  ---------------------- Doxygen info ----------------------
//! \class _SyncTrapezoidalInterpolator
//!
//! \brief
//! This implements a synchronized time-optimal trapezoidal interpolator class 
//! to be used for the interface of NRMKFoundation library
//!
//! \details 
//! This class implements the synchronized time-optimal trapezoidal interpolator class 
//! for interpolating a scalar variable for a given pair of initial and final position and velocity. 
//! It implements F. Ramos's algorithm (Time-Optimal Online Trajectory Generator for Robotic Manipulators)
//! available in http://www.roboearth.org/wp-content/uploads/2013/02/OMG.pdf. 
//!
//! \sa _PolynomialInterpolator
//  ----------------------------------------------------------
class _SyncTrapezoidalInterpolator : public _PolynomialInterpolator<POLYNOMIAL_TRAPEZIODAL_SYNC, _SyncTrapezoidalInterpolator>
{
public:
	//! \details This sets the initial condition 
	//! 
	inline void setInitialTraj(double t0, double p0, double v0 = 0, double = 0, double = 0)
	{
		_t0 = t0;
		_p0 = p0;
		_v0 = v0;

		_curProfile = 0;
		_noSubProfiles = 0;
	}

	//! \details This calculates the coefficients of the default trapezoidal interpolator.
	//!
	//! \note The boundary conditions such as maximum velocity and acceleration should be specified using setBopundaryCond()
	//! before calling this function.
	//! 
	inline void setTargetTraj(double , double pf, double vf = 0, double = 0, double = 0)
	{
		//_tf = tf;
		_tf = _traj_time_undefined;
		_pf = pf;
		_vf = vf;

		_buildProfile();  // Initial time set to zero for new trajectories

		// Set SynchroTime to Duration as an initial value
		_Tsync = _duration;
	}

	//! \details This sets the maximum velocity and acceleration for the default trapezoidal interpolator.
	//!
	//! \note In general, this should be called before setTargetTraj(). 
	inline void setBoundaryCond(double vmax, double amax, double = 0)
	{
		_vmax = vmax;
		_amax = amax;
				
		_trajSign = 1.0;
		_syncSign = 1.0;
	}

	//! \details sets the duration
	inline void setDuration(double duration)
	{
		_setDuration(duration);
	}

	//! \details This calculates the desired trajectory at a given time.
	inline void traj(double t, double & pd, double & vd, double & ad)
	{
		while (_curProfile < (int) _noSubProfiles - 1)
		{
			if (t < _subProfiles[_curProfile + 1][0])
				break;
			else
				_curProfile++;
		}			

		double const * const profile = _subProfiles[_curProfile];

		double dt = t - profile[0];

		pd = profile[1] + dt*(profile[2] + 0.5*profile[3]*dt);
		vd = profile[2] + profile[3]*dt;
		ad = profile[3];

#if defined(_DEBUG_PRINT)
		std::cout << "===== Current profile: " << _curProfile << "================" << std::endl;
    	std::cout << "t " << t << std::endl;
    	std::cout << "pd " << pd << std::endl;
    	std::cout << "vd " << vd << std::endl;
    	std::cout << "ad " << ad << std::endl;
#endif
	}
	
private:
	inline void _buildProfile() 
	{
		static const double epsilon = 0.0001;

		// We need these auxiliar variables for classifying the trajectory
		double dp = _pf - _p0; // Distance to cover
		double dv = _vf - _v0; // Velocity leap
		double vmean = (_vf + _v0)/2; // Mean velocity for critical trajectory
		double tf_cr = ::fabs(dv/_amax); // Duration of critical trajectory
		double dp_cr = vmean*tf_cr; // Critical distance to be covered

		///////////////
		// TODO: Check on the velocity if we are going to hit the limits with the final state!!!
		// We cannot check here as the hardware limits are not available. Should this be changed??
		// Passing the check to the Trajectory Generator
		///////////////

		// _trajSign gives the direction of the trajectory
		// +1 for trajectories over the dp_cr
		_trajSign = 0.0;
		if (dp > dp_cr + epsilon)
			_trajSign = 1.0;
		// -1 for trajectories under the dp_cr
		else if (dp < dp_cr - epsilon)
			_trajSign = -1.0;
		// If the distance to cover is equal to critical, the direction depends on velocity leap
		else if (dv > epsilon)
			_trajSign = 1.0;
		else if (dv < -epsilon)
			_trajSign = -1.0;

#if defined(_DEBUG_PRINT)
			std::cout << "======================================" << std::endl;
    		std::cout << "_trajSign " << _trajSign << std::endl;
    		std::cout << "_p0 " << _p0 << std::endl;
    		std::cout << "_pf " << _pf << std::endl;
    		std::cout << "_v0 " << _v0 << std::endl;
    		std::cout << "_vf " << _vf << std::endl;
    		std::cout << "dp " << dp << std::endl;
       		std::cout << "dp_cr " << dp_cr << std::endl;
       		std::cout << "vmean " << vmean << std::endl;
#endif

		// If the initial state is equal to final state: we are at the target!!!
		if (_trajSign == 0.0)
		{
			// We create a dummy maneuver
			double * const sp = _subProfiles[_noSubProfiles++];

			sp[0] = _t0; 
			sp[1] = _p0; 
			sp[2] = _v0; 
			sp[3] = 0.0; 		

			_duration = epsilon;
		}
		else
		{
			// We calculate maximum velocity reached during maneuver
			double vsqr = _v0*_v0 + _vf*_vf;
			double v_peak = ::sqrt(_trajSign*dp*_amax + vsqr/2);
			//double v_peak = ::sqrt(_trajSign*dp*_amax + (_v0*_v0+_vf*_vf)/2);

#if defined(_DEBUG_PRINT)
       		std::cout << "v_peak " << v_peak << std::endl;
#endif
			
			if (v_peak < _vmax)
			{
				//slow down to zero and perform a new trajectory going back
				//log(Info) << "---- Triangular Velocity Profile ----" << endlog();
				// Two pieces trajectory
				// std::vector<double> sp1,sp2;

				// Characteristic times
				double T20 = (_trajSign*v_peak - _v0)/(_trajSign*_amax);

				double * const sp1 = _subProfiles[_noSubProfiles++];

				sp1[0] = _t0; 
				sp1[1] = _p0; 
				sp1[2] = _v0; 
				sp1[3] = _trajSign*_amax; 		

				double * const sp2 = _subProfiles[_noSubProfiles++];
				
				sp2[0] = _t0 + T20; 
				sp2[1] = sp1[1] + T20*(sp1[2] + 0.5*T20*sp1[3]); 
				sp2[2] = _trajSign*v_peak; 
				sp2[3] = -_trajSign*_amax; 		

#if defined(_DEBUG_PRINT)
	       		std::cout << "T20 " << T20 << std::endl;
#endif

				_duration = T20 + (_vf - _trajSign*v_peak)/(-_trajSign*_amax);
				//end of triangular velocity profile

			}
			else
			{
				// log(Info) << "---- Trapezoidal Velocity Profile ----" << endlog();
				// Three pieces trajectory
				//std::vector<double> sp1,sp2,sp3;
				// Characteristic times
				double T20 = (_trajSign*_vmax - _v0)/(_trajSign*_amax);
				double T30 = 1/(_vmax) * (_trajSign*dp + (vsqr - _trajSign*2*_vmax*_v0)/(2*_amax));

				double * const sp1 = _subProfiles[_noSubProfiles++];

				sp1[0] = _t0; 
				sp1[1] = _p0; 
				sp1[2] = _v0; 
				sp1[3] = _trajSign*_amax; 			

				//Adding the constant velocity subProfile
				double * const sp2 = _subProfiles[_noSubProfiles++];
				
				sp2[0] = _t0 + T20; 
				//sp2[1] = sp1[1] + (T20 - _t0)*(sp1[2] + 0.5*(T20 - _t0)*sp1[3]); 
				sp2[1] = sp1[1] + T20*(sp1[2] + 0.5*T20*sp1[3]); 
				sp2[2] = _trajSign*_vmax; 
				sp2[3] = 0.0; 			


				//Adding the Deceleration subProfile
				double * const sp3 = _subProfiles[_noSubProfiles++];
				
				sp3[0] = _t0 + T30; 
				sp3[1] = sp2[1] + (T30 - T20)*sp2[2]; 
				sp3[2] = _trajSign*_vmax; 
				sp3[3] =  -_trajSign*_amax; 	

				_duration = T30 + (_vf - _trajSign*_vmax)/(-_trajSign*_amax);

#if defined(_DEBUG_PRINT)
	       		std::cout << "T20 " << T20 << std::endl;
	       		std::cout << "T30 " << T30 << std::endl;
	       		std::cout << "_duration " << _duration << std::endl;
#endif
				//End of Asymmetric trapezoidal velocity profile
			}
		} // if-else that detects if the final state is equal to the original


		//TODO: Add a final piece of trajectory that decelerates smoothly to zero
		/* Fix issue found by gcc6.5: call of overloaded 'abs(double&)' is ambiguous */
        //if (abs(_vf) > epsilon)
        if (std::abs(_vf) > epsilon)
		{
			// Decelerate system to zero
			double * const spDec = _subProfiles[_noSubProfiles++];
				
			spDec[0] = _t0 + _duration; 
			spDec[1] = _pf; 
			spDec[2] = _vf; 
			spDec[3] = -_vf/::fabs(_vf)*_amax; 			
		}
		
		// Finally, we add a steady piece giving the final state for times higher than _duration+Tdec
		double * const spSS = _subProfiles[_noSubProfiles++];
		double Tdec = ::fabs(_vf)/_amax;
				
		spSS[0] = _t0 + _duration + Tdec; 
		spSS[1] = _pf + 0.5*_vf*Tdec; 
		spSS[2] = 0.0; 
		spSS[3] = 0.0; 			

#if defined(_DEBUG_PRINT)
   		std::cout << "_amax " << _amax << std::endl;
   		std::cout << "_vmax " << _vmax << std::endl;
   		std::cout << "_duration " << _duration << std::endl;
   		std::cout << "======================================" << std::endl;
#endif
	}

	inline void _setDuration(double newDuration)
	{
		static const double epsilon = 0.0001;

		//if (newDuration <= _duration) 
		if (::fabs(newDuration - _duration) <= epsilon) 
		{
    		_Tsync = _duration;
#if 0
    		std::cout << "Requested time for synchronization is not valid" << std::endl;
#endif
		}
		else if (_vf != 0.0)
		{
    		_Tsync = _duration;
#if 0
    		std::cout << "Cannot synchronize for vf other than zero" << std::endl;
#endif
		}
		//Synchronizing trajectories
		else 
		{
			double vconst = _vmax;

			double dp = _pf - _p0;

			// It synchronization time is very large, we need a double ramp profile
			if (_v0 == 0.0 || (_trajSign != ::fabs(_v0)/_v0))
				_syncSign = 1.0;
			//else if (newDuration > (_pf - _p0)/_v0 + 0.5*_v0/(_trajSign*_amax))
			else if (newDuration > dp/_v0 + 0.5*_v0/(_trajSign*_amax))
				_syncSign = -1.0;

			//if(::fabs(_pf - _p0) < epsilon && abs(_v0) < epsilon )
            /* Fix issue found by gcc6.5: call of overloaded 'abs(double&)' is ambiguous */                
			//if(::fabs(dp) < epsilon && abs(_v0) < epsilon )
            if(::fabs(dp) < epsilon && std::abs(_v0) < epsilon )
			{
				//std::cout << "Zero trajectory. It will not be synchronized" << std::endl; 
				;
			}

			///// The trajectory generator has been changed to allow for vf other than zero
			///// At the same time, the posible trajectories have been reduced to two cases
			///// triangular and trapezoidal, without this deceleration one (particular case of triangular)

			///////////////////
			else
			{
				_Tsync = newDuration;

				// Asymmetric trapezoidal or double ramp velocity profile (depending on syncTime)
				//log(Info) << "Synchronizing trajectory " << endlog();
				// Calculate the constant velocity along the maneuver
				if (_syncSign == 1.0)
				{
					double aux = _amax*_Tsync + _trajSign*_v0;
					vconst = 0.5*(aux - sqrt(aux*aux - 4*_trajSign*_amax*dp - 2*_v0*_v0));
		//				std::cout << "Trapezoidal synchronized trajectory" << endlog();
				}
				else{
					vconst = (_trajSign*dp - 0.5*_v0*_v0/_amax)/(_Tsync - _v0/(_trajSign*_amax));
		//				std::cout << "Double ramp synchronized trajectory" << endlog();
				}


#if defined(_DEBUG_PRINT)
    			std::cout << "_trajSign " << _trajSign << std::endl;
    			std::cout << "_syncSign " << _syncSign << std::endl;
    			std::cout << "_Tsync " << _Tsync << std::endl;
    			std::cout << "vconst " << vconst << std::endl;
    			std::cout << "_amax " << _amax << std::endl;

#endif
				// Change the acceleration in case the sync time is very large
				_subProfiles[0][3] = _syncSign*_trajSign*_amax;

				// Calculate the modified constant velocity subprofile (2nd part)
				double T1 = (_trajSign*vconst - _v0)/(_syncSign*_trajSign*_amax);
				double P1 = _p0 + 0.5*T1*(_trajSign*vconst + _v0) ; // position at the start of the subProfile
				_subProfiles[1][0] = T1 + _t0;
				_subProfiles[1][1] = P1;
				_subProfiles[1][2] = _trajSign*vconst;
				_subProfiles[1][3] = 0.0;

				// Calculate the modified deceleration subprofile (3rd part)
				double T2 = _Tsync - vconst/_amax;
				double P2 = P1 + (T2 - T1)*_trajSign*vconst;

				_subProfiles[2][0] = T2 + _t0;
				_subProfiles[2][1] = P2;
				_subProfiles[2][2] = _trajSign*vconst;
				_subProfiles[2][3] = -_trajSign*_amax;

				// Finally, we add a steady piece giving the final state for times higher than _duration+TDec
				_subProfiles[3][0] = _Tsync;
				_subProfiles[3][1] = _pf;
				_subProfiles[3][2] = 0.0;
				_subProfiles[3][3] = 0.0;

				_noSubProfiles = 4;

			//End of Synchronization
			}
	////////////////////////
		}


#if defined(_DEBUG_PRINT)
		/*
		std::cout << "Synchronized trajectory" << std::endl;
		std::cout << "=======================" << std::endl;
		std::cout << "fPos " << _pf << " _p0 " << _p0 << " iVel " << _v0 << std::endl;
		if(abs(_pf-_p0) > epsilon || abs(_v0) > epsilon )
		{
			int i;
			for( i = 0 ; i < _noSubProfiles - 1 ; i++ )
			{
				std::cout << "Trajectory piece " << i << std::endl;
				std::cout << "------------------" << std::endl;
				std::cout << "_t0: " << _subProfiles[i][0] << "; _p0: " << _subProfiles[i][1] << "; _v0: " \
						<< _subProfiles[i][2] << "; initAcc: " << _subProfiles[i][3] << std::endl;
				std::cout << "finalTime: " << _subProfiles[i+1][0] << "; _pf: " << Pos(_subProfiles[i+1][0]-0.001) << "; _vf: " \
						<< Vel(_subProfiles[i+1][0]-0.001) << "; finalAcc: " << Acc(_subProfiles[i+1][0]-0.001) << std::endl;
			}
			std::cout << "Trajectory piece " << i << std::endl;
			std::cout << "------------------" << std::endl;
			std::cout << "_t0: " << _subProfiles[i][0] << "; _p0: " << _subProfiles[i][1] << "; _v0: " \
					<< _subProfiles[i][2] << "; initAcc: " << _subProfiles[i][3] << std::endl;
			std::cout << "Final values" << std::endl;
			std::cout << "------------" << std::endl;
			std::cout << "finalTime: " << _Tsync << "; _pf: " << Pos(_Tsync) << "; _vf: " \
					<< Vel(_Tsync) << "; finalAcc: " << Acc(_Tsync) << std::endl;
		}
		else
		{
			std::cout << "Trajectory piece 0" << std::endl;
			std::cout << "------------------" << std::endl;
			std::cout << "Begin of printing" << std::endl;
			std::cout << "_t0: " << _subProfiles[0][0] << "; _p0: " << _subProfiles[0][1] << "; _v0: " \
					<< _subProfiles[0][2] << "; initAcc: " << _subProfiles[0][3] << std::endl;
			std::cout << "End of printing" << std::endl;
		}
		*/
#endif
	}

private:
	//static const double epsilon  = 0.0001;

	 /** It is a vector of SubProfiles, each of them with the coefficients
     *  of a polynomial corresponding to that piece of the profile
     */
	//std::vector< std::vector<double> > _subProfiles;
	int _curProfile;		// current progile index
	double _subProfiles[5][4];		// 5 for maximum numbger of segment
	unsigned int _noSubProfiles;

	// time parameters
	double _Tsync;	// synchro time
	
	double _trajSign;	// sign of the trajectory
	double _syncSign;	// sign of synchronization. Distinguishes between trap and double ramp profiles

};

//  ---------------------- Doxygen info ----------------------
//! \class _DoubleSInterpolator
//!
//! \brief
//! This implements a double-S interpolator class to be used for the interface of NRMKFoundation library
//!
//! \details 
//! This class implements the double-S interpolator class 
//! for interpolating a scalar variable for a given pair of initial 
//! and final position and velocity. 
//! The default interpolator when the boundary conditions are not specified 
//! is defined by two _duration ratios of the first and last acceleration (or deceleration) phase.
//! Given the boundary conditions, the assigned maximum acceleration will be achieved if feasible.
//! When the maximum velocity cannot be respected, the time _duration is modified accordingly.
//! That means the total _duration may change.
//!
//! \sa _PolynomialInterpolator
//  ----------------------------------------------------------
class _DoubleSInterpolator : public _PolynomialInterpolator<POLYNOMIAL_INTERPOLATOR_DOUBLE_S, _DoubleSInterpolator>
{
public:
	inline void setInitialTraj(double t0, double p0, double v0 = 0, double = 0, double = 0)
	{
		_t0 = t0;
		_p0 = p0;
		_v0 = v0;
	}

	//! \details This sets the final conditions of the default double-S interpolator.
	//!
	//! \note 
	//! After calling this function, users can call setBoundaryCond() to set maximum velocity/acceleration/jerk condition 
	//  to define the time constants of the double-S trajectory. If this is intended, the first argument is modified. 
	//! Alternatively, setDurations() can be called to keep the final time by imposing maximum conditions internally.
	//! 
	inline void setTargetTraj(double tf, double pf, double vf, double = 0, double = 0)
	{
		_tf = tf;	
		_pf = pf;
		_vf = vf;

		// determine direction
		_direction();

		/// ADDED @20160512
		_setupTraj();
	}

	//! \details This sets the maximum velocity/acceleration/jerk for the default double-S interpolator.
	//!
	//! \note The limit is symmetric in the sense that vmin = -vmax, amin = -amin, jmin = -jmax.
	//! In terms of initial and final trajectory conditions as well as these boundary conditions
	//! the time constants defining a complete double-S trajectory are determined. 
	//! 
	inline void setBoundaryCond(double vmax, double amax, double jmax)
	{
		_vmax = vmax;
		_amax = amax;
		_jmax = jmax;

		_vmin = -vmax;
		_amin = -amax;
		_jmin = -jmax;
	}

	//! \details sets the duration
	inline void setDuration(double duration)
	{
	}

	//! \details 
	//! This sets the maximum jerk portion (during initial acceleration phase) and 
	//! the minimum jerk portion (during final deceleration phase). 
	//!
	//! \note 
	//! One can always generate feasible double-S trajectory using this parameter.
	//! Both parameters are positive number <= 0.5.
	//! 
	inline void setDurations(double alpha, double beta)
	{
		assert(0 < alpha && alpha <= 0.5);
		assert(0 < beta && beta <= 0.5);

		_duration = _tf - _t0;

		_Ta = _Td = _duration*alpha;
		_Tj1 = _Tj2 = _Ta*beta;
		_Tv = _duration - _Ta - _Td;

		_construct_bc();

		_vmin = -_vmax;
		_amin = -_amax;
		_jmin = -_jmax;
	}

	//! \details This calculates the desired trajectory at a given time.
	inline void traj(double t, double & pd, double & vd, double & ad)
	{
		t -= _t0;	// time origin shift

		if (t < 0)
		{
			_jd = 0;
			ad = 0;
			vd = 0;
			pd = _p0;
		}
		else if (t < _Tj1)
		{
			_jd = _jmax;
			
			ad = _jmax*t;
			
			//double t2 = t*t;
			//double t3 = t2*t;

			//vd = _v0 + _jmax*t2/2;
 			//pd = _p0 + _v0*t + _jmax*t3/6;

			vd = _v0 + t*ad/2; //  = _v0 + _jmax*t2/2;
			pd = _p0 + t*(vd - ad*t/3); // = _p0 + _v0*t + _jmax*t3/6;
		}
		else if (t < _Ta - _Tj1)
		{
			_jd = 0;
			ad = _jmax*_Tj1; // = a_lim_a
			vd = _v0 + ad*(t - _Tj1/2);
			pd = _p0 + _v0*t + ad/6*(3*t*(t - _Tj1) + _Tj1*_Tj1);
		}
		else if (t < _Ta)
		{
			_jd = _jmin; // = -_jmax

			double dt = _Ta - t;
			double dt2 = dt*dt;

			ad = -_jmin*dt;
			vd = _vlim + _jmin*dt2/2;
			pd = _p0 + (_vlim + _v0)/2*_Ta - dt*(_vlim + _jmin*dt2/6);
		}
		else if (t < _Ta + _Tv)
		{
			_jd = 0;
			ad = 0;
			vd = _vlim;
			pd = _p0 + (_vlim + _v0)/2*_Ta + _vlim*(t - _Ta);
		}
		else if (t < _duration - _Td + _Tj2)
		{
			_jd = _jmin;

			double dt = t - _duration + _Td;
			double dt2 = dt*dt;

			ad = -_jmax*dt;
			vd = _vlim - _jmax*dt2/2;
			pd = _pf - (_vlim + _vf)*_Td/2 + dt*(_vlim - _jmax*dt2/6);
		}
		else if (t < _duration - _Tj2)
		{
			_jd = 0;

 			double dt = _duration - t;

			ad = _jmin*_Tj2; // = a_lim_d;
			vd = _vf - ad*(dt - _Tj2/2);
			pd = _pf - _vf*dt + ad/6*(3*dt*(dt - _Tj2) + _Tj2*_Tj2);
		}
		else if (t < _duration)
		{
			_jd = _jmax;
			
			double dt = _duration - t;
			double dt2 = dt*dt;

			ad = -_jmax*dt;
			vd = _vf + _jmax*dt2/2;
			pd = _pf - dt*(_vf + _jmax*dt2/6);
		}
		else // _duration <= t
		{
			_jd = 0;
			ad = 0;
			vd = 0;
			pd = _pf;
		}

		// modify the direction
		if (_dir == -1)
		{
			ad = -ad;
			vd = -vd;
			pd = -pd;
			_jd = -_jd;
		}		
	}

	inline double jerk() const { return _jd; }

private:
	//! \details This sets the maximum velocity/acceleration/jerk for the default double-S interpolator.
	//!
	//! \note The limit is symmetric in the sense that vmin = -vmax, amin = -amin, jmin = -jmax.
	//! In terms of initial and final trajectory conditions as well as these boundary conditions
	//! the time constants defining a complete double-S trajectory are determined. 
	//! 
	inline void _setupTraj()
	{
		if (!_feasible())
		{
			assert(false);
			return;
		}

		// construct time constants
		if (_v0 == 0 && _vf == 0) // with zero velocity bcs, alwayd feasible
			_construct_times_zero_vel_tc();
		else
			_construct_times();

		// set trajectory completion time
		_duration = _Ta + _Tv + _Td;
		_tf = _t0 + _duration;
	}

	//! \details This checks whether the boundary conditions are compatible 
	//! with the initial and final trajectory conditions.
	//! 
	inline bool _feasible() const 
	{
		if (_v0 == 0 && _vf == 0) // Zero velocity trajectory conditions always leads to feasible trajectory.
			return true;
		
		// In case of non-zero velocity trajectory conditions...
		double Tj = ::sqrt(::fabs(_vf - _v0)/_jmax);
		if (Tj < _amax/_jmax)
		{
			if (_pf - _p0 > Tj*(_v0 + _vf))
				return true;
		}
		else 
		{
			Tj = _amax/_jmax;
			if (_pf - _p0 > (_v0 + _vf)/2*(Tj + ::fabs(_vf - _v0)/_amax))
				return true;
		}

		return false;
	}

	inline void _construct_times_zero_vel_tc()
	{
		_vlim = _vmax;

		double amax2 = _amax*_amax;
		
		if (_vmax*_jmax < amax2) // _amax not reached
		{
			_Tj1 = ::sqrt(_vmax/_jmax);
			_Ta = 2*_Tj1;
		}
		else
		{
			_Tj1 = _amax/_jmax;
			_Ta = _Tj1 + _vmax/_amax;
		}

		double h = _pf - _p0;

		_Tv = h/_vmax - _Ta;
		if (_Tv >= 0) // vmax reached
		{
			_Tj2 = _Tj1;
			_Td = _Ta;

			return;
		}

		// now, vmax not reached
		_Tv = 0;

		if (h < 2*amax2*_amax/(_jmax*_jmax))
		{
			_Tj1 = ::pow(h/(2*_jmax), 1.0/3.0);
			_Ta = 2*_Tj1;
		}
		else
		{
			_Tj1 = _amax/_jmax;
			
			double Tj1_2 = _Tj1/2;
			
			_Ta = Tj1_2 + ::sqrt( Tj1_2*Tj1_2 + h/_amax);
		}
		
		_Tj2 = _Tj1;
		_Td = _Ta;

		double alim_a = _jmax*_Tj1;
		_vlim = (_Ta - _Tj1)*alim_a;
	}

	inline void _construct_times()
	{
		_vlim = _vmax;

		double amax2 = _amax*_amax;
		double vmax0 = _vmax - _v0;

		if (vmax0*_jmax < amax2) // _amax not reached
		{
			_Tj1 = ::sqrt(vmax0/_jmax);
			_Ta = 2*_Tj1;
		}
		else
		{
			_Tj1 = _amax/_jmax;
			_Ta = _Tj1 + vmax0/_amax;
		}

		double vmaxf = _vmax - _vf;
		if (vmaxf*_jmax < amax2) // _amin not reached
		{
			_Tj2 = ::sqrt(vmaxf/_jmax);
			_Td = 2*_Tj2;
		}
		else
		{
			_Tj2 = _amax/_jmax;
			_Td = _Tj2 + vmaxf/_amax;
		}

		double h = _pf - _p0;

		_Tv = h/_vmax - _Ta/2*(1 + _v0/_vmax) - _Td/2*(1 + _vf/_vmax);
		if (_Tv >= 0) // vmax reached
			return;

		// now, vmax not reached
		_Tv = 0;

		double vsum = _vf + _v0;

		double gamma = 1;
		while (gamma > 0)
		{
			// solve 
			double amax = gamma*_amax;

			_Tj1 = _Tj2 = amax/_jmax;

			double amax2_j = amax*_Tj1; // = amax*amax/_jmax;
			//double D = amax2_j*amax2_j + 2*(_v0*_v0 + _vf*_vf) + amax*(4*h - 2*amax/_jmax*(_v0 + _vf));
			double D = amax2_j*amax2_j + 2*(_v0*_v0 + _vf*_vf + amax*(2*h - _Tj1*vsum));

			double common = amax2_j + ::sqrt(D);
			_Ta = (common - 2*_v0)/(2*amax);
			_Td = (common - 2*_vf)/(2*amax);

			// check
			if (_Ta > 2*_Tj1 && _Td > 2*_Tj2)
				break;

			else
			{
				if (_Ta < 0) // necessarily _v0 > _v1
				{
					_Ta = 0;
					_Tj1 = 0;
					
					_Td = 2*h/vsum;
					_Tj2 = (_jmax*h - ::sqrt(_jmax*(_jmax*h*h + vsum*vsum*(_vf - _v0))))/(_jmax*vsum);

					break;
				}

				if (_Td < 0) // necessarily _v0 < _v1
				{
					_Td = 0;
					_Tj2 = 0;

					_Ta = 2*h/vsum;
					_Tj1 = (_jmax*h - ::sqrt(_jmax*(_jmax*h*h - vsum*vsum*(_vf - _v0))))/(_jmax*vsum);

					break;
				}
			}

			gamma -= 0.01;
		}

		// cannot construct a trajectory
		if (gamma <= 0)
			assert(false);

		double alim_a = _jmax*_Tj1;
		double alim_d = _jmin*_Tj2;

		if (alim_a > 0)
			_vlim = _v0 + (_Ta - _Tj1)*alim_a;
		else 
			_vlim = _vf - (_Td - _Tj2)*alim_d;
	}
	
	inline void _construct_bc()
	{
		assert(_v0 == 0 && _vf == 0);

		double h = _pf - _p0;

		/*
		//double h = ::abs(_pf - _p0);
		
		_vmax = h/((1 - alpha)*T);
		_amax = _vmax/(alpha*(1 - beta)*T);
		_jmax = _amax/(alpha*beta*T);
		*/

		_vmax = h/(_duration - _Ta);
		_amax = _vmax/(_Ta - _Tj1);
		_jmax = _amax/_Tj1;

		_vlim = _vmax;
	}

// 	inline void _setCoefficients()
// 	{
// 	}

	inline void _direction()
	{
		_dir = 1;
		if (_p0 > _pf)
		{
			_dir = -1;

			_p0 *= _dir;
			_pf *= _dir;
			_v0 *= _dir;
			_vf *= _dir;
		}			
	}

private:
	// time parameters
	double _Tj1;
	double _Ta;	// acceleration _duration
	double _Tv; // constant velocity _duration
	double _Tj2;
	double _Td;	// deceleration _duration

	double _vlim;

	double _jd; // desired jerk

	int _dir;	
};

//  ---------------------- Doxygen info ----------------------
//! \class _OnlineDoubleSInterpolator
//!
//! \brief
//! This implements a double-S interpolator class to be used for the interface of NRMKFoundation library
//!
//! \details 
//! This class implements the double-S interpolator class 
//! for interpolating a scalar variable for a given pair of initial 
//! and final position and velocity. 
//! The default interpolator when the boundary conditions are not specified 
//! is defined by two _duration ratios of the first and last acceleration (or deceleration) phase.
//! Given the boundary conditions, the assigned maximum acceleration will be achieved if feasible.
//! When the maximum velocity cannot be respected, the time _duration is modified accordingly.
//! That means the total _duration may change.
//!
//! \sa _PolynomialInterpolator
//  ----------------------------------------------------------
class _OnlineDoubleSInterpolator : public _PolynomialInterpolator<POLYNOMIAL_INTERPOLATOR_DOUBLE_S, _OnlineDoubleSInterpolator>
{
public:
	inline void setInitialTraj(double t0, double p0, double v0 = 0, double a0 = 0, double = 0)
	{
		_t0 = t0;
		_p0 = p0;
		_v0 = v0;
		_a0 = a0;
	}

	//! \details This sets the final conditions of the default double-S interpolator.
	//!
	//! \note 
	//! After calling this function, users can call setBoundaryCond() to set maximum velocity/acceleration/jerk condition 
	//  to define the time constants of the double-S trajectory. If this is intended, the first argument is modified. 
	//! Alternatively, setDurations() can be called to keep the final time by imposing maximum conditions internally.
	//! 
	inline void setTargetTraj(double tf, double pf, double vf, double af = 0, double = 0)
	{
		_tf = tf;	
		_pf = pf;
		_vf = vf;
		_af = af;

		// determine direction
		_direction();

		_p_next = _p0;
		_v_next = _v0;
		_a_next = _a0;

		_phase1 = 10;	// indicating start of phase1
		_phase2 = -10;

// #ifdef __RTX__
// 		_duration = DBL_MAX;
// #else
		_duration = std::numeric_limits<double>::infinity();
//#endif
	}

	//! \details This sets the maximum velocity/acceleration/jerk for the default double-S interpolator.
	//!
	//! \note The limit is symmetric in the sense that vmin = -vmax, amin = -amin, jmin = -jmax.
	//! In terms of initial and final trajectory conditions as well as these boundary conditions
	//! the time constants defining a complete double-S trajectory are determined. 
	//! 
	inline void setBoundaryCond(double vmax, double amax, double jmax)
	{
		if (vmax > 0)
			_vmax = vmax;
		else 
			_vmin = vmax;

		if (amax > 0)
			_amax = amax;
		else 
			_amin = amax;

		if (jmax > 0)
			_jmax = jmax;
		else 
			_jmin = jmax;
	}

	//! \details sets the duration
	inline void setDuration(double duration)
	{
	}

	//! \details This calculates the desired trajectory at a given time.
	inline void traj(double t, double & pd, double & vd, double & ad)
	{
		// modify the boundary conditions according to the direction
		_applyDirection();

		// p, v, a are variables corresponding to the case _p0 < _p1, i.e. _dir = 1.
		double a = _a_next;
		double v = _v_next;
		double p = _p_next;

		// modify the direction
		if (_dir == 1)
		{
			ad = a;
			vd = v;
			pd = p;
		}
		else if (_dir == -1)
		{
			ad = -a;
			vd = -v;
			pd = -p;
		}

// 		if (_dir == 1)
// 		{
// 			a = ad = _a_next;
// 			v = vd = _v_next;
// 			p = pd = _p_next;
// 		}
// 		else if (_dir == -1)
// 		{
// 			p = -pd;
// 			v = -vd;
// 			a = -ad;
// 
// 			ad = -_a_next;
// 			vd = -_v_next;
// 			pd = -_p_next;
// 		}			

		// compute jerk and manage phases
		if (_phase1 > 0)
			_jd = _jerk_phase1(v, a);

		if (_phase1 == 0)
		{
			if (_phase2 < 0)
			{
				_construct_phase2(p, v, a);
				if (_phase2 == 0)
				{
					_tphase2 = t;
					_duration = _tphase2 - _t0 + _Td;
				}
			}

			if (_phase2 == -1)
				_jd = 0;

			else if (_phase2 == 0)
				_jd = _jerk_phase2(t);
		}

		t += _delT;

		if (_phase2 == 0 && t > _Td + _tphase2)
		{
			_p_next = _pf;
			_v_next = 0;
			_a_next = 0;
			_jd = 0;
		}
		else
		{
			_a_next = a + _delT*_jd;
			_v_next = v + _delT/2*(a + _a_next);
			_p_next = p + _delT/2*(v + _v_next);
		}

		if (_dir == -1)
			_jd *= -1;		
	}

	inline double jerk() const { return _jd; }

	
private:
	inline double _jerk_phase1(double vd, double ad)
	{
		double v = vd - ad*ad/(2*_jmin);

		if (v < _vmax)
		{
			if (ad < _amax)
			{
				_phase1 = 3;

				return _jmax;
			}
			else
			{
				_phase1 = 2;
				return 0;
			}
		}
		else
		{
			if (ad > 0)
			{
				_phase1 = 1;

				return _jmin;
			}
			else
			{
				_phase1 = 0;

				return 0;
			}
		}
	}

	inline double _jerk_phase2(double t)
	{
		t -= _tphase2;

		if (t <= _Tj2a)
			return _jmin;

		else
		{
			if (t < _Td - _Tj2b)
				return 0;

			else if (t <= _Td)
				return _jmax;

			else 
				return 0;
		}
	}

	inline void _construct_phase2(double pd, double vd, double ad)
	{
		_Tj2a = (_amin - ad)/_jmin;
		_Tj2b = (_af - _amin)/_jmax;
		_Td = (_vf - vd)/_amin + _Tj2a/2*(1 - ad/_amin) + _Tj2b/2*(1 - _af/_amin);

		if (_Td < _Tj2a + _Tj2b)
		{
			double temp = ::sqrt((_jmax - _jmin)*(ad*ad*_jmax - _jmin*(_af*_af + 2*_jmax*(vd - _vf))));
			temp /= (_jmax - _jmin);

			_Tj2a = -1/_jmin*(ad + temp);
			_Tj2b = 1/_jmax*(_af + temp);  
			_Td = _Tj2a + _Tj2b;
		}

		double Td2 = _Td*_Td;
		double h = ad*Td2/2 + (_jmin*_Tj2a*(3*Td2 + _Tj2a*(_Tj2a - 3*_Td)) + _jmax*_Tj2b*_Tj2b*_Tj2b)/6 + _Td*vd;

		if (h < _pf - pd)
			_phase2 = -1;
		else
			_phase2 = 0;
	}

	inline void _direction()
	{
		_dir = 1;
		if (_p0 > _pf)
		{
			_dir = -1;

			_p0 *= _dir;
			_pf *= _dir;
			_v0 *= _dir;
			_vf *= _dir;
			_a0 *= _dir;
			_af *= _dir;
		}			
	}

	inline void _applyDirection()
	{
		// there may change boundary conditions online.
		// if not, de-comment the followings.
		static bool applied = false;
		if (applied)
			return;

		if (_dir == -1)
		{
			std::swap(_vmax, _vmin);
			std::swap(_amax, _amin);
			std::swap(_jmax, _jmin);

			_vmax *= -1;
			_vmin *= -1;
			_amax *= -1;
			_amin *= -1;
			_jmax *= -1;
			_jmin *= -1;
		}

		applied = true;
	}

private:

	double _p_next;
	double _v_next;
	double _a_next;

	// time parameters	
	double _Tj2a;
	double _Tj2b;
	double _Td;	// deceleration _duration

	double _tphase2;

	int	_phase1;
	int _phase2;

	double _jd; // desired jerk

	int _dir;	
};

//  ---------------------- Doxygen info ----------------------
//! \class _ClosedLoopTrapezoidalInterpolator
//!
//! \brief
//! This implements a trapezoidal interpolator (a.k.a. linear interpolator 
//! with parabolic blends) class to be used for the interface of NRMKFoundation library
//!
//! \details 
//! This class implements the trapezoidal interpolator class 
//! for interpolating a scalar variable for a given pair of initial 
//! and final position and velocity. 
//! The default interpolator when the boundary conditions are not specified 
//! is defined by two _duration ratios of the first and last acceleration (or deceleration) phase.
//! Given the boundary conditions, the assigned maximum acceleration will be achieved if feasible.
//! When the maximum velocity cannot be respected, the time _duration is modified accordingly.
//! That means the total _duration may change.
//!
//! \sa _PolynomialInterpolator
//  ----------------------------------------------------------
class _ClosedLoopTrapezoidalInterpolator : public _PolynomialInterpolator<POLYNOMIAL_INTERPOLATOR_LINEAR_PARABOLIC_BLEND, _ClosedLoopTrapezoidalInterpolator>
{
public:
	inline void setInitialTraj(double t0, double p0, double v0 = 0, double = 0, double = 0)
	{
		_t0 = t0;
		_p0 = p0;
		_v0 = v0;

		_p_next = _p0;
		_v_next = _v0;

		_r = _p0;
		_rdot = _v0;

		_repetition = 1;
	}

	//! \details This sets reference trajectory for the filter.
	//! 
	inline void setTargetTraj(double tf, double pf, double vf = 0, double = 0, double = 0)
	{
		_r = pf;
		_rdot = vf;
	}

	//! \details This sets the maximum velocity and acceleration for the default trapezoidal interpolator.
	//!
	//! \note In general, they can modify the segment durations. 
	//! Depending on the maximum acceleration, the trapezoidal trajectory can not be realized.
	//! feasible(amax) can detect this feasibility. If not feasible, the boundary conditions are neglected.
	//! The _duration may not be consistent with the maximum velocity condition. 
	//! In this case the _duration time is modified to attain the boundary conditions.
	//! 
	inline void setBoundaryCond(double vmax, double amax, double = 0)
	{
		_vmax = ::fabs(vmax);
		_amax = ::fabs(amax);
	}

	//! \details sets the duration
	inline void setDuration(double duration)
	{
	}

	//! \details This calculates the desired trajectory at a given time.
	inline void traj(double t, double & pd, double & vd, double & ad)
	{
		/// FIXED @20160323
		pd = _p_next;
		vd = _v_next;

		ad = _filter(_r, _rdot, pd, vd);

		_generate(ad, _p_next, _v_next);

		for (unsigned int k = 1; k < _repetition; k++)
		{
			double pd = _p_next;
			double vd = _v_next;

			double ad = _filter(_r, _rdot, pd, vd);

			_generate(ad, _p_next, _v_next);
		}
	}

	//! \details This sets the number of repetitive calls in traj(), when the trajectory 
	//!  generation period _delT is different from its call. 
	void setRepetition(int repetition)
	{
		_repetition = repetition;
	}

private:
	inline void _generate(double u, double & p, double &pdot)
	{
		double v_pre = pdot;

		pdot += _delT*u;
		p += _delT/2*(pdot + v_pre);
	}

	// generate input to the filter
	inline double _filter(double r, double rdot, double p, double pdot)
	{
		double e = (p - r)/_amax;
		double edot = (pdot - rdot)/_amax;

		double z = 1/_delT*(e/_delT + edot/2);
		double zdot = edot/_delT;

		double m = ::floor((1 + ::sqrt(1 + 8*::fabs(z)))/2);
		double sigma = zdot + z/m + (m - 1)/2.0*_sign(z);

		return -_amax*_sat(sigma)*(1.0 + _sign(pdot*_sign(sigma) + _vmax - _delT*_amax))/2.0;
	}

private:
	double _r;
	double _rdot;

	double _p_next;
	double _v_next;

	unsigned int _repetition;
};

//  ---------------------- Doxygen info ----------------------
//! \class _ClosedLoopDoubleSInterpolator
//!
//! \brief
//! This implements a trapezoidal interpolator (a.k.a. linear interpolator 
//! with parabolic blends) class to be used for the interface of NRMKFoundation library
//!
//! \details 
//! This class implements the double-S interpolator class 
//! for interpolating a scalar variable for a given pair of initial 
//! and final position and velocity. 
//! The default interpolator when the boundary conditions are not specified 
//! is defined by two _duration ratios of the first and last acceleration (or deceleration) phase.
//! Given the boundary conditions, the assigned maximum acceleration will be achieved if feasible.
//! When the maximum velocity cannot be respected, the time _duration is modified accordingly.
//! That means the total _duration may change.
//!
//! \sa _PolynomialInterpolator
//  ----------------------------------------------------------
class _ClosedLoopDoubleSInterpolator : public _PolynomialInterpolator<POLYNOMIAL_INTERPOLATOR_DOUBLE_S, _ClosedLoopDoubleSInterpolator>
{
public:
	inline void setInitialTraj(double t0, double p0, double v0 = 0, double a0 = 0, double = 0)
	{
		_t0 = t0;
		_p0 = p0;
		_v0 = v0;
		_a0 = a0;

		_p_next = _p0;
		_v_next = _v0;
		_a_next = _a0;

		_r = _p0;
		_rdot = _v0;
		_rddot = _a0;

		_repetition = 1;
	}
	
	//! \details This sets the maximum velocity and acceleration for the default trapezoidal interpolator.
	//!
	//! \note In general, they can modify the segment durations. 
	//! Depending on the maximum acceleration, the trapezoidal trajectory can not be realized.
	//! feasible(amax) can detect this feasibility. If not feasible, the boundary conditions are neglected.
	//! The _duration may not be consistent with the maximum velocity condition. 
	//! In this case the _duration time is modified to attain the boundary conditions.
	//! 
	inline void setBoundaryCond(double vmax, double amax, double jmax)
	{
		if (vmax > 0)
			_vmax = vmax;
		else 
			_vmin = vmax;

		if (amax > 0)
			_amax = amax;
		else 
			_amin = amax;

		_jmax = ::fabs(jmax);
	}

	//! \details sets the duration
	inline void setDuration(double duration)
	{
	}

	//! \details This calculates the desired trajectory at a given time.
	inline void traj(double t, double & pd, double & vd, double & ad)
	{
		/// FIXED @20160323
		pd = _p_next;
		vd = _v_next;
		ad = _a_next;

		_jd = _filter(_r, _rdot, _rddot, pd, vd, ad);

		_generate(_jd, _p_next, _v_next, _a_next);

		for (unsigned int k = 0; k < _repetition; k++)
		{
			double pd = _p_next;
			double vd = _v_next;
			double ad = _a_next;

			_jd = _filter(_r, _rdot, _rddot, pd, vd, ad);

			_generate(_jd, _p_next, _v_next, _a_next);
		}
	}
	
	//! \details This sets the number of repetitive calls in traj(), when the trajectory 
	//!  generation period _delT is different from its call. 
	void setRepetition(int repetition)
	{
		_repetition = repetition;
	}

	inline double jerk() const { return _jd; }

private:
	inline void _generate(double u, double & p, double &v, double &a)
	{
		double a_pre = a;
		double v_pre = v;

		a += _delT*u;
		v += _delT/2*(a + a_pre);
		p += _delT/2*(v + v_pre);
	}

	// generate input to the filter
	inline double _filter(double r, double rdot, double rddot, double p, double pdot, double pddot)
	{
		double e = (p - r)/_jmax;
		double edot = (pdot - rdot)/_jmax;
		double eddot = (pddot - rddot)/_jmax;

		double edot_min = (_vmin - rdot)/_jmax;
		double edot_max = (_vmax - rdot)/_jmax;

		double eddot_min = (_amin - rddot)/_jmax;
		double eddot_max = (_amax - rddot)/_jmax;

		double delta = edot + eddot*::fabs(eddot)/2;
		int sdelta = _sign(delta);

		double eddot2 = eddot*eddot;
		double eddot3 = eddot2*eddot;
		double temp = eddot2 + 2*edot*sdelta;

		double sigma = e + edot*eddot*sdelta - eddot3/6*(1 - 3*::abs(sdelta)) + sdelta/4.0*::sqrt(2*temp*temp*temp);

		//double common1 = e - eddot_max*(eddot2 - 2*edot)/4 - (eddot2 - 2*edot)^2/(8*eddot_max);

		temp = eddot2 - 2*edot;
		//double common1 = e - (eddot2 - 2*edot)/4*(eddot_max + (eddot2 - 2*edot)/(2*eddot_max));
		double common1 = e - temp/4*(eddot_max + temp/(2*eddot_max));
		double common2 = eddot*(3*edot - eddot2)/3;

// 		double nu_plus = e - eddot_max*(eddot*eddot - 2*edot)/4 - (eddot*eddot - 2*edot)^2/(8*eddot_max) - eddot*(3*edot - eddot*eddot)/3;
		double nu_plus = common1 - common2;

		//common1 = e - eddot_min*(eddot*eddot + 2*edot)/4 - (eddot*eddot + 2*edot)^2/(8*eddot_min);
		temp = eddot2 + 2*edot;
		common1 = e - temp/4*(eddot_min + temp/(2*eddot_min));

		common2 = eddot*(3*edot + eddot2)/3;

		//double nu_minus = e - eddot_min*(eddot*eddot + 2*edot)/4 - (eddot*eddot + 2*edot)^2/(8*eddot_min) + eddot*(3*edot + eddot*eddot)/3;
		double nu_minus = common1 + common2;

		double Sigma = sigma;

		if (eddot <= eddot_max && edot <= eddot2/2 - eddot_max*eddot_max)
			Sigma = nu_plus;
		
		if (eddot >= eddot_min && edot >= eddot_min*eddot_min - eddot2/2) 
			Sigma = nu_minus;
		
		double uc = -_jmax*_sign(Sigma + (1 - ::abs(_sign(Sigma)))*(delta + (1 - ::abs(sdelta))*eddot));

		double uv_min = _u_v(edot_min, edot, eddot, eddot_min, eddot_max);
		double uv_max = _u_v(edot_max, edot, eddot, eddot_min, eddot_max);

		return _max( uv_min, _min( uc, uv_max) );
	}
	
	inline double _delta_v(double edot, double eddot, double v)
	{
		return eddot*::fabs(eddot) + 2*(edot - v); 
	}

	inline double _u_cv(double edot, double eddot, double v)
	{
		double deltav = _delta_v(edot, eddot, v);
		
		return -_jmax*_sign(deltav + (1 - ::abs(_sign(deltav)))*eddot);
	}

	inline double _u_a(double eddot, double a)
	{
		return -_jmax*_sign(eddot - a);
	}

	inline double _u_v(double v, double edot, double eddot, double eddot_min, double eddot_max)
	{
		return _max( _u_a(eddot, eddot_min), _min( _u_cv(edot, eddot, v), _u_a(eddot, eddot_max)) );
	}

private:
	double _r;
	double _rdot;
	double _rddot;

	double _p_next;
	double _v_next;
	double _a_next;
	
	double _jd;	// jerk

	unsigned int _repetition;
};

} // namespace internal

} // namespace NRMKFoundation
