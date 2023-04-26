//  ---------------------- Doxygen info ----------------------
//! \file MultipointAlgorithm.h
//!
//! \brief
//! Header file for the class _MultipointInterpolator (Internal API of the NRMKFoundation Libraries)
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

#include "PolynomialAlgorithm.h"
#include <TrajectoryDataList.h>

namespace NRMKFoundation
{

namespace internal
{

//  ---------------------- Doxygen info ----------------------
//! \class _MultipointInterpolator
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
//! \tparam _NUM_SEGMENTS is the number of trajectory data (except initial one).
//! \tparam InterpolatorType is the derived class of this class.
//!
//! \sa _CSplineInterpolator
//! \sa _MultiSegmentInterpolator
//  ----------------------------------------------------------
template<PolynomialInterpolatorMode _TYPE, int _NUM_SEGMENTS, typename InterpolatorType>
class _MultipointInterpolator : public _PolynomialInterpolator<_TYPE, _MultipointInterpolator<_TYPE, _NUM_SEGMENTS, InterpolatorType> >
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \enum
	//! 
	//! \brief Provides compile-time constants of the multi-point interpolator
	//  ----------------------------------------------------------
	enum 
	{	
		//! \details The number of trajectory point data
		NUM_SEGMENTS = _NUM_SEGMENTS,
	};

	//! \returns 
	//	a reference to the derived object 
	inline InterpolatorType& derived() { return *static_cast<InterpolatorType*>(this); }
	//! \returns 
	//! a const reference to the derived object 
	inline const InterpolatorType& derived() const { return *static_cast<const InterpolatorType*>(this); }

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//!  sets the initial condition of the interpolated trajectory
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
		_t[0] = t0;
		_p[0] = p0;
		_v[0] = v0;
		_a[0] = a0;

		//_a0 = a0;
		//_a[0] = a0;
		//_j[0] = j0;

		_cur_seg = 0;
		_num_segments = 0;
		//_num_segments = NUM_SEGMENTS;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//!  sets the target condition, including intermediate one, of the interpolated trajectory
	//!
	//! \details
	//! This should be called after setTargetTraj() before calling traj(). 
	//! Note that the first argument tf should be greater than the one used in the previous call to this function. 
	//! 
	//! \param tf Final time
	//! \param pf Final position
	//! \param vf Final velocity (default value = 0)
	//! \param af Final acceleration (default value = 0)
	//! \param jf Final jerk (default value = 0)
	//  ----------------------------------------------------------
	inline void setTargetTraj(double tf, double pf, double vf = 0, double af = 0, double jf = 0)
	{
		if (_num_segments > NUM_SEGMENTS)
			return;

// 		// ADDED@20150712
// 		if (_delT > 0)
// 			_tick[_num_segments] = (tf - _t[_num_segments])/_delT;
// 		else
// 			_tick[_num_segments] = 0;
		
		// k should be between 1 to NUM_SEGMENTS
		_num_segments++;

		_t[_num_segments] = tf;
		_p[_num_segments] = pf;
		_v[_num_segments] = vf;
		_a[_num_segments] = af;		

		derived().determine();

// 		if (_num_segments == NUM_SEGMENTS)
// 			finalize();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//!  finalize the trajectory specification
	//!
	//! \details
	//! This should not be called explicitly unless the algorithm supports this capability. 
	//! For example, _MultiSegmentInterpolator supports this.
	//  ----------------------------------------------------------
// 	inline void finalize()
// 	{
// 		if (_cur_seg < NUM_SEGMENTS)
// 		{
// 			_num_segments = _cur_seg;		
// 			//derived().determine();
// 		}
// 
// 		_cur_seg = 0;
// 	}

	//! \details Boundary conditions are not respected.
	inline void setBoundaryCond(double = 0, double = 0, double = 0)
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//!  generates the desired trajectory at a given time. 
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
	//! sets the duration of the interpolated trajectory
	//! 
	//! \param duration duration
	//  ----------------------------------------------------------
	inline void setDuration(double duration)
	{
		derived().setDuration(duration);
	}

protected:
	//  ---------------------- Doxygen info ----------------------
	//! \var double _t[NUM_SEGMENTS + 1]
	//! 
	//! \brief time data
	//! 
	//! \details _t[0] is the initial time and _t[NUM_SEGMENTS] is the final time.
	//  ----------------------------------------------------------
	double _t[NUM_SEGMENTS + 1];

	//  ---------------------- Doxygen info ----------------------
	//! \var double _p[NUM_SEGMENTS + 1]
	//! 
	//! \brief position data
	//  ----------------------------------------------------------
	double _p[NUM_SEGMENTS + 1];

	//  ---------------------- Doxygen info ----------------------
	//! \var double _v[NUM_SEGMENTS + 1]
	//! 
	//! \brief velocity data
	//  ----------------------------------------------------------
	double _v[NUM_SEGMENTS + 1];

	//  ---------------------- Doxygen info ----------------------
	//! \var double _a[NUM_SEGMENTS + 1]
	//! 
	//! \brief acceleration data
	//  ----------------------------------------------------------
	double _a[NUM_SEGMENTS + 1];

	//double _j[NUM_SEGMENTS + 1];

	//  ---------------------- Doxygen info ----------------------
	//! \var int _cur_seg
	//! 
	//! \brief cursor to detect the current segment
	//  ----------------------------------------------------------
	int _cur_seg;

	//  ---------------------- Doxygen info ----------------------
	//! \var int _num_segments
	//! 
	//! \brief actual number of segments
	//  ----------------------------------------------------------
	int _num_segments;

 	// ADDED@20150712
// 	//  ---------------------- Doxygen info ----------------------
// 	//! \var int _tick[NUM_SEGMENTS]
// 	//! 
// 	//! \brief number of ticks to move to the next segment
// 	//! 
// 	//! \details setPeriod() should be called before using _tick[]
// 	//  ----------------------------------------------------------
// 	int _tick[NUM_SEGMENTS];
// 
 	 using _PolynomialInterpolator<_TYPE, _MultipointInterpolator<_TYPE, _NUM_SEGMENTS, InterpolatorType> >::_delT;
};


//  ---------------------- Doxygen info ----------------------
//! \class _MultiSegmentInterpolator
//!
//! \brief
//! This implements a multiple segment interpolator class
//! to be used for the interface of NRMKFoundation library
//!
//! \details 
//! This class implements the multiple segment interpolator class 
//! for interpolating a scalar variable for a given pair of initial 
//! and final position and velocity. 
//!
//! \sa _MultipointInterpolator
//  ----------------------------------------------------------
template<PolynomialInterpolatorMode MODE, int NUM_SEGMENTS>
class _MultiSegmentInterpolator : public _MultipointInterpolator<MODE, NUM_SEGMENTS, _MultiSegmentInterpolator<MODE, NUM_SEGMENTS> >
{
public:
	//! \details This calculates the each segment trajectory.
	inline void traj(double t, double & pd, double & vd, double & ad)
	{
		// FIXED@20150712
// 		if (t > _t[_cur_seg + 1])
// 				if (tick >= _ticks[_cur_seg])
// 				{
// 					_cur_seg++;
// 					tick = 0;
// 				}
// 				else
// 					tick++;

		if ((_cur_seg < _num_segments - 1) && (t >= _t[_cur_seg + 1] - _delT*0.9))	
			_cur_seg++;
		
		_segment[_cur_seg].traj(t, pd, vd, ad);
	}

	//! \details This calculates the coefficients of the each segment trajectory.
	inline void determine()
	{
// 		if (_cur_seg == _num_segments)		
// 			return;

		int k = _num_segments - 1;

		_segment[k].setInitialTraj(_t[k], _p[k], _v[k], _a[k]);
		_segment[k].setTargetTraj(_t[k+1], _p[k+1], _v[k+1], _a[k+1]);

		/*
		for (int k = 0; k < NUM_SEGMENTS; k++)
		{
			_segment[k].setInitialTraj(_t[k], _p[k], _v[k], _a[k]);
			_segment[k].setTargetTraj(_t[k+1], _p[k+1], _v[k+1], _a[k+1]);
		}
		*/
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the duration of the interpolated trajectory
	//! 
	//! \param duration duration
	//  ----------------------------------------------------------
	inline void setDuration(double duration)
	{
	}

	int getCurSeg()
	{
		return _cur_seg;
	}
	
private:	
	typename _PolynomialInterpolatorType<MODE>::Type _segment[NUM_SEGMENTS];
	
	typedef _MultipointInterpolator<MODE, NUM_SEGMENTS, _MultiSegmentInterpolator<MODE, NUM_SEGMENTS> > _Base;
	
	using _Base::_t;
	using _Base::_p;
	using _Base::_v;
	using _Base::_a;
	using _Base::_cur_seg;
	using _Base::_num_segments;
	using _Base::_delT;
};

//  ---------------------- Doxygen info ----------------------
//! \class _CSplineInterpolator
//!
//! \brief
//! This implements a cubic spline interpolator class
//! to be used for the interface of NRMKFoundation library
//!
//! \details 
//! This class implements the cubic spline interpolator class 
//! for interpolating a scalar variable for a given pair of initial 
//! and final position and velocity. 
//!
//! \sa _MultipointInterpolator
//  ----------------------------------------------------------
template<int NUM_SEGMENTS>
class _CSplineInterpolator : public _MultipointInterpolator<POLYNOMIAL_INTERPOLATOR_CSPLINE, NUM_SEGMENTS, _CSplineInterpolator<NUM_SEGMENTS> >
{
	//EIGEN_STATIC_ASSERT(NUM_SEGMENTS == DynamicSize, YOU_SHOULD_FIX_THE_NUMBER_OF_DATA);

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Constructor of the class with mode
	//!
	//! \param mode the mode to fix the trajectory. 
	//!		Choose one from CSplineMode.
	//  ----------------------------------------------------------
	_CSplineInterpolator(CSplineMode mode = NATURAL_BOUNDARY_CONDITIONS) : _mode(mode)
	{
	}

	inline void setMode(CSplineMode mode)
	{
		_mode = mode;
	}

	//! \details This calculates the cspline trajectory.
	inline void traj(double t, double & pd, double & vd, double & ad)
	{
		// FIXED@20150712
		//if (t > _t[_cur_seg + 1])
		if ((_cur_seg < _num_segments - 1) && (t >= _t[_cur_seg + 1] - _delT*0.9))	
			_cur_seg++;

		double T[_ORDER + 1] = { 1, t - _t[_cur_seg], 0 };
		for (int k = 2; k < _ORDER + 1; k++)
			T[k] = T[1]*T[k-1];

		double const * const a = _c[_cur_seg];

		pd = a[0] + a[1]*T[1] + a[2]*T[2] + a[3]*T[3];
		vd = a[1] + 2*a[2]*T[1] + 3*a[3]*T[2];
		ad = 2*a[2] + 6*a[3]*T[1];		

// 		for (int k = 4; k < ORDER + 1; k++)
// 		{
// 			pd += a[k]*T[k];
// 			vd += k*a[k]*T[k-1];
// 			ad += k*(k-1)*a[k]*T[k-2];
// 		}
	}

	//! \details This calculates the coefficients of the cspline trajectory.
	inline void determine()
	{
		if (_cur_seg < NUM_SEGMENTS)
			return;
		
		// make common part of the acceleration coefficient matrix 
		Eigen::Matrix<double, NUM_SEGMENTS + 1, NUM_SEGMENTS + 1> A(Eigen::Matrix<double, NUM_SEGMENTS + 1, NUM_SEGMENTS + 1>::Zero());
		Eigen::Matrix<double, NUM_SEGMENTS + 1, 1> c;

		Eigen::Matrix<double, NUM_SEGMENTS, 1> h;	// time step
		Eigen::Matrix<double, NUM_SEGMENTS, 1> y;	// position step
		for (int k = 0; k < NUM_SEGMENTS; k++)
		{
			h[k] = _t[k+1] - _t[k];
			y[k] = _p[k+1] - _p[k];
		}
		
		for (int k = 1; k < NUM_SEGMENTS; k++)
		{
			A(k, k-1) = h[k-1];
			A(k, k) = 2*(h[k-1] + h[k]);
			A(k, k+1) = h[k];

			c[k] = 6*(y[k]/h[k] - y[k-1]/h[k-1]);
		}

		// boundary-condition specific
		switch (_mode)
		{
		case NATURAL_BOUNDARY_CONDITIONS:
			A(0, 0) = A(NUM_SEGMENTS, NUM_SEGMENTS) = 1;
			c[0] = c[NUM_SEGMENTS] = 0;

			break;

		case ASSIGNED_BOUNDARY_ACCELERATIONS:
			A(0, 0) = A(NUM_SEGMENTS, NUM_SEGMENTS) = 1;
			c[0] = _a[0]; //_a0;
			c[NUM_SEGMENTS] = _a[NUM_SEGMENTS]; //_af;

			break;

		case PARABOLIC_RUNOUT:
			A(0, 0) = A(NUM_SEGMENTS, NUM_SEGMENTS) = 1;
			A(0, 1) = A(NUM_SEGMENTS, NUM_SEGMENTS - 1) = -1;
			c[0] = c[NUM_SEGMENTS] = 0;

			break;

		case ZERO_SLOPE:
			A(0, 0) = 2*h[0]; 
			A(0, 1) = h[0];
			A(NUM_SEGMENTS, NUM_SEGMENTS) = 2*h[NUM_SEGMENTS - 1]; 
			A(NUM_SEGMENTS, NUM_SEGMENTS - 1) = h[NUM_SEGMENTS - 1]; 

			c[0] = 6*y[0]/h[0];
			c[NUM_SEGMENTS] = -6*y[NUM_SEGMENTS - 1]/h[NUM_SEGMENTS - 1];

			break;

		case ASSIGNED_BOUNDARY_VELOCITIES:

			A(0, 0) = 2*h[0]; 
			A(0, 1) = h[0];
			A(NUM_SEGMENTS, NUM_SEGMENTS) = 2*h[NUM_SEGMENTS - 1]; 
			A(NUM_SEGMENTS, NUM_SEGMENTS - 1) = h[NUM_SEGMENTS - 1]; 

			c[0] = 6*(y[0]/h[0] - _v[0]);
			c[NUM_SEGMENTS] = 6*(_v[NUM_SEGMENTS] - y[NUM_SEGMENTS - 1]/h[NUM_SEGMENTS - 1]);

			break;


		case CLOSED_LOOP:
			A(0, 0) = 2*h[0]; 
			A(0, 1) = h[0];
			A(0, NUM_SEGMENTS - 1) = h[NUM_SEGMENTS - 1]; 
			A(0, NUM_SEGMENTS) = 2*h[NUM_SEGMENTS - 1];

			A(NUM_SEGMENTS, NUM_SEGMENTS) = 1;
			A(NUM_SEGMENTS, 0) = -1;

			c[0] = 6*(y[0]/h[0] - y[NUM_SEGMENTS - 1]/h[NUM_SEGMENTS - 1]);
			c[NUM_SEGMENTS] = 0;

			break;
		}

		// solve acceleration at the nodes
		if (_mode != CLOSED_LOOP)
			_solveTridiagonal(A, c);

		else
		{
			//_solveCyclicTridiagonal(A, c);
			c = A.colPivHouseholderQr().solve(c);
		}

		// compute the coefficients
		for (int k = 0; k < NUM_SEGMENTS; k++)
		{
			_c[k][0] = _p[k];
			_c[k][1] = y[k]/h[k] - (2*c[k] + c[k + 1])*h[k]/6;

			_c[k][2] = c[k]/2; 
			_c[k][3] = (c[k + 1] - c[k])/(6*h[k]);
		}
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the duration of the interpolated trajectory
	//! 
	//! \param duration duration
	//  ----------------------------------------------------------
	inline void setDuration(double duration)
	{
	}
	
private:
	void _solveTridiagonal(Eigen::Matrix<double, NUM_SEGMENTS + 1, NUM_SEGMENTS + 1> & A, Eigen::Matrix<double, NUM_SEGMENTS + 1, 1> & d)
	{
		// forward elimination
		for (int k = 1; k <= NUM_SEGMENTS; k++)
		{
			double m = A(k, k - 1)/A(k - 1, k - 1);
			
			A(k, k) -= m*A(k-1, k);
			d[k] -= m*d[k-1];
		}

		// backward substitution
		d[NUM_SEGMENTS] /= A(NUM_SEGMENTS, NUM_SEGMENTS);
		for (int k = NUM_SEGMENTS - 1; k >= 0; k--)
		{
			d[k] = (d[k] - A(k, k + 1)*d[k + 1])/A(k ,k);		
		}

#if 0
		Eigen::Matrix<double, NUM_SEGMENTS + 1, 1> b = A.diagonal();
		Eigen::Matrix<double, NUM_SEGMENTS + 1, 1> c;
		Eigen::Matrix<double, NUM_SEGMENTS + 1, 1> a;

		c.head<NUM_SEGMENTS>() = A.diagonal<1>();
		a.tail<NUM_SEGMENTS>() = A.diagonal<-1>();


		// forward elimination
		c[0] /= b[0];
		d[0] /= b[0];

		for (int k = 1; k <= NUM_SEGMENTS; k++)
		{
			double m = 1.0/(b[k] - a[k] * c[k-1]);
			c[k] *= m;

			d[k] = (d[k] - a[k]*d[k-1])*m;
		}

		// backward substitution
		for (int k = NUM_SEGMENTS - 1; k-- > 0; )
			d[k] -= c[k]*d[k + 1];		

//#if 0
		// forward elimination
		for (int k = 1; k <= NUM_SEGMENTS; k++)
		{
			b[k] = b[k]*b[k-1] - a[k] * c[k-1];
			c[k] *= b[k-1];

			d[k] = d[k]*b[k-1] - a[k]*d[k-1];
		}

		// backward substitution
		d[NUM_SEGMENTS] /= b[NUM_SEGMENTS];
		for (int k = NUM_SEGMENTS - 1; k >= 0; k--)
			d[k] -= c[k]*d[k + 1];		
#endif	

	}

// 	void _solveCyclicTridiagonal(Eigen::Matrix<double, NUM_SEGMENTS + 1, NUM_SEGMENTS + 1> & A, Eigen::Matrix<double, NUM_SEGMENTS + 1, 1> & d)
// 	{
// 		Eigen::Matrix<double, NUM_SEGMENTS + 1, 1> u(Eigen::Matrix<double, NUM_SEGMENTS + 1, 1>::Zero());
// 		u[0] = A(0, 0);
// 		u[NUM_SEGMENTS] = A(NUM_SEGMENTS, 0);
// 
// 		Eigen::Matrix<double, NUM_SEGMENTS + 1, 1> v(Eigen::Matrix<double, NUM_SEGMENTS + 1, 1>::Zero());
// 		v[0] = 1;
// 		v[NUM_SEGMENTS - 1] = A(0, NUM_SEGMENTS - 1)/u[0];
// 		v[NUM_SEGMENTS] = A(0, NUM_SEGMENTS)/u[0];
// 
// 		A(0, 0) = A(0, NUM_SEGMENTS - 1) = A(0, NUM_SEGMENTS) = 0;
// 		A(NUM_SEGMENTS, NUM_SEGMENTS) -= v[NUM_SEGMENTS]*A(NUM_SEGMENTS, 0);
// 		A(NUM_SEGMENTS, NUM_SEGMENTS - 1) -= v[NUM_SEGMENTS - 1]*A(NUM_SEGMENTS, 0);
// 		A(NUM_SEGMENTS, 0) = 0;
// 
// 		//Eigen::Matrix<double, NUM_SEGMENTS + 1, NUM_SEGMENTS + 1> Abar = A;
// 
// 		_solveTridiagonal(A, d);
// 		_solveTridiagonal(A, u);
// 
// 		d -= (v.dot(d)/(1 + v.dot(u)))*u;
// 	}

private:	
	typedef _MultipointInterpolator<POLYNOMIAL_INTERPOLATOR_CSPLINE, NUM_SEGMENTS, _CSplineInterpolator<NUM_SEGMENTS> > _Base;
	
	using _Base::_t;
	using _Base::_p;
	using _Base::_v;
	using _Base::_a;
	using _Base::_cur_seg;
	using _Base::_num_segments;
	using _Base::_delT;
	
	enum 
	{
		_ORDER = 3,
	};

	CSplineMode _mode;

	//  ---------------------- Doxygen info ----------------------
	//! \var double _a[NUM_SEGMENTS][_ORDER + 1]
	//! 
	//! \brief polynomial coefficients of every segment
	//  ----------------------------------------------------------
	double _c[NUM_SEGMENTS][_ORDER + 1];
};

}

} // namespace NRMKFoundation
