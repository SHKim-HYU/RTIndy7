//  ---------------------- Doxygen info ----------------------
//! \file TrigonometricEquations.h
//!
//! \brief
//! Header file for the class TrigonometricEquations (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements an analytical algorithm to solve trigonometric equations 
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
//! \date May 2014
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

#pragma once

//#include <iostream>

#include <math.h>

namespace NRMKFoundation
{
	// p(t) = amp*sin(2*M_PI*freq*(t - t0) - phase) + offset  
	class SinusoidalTrajectory
	{
	public:
		SinusoidalTrajectory() : _amp(1), _freq(1), _t0(0), _phase(0), _offset(0)
		{
		}

		void setParameters(double amp, double freq, double phase, double offset)
		{
			_amp = amp;
			_freq = freq;
			_phase = phase;
			_offset = offset;

			_f = 2*M_PI*_freq;
		}

		void setTimeOffset(double t0)
		{
			_t0 = t0;
		}

		void traj(double t, double & p, double & v, double & a)
		{
			double s = _f*(t - _t0) - _phase;
			double fa = _f*_amp;

			p = _amp*sin(s) + _offset; 
			v = fa*cos(s);
			a = -_f*fa*sin(s);
		}

	private:
		double _amp; 
		double _freq;
		double _t0;
		double _phase;
		double _offset;

		double _f;
	};

	class TrigonometricEquations
	{
	public:
		typedef std::complex<double> Complex; 

	public:
		// solve a * cos(q) + b * sin(q) = c
		static int PhaseShiftEquation(double a, double b, double c, double * q)
		{
			int num_roots = 0;

			double r = ::sqrt(a*a + b*b);
			
			if (c > r || -c > r)
			{
				return num_roots;
			}
			else 
			{
				double alpha = ::atan2(b, a);

				if (c == r)
				{
					q[0] = alpha;
					num_roots = 1;
				}
				else if (c == -r)
				{
					q[0] = alpha + M_PI;
					num_roots = 1;
				}
				else
				{
					double theta = ::acos(c/r);

					q[0] = alpha + theta;
					q[1] = alpha - theta;

					num_roots = 2;
				}
				
// 				std::cout << "error = " << a*::cos(q[0]) + b*::sin(q[0]) - c << std::endl;
// 				std::cout << "error = " << a*::cos(q[1]) + b*::sin(q[1]) - c << std::endl;

				return num_roots;
			}
		}

		// solve l1 * cos(q1) + l2 * cos(q1 + q2) = x
		//           l1 * sin(q1) + l2 * sin(q1 + q2) = y
		static int Planar2DEquation(double l1, double l2, double x, double y, double * q1, double * q2)
		{
			assert(l1 > 0);
			assert(l2 > 0);

			int num_roots = 0;

			double r2 = x*x + y*y;
			double lmax = l1*l1 + l2*l2;

			double c2 = (r2 - lmax)/(2*l1*l2);

			if (c2 > 1 || c2 < -1)
			{
				return num_roots;
			}
			else
			{
				if (c2 == 1)
				{
					num_roots = 1;

					q2[0] = 0;
					q1[0] = ::atan2(y, x);
				}
				else if (c2 == -1)
				{
					num_roots = 1;

					q2[0] = M_PI;
					q1[0] = ::atan2(y, x);
				}
				else
				{
					num_roots = 2;					

					double k0 = l2*c2;
					double k1 = l1 + k0; 
					double det = lmax + 2*l1*k0;

					// First solution
					q2[0] = ::acos(c2);

					double s2 = sin(q2[0]);
					double k2 = l2*s2;
					
					double c1 = (k1*x + k2*y)/det;
					double s1 = (-k2*x + k1*y)/det;

					q1[0] = ::atan2(s1, c1);

					// Second solution
					q2[1] = -q2[0]; 

					c1 = (k1*x - k2*y)/det;
					s1 = (k2*x + k1*y)/det;

					q1[1] = ::atan2(s1, c1);
				}

				return num_roots;
			}
		}

		// solve a11 * cos(q) +  a12*sin(q) = b1
		//           a21 * cos(q) + a22*sin(q) = b2
		static int LinearEquation(double a11, double a12, double a21, double a22, double b1, double b2, double *q)
		{		
			double det = a11*a22 - a12*a21;

			if (det == 0)
				return 0;

			double c = (a22*b1 - a12*b2)/det;
			double s = (-a21*b1 + a11*b2)/det;

// 			if (c < -1 ||  c > 1)
// 				return 0;
// 
// 			if (s < -1 ||  s > 1)
// 				return 0;

			*q = ::atan2(s, c);

			return 1;
		}
	};

	// Convert the angle withing a single turn range, i.e. [0, 2*pi)
	inline double SingleTurnAngle(double q)
	{
		while (q < 0)
			q += 2*M_PI;

		while (q >= 2*M_PI)
			q -= 2*M_PI;

		/*
		if (q < 0)
		{
			while (q < 0)
				q += 2*M_PI;
		}
		else if (q >= 2*M_PI)
		{
			while (q >= 2*M_PI)
				q -= 2*M_PI;
		}
		*/

		return q;
	}

	// choose closer angle between q1 and q2 taking into account of +/- 2*M_PI offset
	inline double CloserAngle(double q1, double q2, double q)
	{
		double d[2];
		d[0] = NRMKFoundation::SingleTurnAngle(q - q1);
		d[1] = NRMKFoundation::SingleTurnAngle(q - q2);

		if (d[0] < d[1])
		{
			// to deal with numerical error
			if (d[0] < (2*M_PI - d[1]))
				return NRMKFoundation::SingleTurnAngle(q1);
			else
				return NRMKFoundation::SingleTurnAngle(q2);
		}
		else
		{
			if (d[1] < (2*M_PI - d[0]))
				return NRMKFoundation::SingleTurnAngle(q2);
			else
				return NRMKFoundation::SingleTurnAngle(q1);
		}
	}

} // namespace NRMKFoundation