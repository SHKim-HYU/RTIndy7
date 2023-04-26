//  ---------------------- Doxygen info ----------------------
//! \file Subsys.h
//!
//! \brief
//! Header file for the class Subsys (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements an articulated multibody system class
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
//! \date December 2013
//! 
//! \version 1.5
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#include <math.h>
#include <complex>

namespace NRMKFoundation
{
	class PolynomialRoots
	{
	public:
		typedef std::complex<double> Complex; 

	public:
		static void SquareRootComplex(Complex const & a, Complex * root)
		{
			double r = ::sqrt(a.real()*a.real() + a.imag()*a.imag());
			double y = ::sqrt((r - a.real())/2);
			double x = a.imag()/(2*y);

			root[0].real(x); 
			root[0].imag(y);
			
			root[1].real(-x);
			root[1].imag(-y);
		}

		static void SquareRootComplex(Complex const & a, Complex & root)
		{
			double r = ::sqrt(a.real()*a.real() + a.imag()*a.imag());
			double y = ::sqrt((r - a.real())/2);
			double x = a.imag()/(2*y);

			root.real(x); 
			root.imag(y);
		}

		//----------------------------------------------------------------------------
		static int SolveQuadratic(double a, double b, double c, double * root, double * im = NULL)
		{
 			if (::abs(a) < 1.0e-6)
 			{
 				if (::abs(b) < 1.0e-4)
 					return 0;
 				else
 				{
 					root[0] = -c/b;
 					return 1;
 				}
 			}

			double d = b*b - 4*a*c;
			if (d > 0.0)
			{
				d = ::sqrt(d);
				root[0] = (-b + d)/(2*a); 
				root[1] = (-b - d)/(2*a); 

				return 2;
			}
			else if (d == 0)
			{
				root[0] = root[1] = -b/(2*a);
				return 2;
			}
			else
			{
				//bool findImaginaryRoots = (!im) ? false : true;
				if (im)
				{
					root[0] = root[1] =  -b/(2*a);
					im[0] = ::sqrt(-d)/(2*a);
					im[1] = -im[0];
				}
				return 0;
			}

			return false;
		}
			
		//----------------------------------------------------------------------------
		static int SolveCubic(double a, double b, double c, double d, double * root, double * im = NULL)
		{
 			if (::abs(a) < 1.0e-6)
 				return SolveQuadratic(b, c, d, root);

			bool findImaginaryRoots = (!im) ? false : true;

			double B = b/a, C = c/a, D = d/a;
			double BB = B*B;

			double f = (3*C - BB)/3;
			double fff_27 = f*f*f/27;
			double g = (2*B*BB - 9*B*C + 27*D)/27;
			double h = g*g/4 + fff_27;

			//double Q = (B*B - C*3.0)/9.0, QQQ = Q*Q*Q;
			//double R = (2.0*B*B*B - 9.0*B*C + 27.0*D)/54.0, RR = R*R;

			// 3 real roots
			if (h < 0)
			{
				double i = ::sqrt(-fff_27);	// ::sqrt(g*g/4 - h)
				double j = ::pow(i, 1.0/3);
				double k = ::acos(-g/(2*i));
				double L = -j; 
				double M = ::cos(k/3);
				double N = ::sqrt(3.0)*::sin(k/3);
				
				double P = -B/3;
				
				root[0] = 2*j*M + P; 
				root[1] = L*(M + N) + P;
				root[2] = L*(M - N) + P;

				return 3;
			}
			// three identical real root
			else if (h == 0)
			{
				if (D > 0)
					root[0] = root[1] = root[2] = (D > 0) ? -::pow(D, 1.0/3.0) : ::pow(-D, 1.0/3.0);

				return 3;
			}
			// 1 real root
			else
			{
				double sqrt_h = ::sqrt(h);
				double R = -g/2 + sqrt_h;
				double S = (R > 0) ? ::pow(R, 1.0/3) : -::pow(-R, 1.0/3);

				double T = -g/2 - sqrt_h;
				double U = (T > 0) ? ::pow(T, 1.0/3) : -::pow(-T, 1.0/3);

				double P = -B/3;
				double SU = S + U;

				root[0] = SU + P; 
				if (findImaginaryRoots)
				{
					im[0] = 0;
					root[1] = root[2] = -SU/2 + P;
					im[1] = (S - U)*::sqrt(3.0)/2;
					im[2] = -im[1];
				}

				return 1;
			}
		}

		//----------------------------------------------------------------------------
		static int SolveQuartic(double a, double b, double c, double d, double e, double *root, double *im = NULL)
		{
			// I switched to this method, and it seems to be more numerically stable.
			// http://www.gamedev.net/community/forums/topic.asp?topic_id=451048 

			// When a or (a and b) are magnitudes of order smaller than C,D,E
			// just ignore them entirely. This seems to happen because of numerical
			// inaccuracies of the line-circle algorithm. I wanted a robust solver,
			// so I put the fix here instead of there.
// 			if(a == 0.0 || ::abs(a/b) < 1.0e-5 || ::abs(a/c) < 1.0e-5 || ::abs(a/d) < 1.0e-5)
// 				return SolveCubic(b, c, d, e, root);

			bool findImaginaryRoots = (!im) ? false : true;

			double B = b/a, C = c/a, D = d/a, E = e/a;
			double BB = B*B;

			double f = -3.0*BB*0.125 + C;
			double g = BB*B*0.125 - B*C*0.5 + D;
			double h = -3*BB*BB/256.0 + C*BB/16.0 - B*D*0.25 + E;
			
			double root3[3];
			double im3[3];
			int numRoots = SolveCubic(1, f/2, (f*f - 4*h)/16, -g*g/64, root3, im3);

			if (numRoots == 3)
			{
				int k = 0;
				double smallest = root3[0];

				if (smallest > root3[1])
				{
					k = 1;
					smallest = root3[1];
				}

				if (smallest > root3[2])
				{
					k = 2;
					smallest = root3[2];
				}

				double p = ::sqrt(root3[ (k+1) % 3 ]);
				double q = ::sqrt(root3[ (k+2) % 3 ]);
				double r = -g/(8*p*q);
				double s = B/4;

				root[0] = p + q + r - s;
				root[1] = p - q - r - s;
				root[2] = -p + q - r - s;
				root[3] = -p - q + r - s;

				return 4;
			}
			else if (numRoots == 1)
			{
				Complex p;
				Complex q;

				Complex root1(root3[1], im3[1]);
				Complex root2(root3[2], im3[2]);

				SquareRootComplex(root1, p);
				SquareRootComplex(root2, q);				
				if ((q.imag() > 0 && p.imag() > 0) || (q.imag() < 0 && p.imag() < 0))
					q = -q;
				
				Complex pq = p*q;

				double r = -g/(8*pq.real());
				double s = B/4;

				root[0] = p.real() + q.real() + r - s;
				root[1] = -p.real() - q.real() + r - s;

				if (findImaginaryRoots)
				{
					im[0] = im[1] = 0;
					
					root[2] = root[3] = - r - s;
					im[2] = p.imag() - q.imag();
					im[3] = -im[2]; 
				}

				return 2;
			}
		}
	};

} // namespace NRMKFoundation
