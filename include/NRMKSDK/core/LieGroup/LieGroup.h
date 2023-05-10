//  ---------------------- Doxygen info ----------------------
//! \file Wrench.h
//!
//! \brief
//! Header file for the entire LieGroup modules 
//!
//! \details
//! This file implements the interface of LieGroup library
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

#include "Eigen/Dense"

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

#define RMATH_EIGEN_DERIVE(TYPE, DERIVED)	\
	typedef TYPE Base;		\
	template<typename OtherDerived> \
	DERIVED& operator= (const Eigen::MatrixBase<OtherDerived>& other)	\
	{	\
		this->Base::operator=(other);	\
		return *this;	\
	} \
	template<typename OtherDerived> \
	DERIVED(const Eigen::MatrixBase<OtherDerived>& other) : Base(other) {} 


namespace LieGroup
{

class Displacement;
class Vector3D;
class Rotation;
class HTransform;
class Twist;
class Wrench;

namespace internal
{
	/**
	* get the ceiled matrix by ceil operator
	* @return the ceil matrix, i.e. [self]
	*/
	inline Eigen::Matrix3d _ceil_expression(double x, double y, double z) 
	{
		Eigen::Matrix3d res;
#if 0
		res.coeffRef(0,0) = 0;
		res.coeffRef(0,1) = -z;
		res.coeffRef(0,2) = y;
		res.coeffRef(1,0) = z;
		res.coeffRef(1,1) = 0;
		res.coeffRef(1,2) = -x;
		res.coeffRef(2,0) = -y;
		res.coeffRef(2,1) = x;
		res.coeffRef(2,2) = 0;
#else
		res <<	0, -z, y, 
				z, 0, -x, 
				-y, x, 0;
#endif
		return res;
	}

	/**
	* get the squared ceiled matrix by ceil operator
	* @return the ceil matrix, i.e. [self]^2
	*/
	inline Eigen::Matrix3d _ceil_sqr_expression(double x, double y, double z) 
	{
		Eigen::Matrix3d res;

		const double xsqr = x * x;
		const double ysqr = y * y;
		const double zsqr = z * z;

		const double xy = x * y;
		const double xz = x * z;
		const double yz = y * z;

#if 0
		res.coeffRef(0,0) = -ysqr - zsqr;
		res.coeffRef(0,1) = xy;
		res.coeffRef(0,2) = xz;
		res.coeffRef(1,0) = xy;
		res.coeffRef(1,1) = -xsqr - zsqr;
		res.coeffRef(1,2) = yz;
		res.coeffRef(2,0) = xz;
		res.coeffRef(2,1) = yz;
		res.coeffRef(2,2) = -xsqr - ysqr;
#else
		res <<	-ysqr - zsqr, xy, xz, 
				xy,	-xsqr - zsqr, yz, 
				xz, yz, -xsqr - ysqr;
#endif

		return res;
	}

	/** Constructs and \returns a 3x3 matrix of the form 
	*		I + a[xi] + b[xi]^2 = I + a[xi] + [b*xi][xi]
	*/
	template<typename OtherDerived>
	inline Eigen::Matrix3d _common_expression_1(double a, double b, const Eigen::MatrixBase<OtherDerived>& xi)
	{
		Eigen::Matrix3d res;
		Eigen::Vector3d u = a * xi;
		//Eigen::Vector3d v = b * xi;

		//double common = v.x() * xi.y();
		double common = b * xi.x() * xi.y();
		res.coeffRef(0,1) = common - u.z();
		res.coeffRef(1,0) = common + u.z();

		//common = v.x() * xi.z();
		common = b * xi.x() * xi.z();
		res.coeffRef(0,2) = common + u.y();
		res.coeffRef(2,0) = common - u.y();

		//common = v.y() * xi.z();
		common = b * xi.y() * xi.z();
		res.coeffRef(1,2) = common - u.x();
		res.coeffRef(2,1) = common + u.x();

		//Eigen::Vector3d tmp = v.cwiseProduct(xi);
		Eigen::Vector3d tmp = b*xi.cwiseProduct(xi);
		res.coeffRef(0,0) = 1 - (tmp.y() + tmp.z());
		res.coeffRef(1,1) = 1 - (tmp.z() + tmp.x());
		res.coeffRef(2,2) = 1 - (tmp.x() + tmp.y());

		return res;
	}

	/** Constructs and \returns a 3x3 matrix of the form 
	*	a[xi] + b[xi]^2 + c*[eta] + d[xi,eta] 
	*		= [a*xi + c*eta] + [xi][d*eta] + [d*eta]*[xi] + [b*xi][xi]
	*		= [a*xi + c*eta] + [xi][d*eta] + [b*xi + d*eta]*[xi]
	*/
	inline Eigen::Matrix3d _common_expression_2(double a, double b, double c, double d, 
			const Eigen::Vector3d& xi, const Eigen::Vector3d& eta)
	{
		Eigen::Matrix3d res;
		Eigen::Vector3d u = a * xi + c * eta;
		Eigen::Vector3d w = d * eta;
		Eigen::Vector3d v = b * xi + w;
		
		res.coeffRef(0,1) = v.y() * xi.x() + xi.y() * w.x() - u.z();
		res.coeffRef(1,0) = v.x() * xi.y() + xi.x() * w.y() + u.z();

		res.coeffRef(0,2) = v.z() * xi.x() + xi.z() * w.x() + u.y();
		res.coeffRef(2,0) = v.x() * xi.z() + xi.x() * w.z() - u.y();

		res.coeffRef(1,2) = v.z() * xi.y() + xi.z() * w.y() - u.x();
		res.coeffRef(2,1) = v.y() * xi.z() + xi.y() * w.z() + u.x();

		Eigen::Vector3d tmp = (v + w).cwiseProduct(xi);

		res.coeffRef(0,0) = -tmp.y() - tmp.z();
		res.coeffRef(1,1) = -tmp.z() - tmp.x();
		res.coeffRef(2,2) = -tmp.x() - tmp.y();

		return res;
	}

	/** Constructs and \returns a 3x3 matrix of the form 
	*	a[xi] + b[dxi/dt] + c*[eta] + d[deta/dt] + e[xi]^2 + f[xi,dxi/dt] + g[xi,eta] + h[xi,deta/dt] + j[dxi/dt,eta] 
	*     = [a*xi + b*(dxi/dt) + c*eta + d*(deta/dt)] 
	*		+ [xi][(e/2)*xi + f*dxi/dt + g*eta + h*deta/dt] + [(e/2)*xi + f*dxi/dt + g*eta + h*deta/dt][xi] 
	*		+ [dxi/dt][j*eta] + [j*eta]*[dxi/dt]
	*/
	inline Eigen::Matrix3d _common_expression_3(double a, double b, double c, double d, double e, double f, double g, double h, double j, 
		const Eigen::Vector3d& xi, const Eigen::Vector3d& xidot, const Eigen::Vector3d& eta, const Eigen::Vector3d& etadot)
	{
		Eigen::Matrix3d res;
		Eigen::Vector3d u = a * xi + b * xidot + c * eta + d * etadot;
		Eigen::Vector3d v = (e/2) * xi + f * xidot + g * eta + h * etadot;
		Eigen::Vector3d w = j * eta; 
		
		double common = v.x() * xi.y() + v.y() * xi.x() + w.x() * xidot.y() + w.y() * xidot.x();
		res.coeffRef(0,1) = common - u.z();
		res.coeffRef(1,0) = common + u.z();

		common = v.x() * xi.z() + v.z() * xi.x() + w.x() * xidot.z() + w.z() * xidot.x();
		res.coeffRef(0,2) = common + u.y();
		res.coeffRef(2,0) = common - u.y();

		common = v.y() * xi.z() + v.z() * xi.y() + w.y() * xidot.z() + w.z() * xidot.y();
		res.coeffRef(1,2) = common - u.x();
		res.coeffRef(2,1) = common + u.x();

		Eigen::Vector3d tmp = 2*(v.cwiseProduct(xi) + w.cwiseProduct(xidot));
		res.coeffRef(0,0) = -tmp.y() - tmp.z();
		res.coeffRef(1,1) = -tmp.z() - tmp.x();
		res.coeffRef(2,2) = -tmp.x() - tmp.y();

		return res;
	}

	/** Constructs and \returns a 3x3 matrix of the form 
	*	a[etadot] + b[xidot,eta] = [a*etadot] + [xidot][b*eta] + [b*eta]*[xidot]
	*/
	inline Eigen::Matrix3d _common_expression_4(double a, double b, 
		const Eigen::Vector3d& xi, const Eigen::Vector3d& xidot, const Eigen::Vector3d& eta, const Eigen::Vector3d& etadot)
	{
		Eigen::Matrix3d res;
		Eigen::Vector3d u = a * etadot;
		Eigen::Vector3d v = b * eta;

		double common = v.x() * xidot.y() + v.y() * xidot.x();
		res.coeffRef(0,1) = common - u.z();
		res.coeffRef(1,0) = common + u.z();

		common = v.x() * xidot.z() + v.z() * xidot.x();
		res.coeffRef(0,2) = common + u.y();
		res.coeffRef(2,0) = common - u.y();

		common = v.y() * xidot.z() + v.z() * xidot.y();
		res.coeffRef(1,2) = common - u.x();
		res.coeffRef(2,1) = common + u.x();

		Eigen::Vector3d tmp = 2*u.cwiseProduct(xidot);
		res.coeffRef(0,0) = -tmp.y() - tmp.z();
		res.coeffRef(1,1) = -tmp.z() - tmp.x();
		res.coeffRef(2,2) = -tmp.x() - tmp.y();

		return res;
	}

	/** Constructs and \returns a 3xN matrix of the form v.cross() * rhs
	*/
	template <typename Derived, typename OtherDerived>
	inline void _cross(double x, double y, double z, Eigen::MatrixBase<Derived> const & rhs, Eigen::MatrixBase<OtherDerived> const & res)
	{
		Eigen::MatrixBase<OtherDerived> & _res = const_cast<Eigen::MatrixBase<OtherDerived> &> (res);
		//_res.derived().resize(3, rhs.cols());

		// FIXME Check sizes statically
		Eigen::Vector3d v(x, y, z);
		for (int k = 0; k < rhs.cols(); k++)
			_res.col(k) = v.cross(rhs.col(k));
	}
}

}

#include "Displacement.h"
#include "Vector3D.h"	
#include "Twist.h"
#include "Wrench.h"

#include "Rotation.h"
#include "HTransform.h"