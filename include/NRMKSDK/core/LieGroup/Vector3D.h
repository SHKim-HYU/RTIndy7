//  ---------------------- Doxygen info ----------------------
//! \file Vector3D.h
//!
//! \brief
//! Header file for the class Vector3D (API of the LieGroup Libraries)
//!
//! \details
//! This file implements a three-dimensional vector (3D vector for velocity and force) class
//! to be used for the interface of LieGroup library
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

namespace LieGroup
{

class Rotation;
class HTransform;

//  ---------------------- Doxygen info ----------------------
//! \class Vector3D
//!
//! \brief
//! This implements a three-dimensional vector class 
//! which represents three-dimensional vectors. In particular, those entities such as velocity and force
//! can be represented, as well as the three-dimensional Lie algebra elements associated with the rotation matrices.  
//! When it stands for the Lie algebra element, we use the notation \f$ \xi \f$. 
//! 
//! \note
//! In homogeneous four-dimensional representation it has '0' as the fourth row.
//! Compared to Eigen::Vector3d, it is properly initialized.
//! 
//! \sa Eigen::Vector3d
//  ----------------------------------------------------------
class Vector3D : public Eigen::Vector3d
{
public:
	RMATH_EIGEN_DERIVE(Eigen::Vector3d, Vector3D)

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a three-dimensional vector object with specified x, y, and z value
	//! \f$ v = \begin{bmatrix} x \\ y \\ z \end{bmatrix} \f$, i.e. \f$ v = 0 \f$. 
	//!
	//! \details
	//! When arguments are not passed, 
	//! it constructs the default zero vector
	//!
	//! \param x x-coordinate value, default = 0
	//! \param y y-coordinate value, default = 0
	//! \param z z-coordinate value, default = 0
	//  ----------------------------------------------------------
	inline Vector3D(double x = 0, double y = 0, double z = 0) : Eigen::Vector3d(x, y, z) 
	{
	}

	inline static  Vector3D const Zero() 
	{
		return Vector3D();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the negative of the current vector, i.e. the opposite direction vector
	//! That is, it returns \f$ -v \f$. 
	//!
	//! \return The negative of the (Vector3D) vector
	//  ----------------------------------------------------------
	inline Vector3D negative() const
	{
		return Vector3D(-x(), -y(), -z());
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the values for the three-dimensional vector, i.e. \f$ v = \begin{bmatrix} u \\ v \\ w \end{bmatrix} \f$
	//!
	//! \param u x-coordinate value
	//! \param v y-coordinate value
	//! \param w z-coordinate value
	//  ----------------------------------------------------------
	inline void set(double u, double v, double w) 
	{ 
		x() = u; y() = v; z() = w; 
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the cross-product matrix (a.k.a. ceiled matrix), denoted by \f$ \left\lceil v \right\rceil \f$,
	//! of the vector by ceil operator
	//!
	//!	\details
	//! The matrix is of the form 
	//! \f$ \left\lceil v \right\rceil = \begin{bmatrix} 0 & -z & y \\ z & 0 & -x \\ -y & x & 0 \end{bmatrix} \f$. 
	//! It satisfies \f$ \left \lceil v \right \rceil w = v \times w \f$. 
	//!
	//! \return The 3-by-3 (skew-symmetric) matrix
	//!
	//! \sa Eigen::Matrix3d
	//  ----------------------------------------------------------
	inline Eigen::Matrix3d ceil() const 
	{
		return internal::_ceil_expression(x(), y(), z());
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the squared matrix of the ceiled matrix
	//! of the displacement vector 
	//!
	//! \details
	//! The matrix is of the form \f$ \left \lceil v \right\rceil^2 \f$, 
	//!	which is symmetric.
	//!
	//! \return The 3-by-3 (symmetric) matrix
	//!
	//! \sa Eigen::Matrix3d
	//  ----------------------------------------------------------
	inline Eigen::Matrix3d ceilSqr() const
	{
		return internal::_ceil_sqr_expression(x(), y(), z());
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the rotation matrix of the vector by the exponential map on \f$ SO(3) \f$.
	//! That is, it returns \f$ \exp(\xi) \f$ for the vector \f$ \xi \f$.
	//!
	//! \return The three-dimensional rotation matrix
	//!
	//! \sa Eigen::Matrix3d
	//  ----------------------------------------------------------
	inline Rotation exp() const;

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the rotation matrix of the vector by the exponential map on \f$ SO(3) \f$
	//! in terms of the precomputed parameters. That is, it returns \f$ \exp(\xi) \f$ for the vector \f$ \xi \f$.
	//!
	//! \details
	//! The parameters should computed by compute_terms(). 
	//! They can be reused to compute exponential map family.
	//!
	//! \param alpha alpha value
	//! \param beta beta value
	//! \param theta_sqr theta_sqr value
	//!
	//! \return The three-dimensional rotation matrix
	//!
	//! \sa Eigen::Matrix3d
	//! \sa compute_terms()
	//  ----------------------------------------------------------
	inline Rotation exp(double alpha, double beta, double theta_sqr) const;

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the differential-of-exponential matrix on \f$ SO(3) \f$ of the vector.
	//! That is, it returns \f$ {\rm dexp}_{\xi} \f$ for the vector \f$ \xi \f$.
	//!
	//! \details
	//! The matrix is denoted by \f$ {\rm dexp}_{\xi} \f$ satisfies the relationship 
	//!   \f$ \omega = {\rm dexp}_{-\xi} \dot{\xi} \f$, where \f$ \xi \f$ is the vector,
	//! where \f$ \omega \f$ is the angular velocity of the rotation matrix such that 
	//!   \f$ \left \lceil \omega \right \rceil = \exp(\xi)^T \frac{d}{dt} \exp(\xi) \f$. 
	//! Note the minus sign in computing \f$ {\rm dexp}_{-\xi} \f$. It is worth noting that 
	//! \f$ {\rm dexp}_{-\xi} = {\rm dexp}_{\xi}^T \f$.
	//!
	//! \return The 3-by-3 differential-of-exponential matrix 
	//!
	//! \sa Eigen::Matrix3d
	//  ----------------------------------------------------------
	inline Eigen::Matrix3d dexp() const 
	{
		double alpha;
		double beta; 
		double theta_sqr;
		compute_terms(&alpha, &beta, &theta_sqr);

		return dexp(alpha, beta, theta_sqr);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the differential-of-exponential matrix on \f$ SO(3) \f$ of the vector 
	//! in terms of the precomputed parameters. 
	//!  That is, it returns \f$ {\rm dexp}_{\xi} \f$ for the vector \f$ \xi \f$.
	//!
	//! \details
	//! The parameters should computed by compute_terms(). 
	//! They can be reused to compute exponential map family.
	//!
	//! \param alpha alpha value
	//! \param beta beta value
	//! \param theta_sqr heta_sqr value
	//!
	//! \return The three-dimensional differential-of-exponential  matrix
	//!
	//! \sa Eigen::Matrix3d
	//! \sa compute_terms()
	//  ----------------------------------------------------------
	inline Eigen::Matrix3d dexp(double alpha, double beta, double theta_sqr) const 
	{
		if (theta_sqr > 0)
			return internal::_common_expression_1(beta*0.5, (1.0 - alpha)/theta_sqr, derived());
		else
			return Eigen::Matrix3d::Identity();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the inverse of differential-of-exponential matrix on \f$ SO(3) \f$ of the vector.
	//!  That is, it returns \f$ {\rm dexp}_{\xi}^{-1} \f$ for the vector \f$ \xi \f$.
	//! It is worth noting that \f$ {\rm dexp}_{-\xi}^{-1} = {\rm dexp}_{\xi}^{-T} \f$.
	//!
	//! \details
	//! The matrix always exists because the differential-of-exponential matrix is always invertible. 
	//!
	//! \return The 3-by-3 inverse of differential-of-exponential matrix 
	//!
	//! \sa Eigen::Matrix3d
	//  ----------------------------------------------------------
	inline Eigen::Matrix3d dexp_inv() const 
	{
		double alpha;
		double beta; 
		double theta_sqr;
		compute_terms(&alpha, &beta, &theta_sqr);

		return dexp_inv(alpha, beta, theta_sqr);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the inverse of differential-of-exponential matrix on \f$ SO(3) \f$ of the vector 
	//! in terms of the precomputed parameters. That is, it returns 
	//! \f$ {\rm dexp}_{\xi}^{-1} \f$ for the vector \f$ \xi \f$.
	//!
	//! \details
	//! The parameters should computed by compute_terms(). 
	//! They can be reused to compute exponential map family.
	//!
	//! \param alpha alpha value
	//! \param beta beta value
	//! \param theta_sqr heta_sqr value
	//!
	//! \return The three-dimensional inverse of differential-of-exponential  matrix
	//!
	//! \sa Eigen::Matrix3d
	//! \sa compute_terms()
	//  ----------------------------------------------------------
	inline Eigen::Matrix3d dexp_inv(double alpha, double beta, double theta_sqr) const 
	{
		if (theta_sqr > 0)
		{
			return internal::_common_expression_1(-0.5, (1.0 - alpha/beta)/theta_sqr, derived());
		}
		else
			return Eigen::Matrix3d::Identity();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the time-derivative of the differential-of-exponential matrix on \f$ SO(3) \f$ of the vector \f$ \xi \f$
	//! for the given time-derivative of the vector \f$ \dot{\xi} \f$. That is, it returns 
	//! \f$ \frac{d}{dt} {\rm dexp}_{\xi} (\dot{\xi}) \f$.
	//!
	//! \param xidot time-derivative of the vector or \f$ \dot{\xi} \f$
	//!
	//! \return The 3-by-3 time-derivative of the differential-of-exponential matrix 
	//!
	//! \sa Eigen::Matrix3d
	//  ----------------------------------------------------------
	template<typename OtherDerived>
	inline Eigen::Matrix3d ddt_dexp(Eigen::MatrixBase<OtherDerived> const & xidot) const 
	{
		double alpha;
		double beta; 
		double theta_sqr;
		compute_terms(&alpha, &beta, &theta_sqr);

		return ddt_dexp(alpha, beta, theta_sqr, this->dot(xidot), xidot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the time-derivative of the differential-of-exponential matrix on \f$ SO(3) \f$ of the vector \f$ \xi \f$
	//! for the given time-derivative of the vector \f$ \dot{\xi} \f$ in terms of the precomputed parameters. 
	//! That is, it returns \f$ \frac{d}{dt} {\rm dexp}_{\xi} (\dot{\xi}) \f$.
	//!
	//! \details
	//! The parameters should computed by compute_terms(). 
	//! They can be reused to compute exponential map family.
	//!
	//! \param alpha alpha value
	//! \param beta beta value
	//! \param theta_sqr square of the theta
	//! \param xi_xidot dot product of the vector with xidot or \f$ \xi^T \dot{\xi} \f$
	//! \param xidot time-derivative of the vector or \f$ \dot{\xi} \f$
	//!
	//! \return The 3-by-3 time-derivative of the differential-of-exponential matrix 
	//!
	//! \sa Eigen::Matrix3d
	//! \sa compute_terms()
	//  ----------------------------------------------------------
	template<typename OtherDerived>
	inline Eigen::Matrix3d ddt_dexp(double alpha, double beta, double theta_sqr, double xi_xidot, Eigen::MatrixBase<OtherDerived> const & xidot) const 
	{
		if (theta_sqr > 0)
		{
			const double d = (1 - alpha)/theta_sqr;
			const double c = 0.5*beta;
			const double common = xi_xidot/theta_sqr;
			const double a = (alpha - beta)*common;
			const double b = (c - 3*d)*common;
			return internal::_common_expression_2(a, b, c, d, derived(), xidot);
		}
		else
		{
			//return Vector3D(0.5*y).ceil();
			return internal::_ceil_expression(0.5*xidot(0), 0.5*xidot(1), 0.5*xidot(2)); 
		}
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the time-derivative of the inverse of differential-of-exponential matrix on \f$ SO(3) \f$ of the vector \f$ \xi \f$
	//! for the given time-derivative of the vector \f$ \dot{\xi} \f$. That is, it returns 
	//! \f$ \frac{d}{dt} {\rm dexp}_{\xi}^{-1} (\dot{\xi}) \f$.
	//!
	//! \param xidot time-derivative of the vector or \f$ \dot{\xi} \f$
	//!
	//! \return The 3-by-3 time-derivative of the inverse of differential-of-exponential matrix 
	//!
	//! \sa Eigen::Matrix3d
	//  ----------------------------------------------------------
	template<typename OtherDerived>
	inline Eigen::Matrix3d ddt_dexp_inv(Eigen::MatrixBase<OtherDerived> const & xidot) const 
	{
		double alpha;
		double beta; 
		double theta_sqr;
		compute_terms(&alpha, &beta, &theta_sqr);

		return ddt_dexp_inv(alpha, beta, theta_sqr, this->dot(xidot), xidot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the time-derivative of the inverse of differential-of-exponential matrix on \f$ SO(3) \f$ of the vector \f$ \xi \f$
	//! for the given time-derivative of the vector \f$ \dot{\xi} \f$ in terms of the precomputed parameters. 
	//! That is, it returns \f$ \frac{d}{dt} {\rm dexp}_{\xi}^{-1} (\dot{\xi}) \f$.
	//!
	//! \details
	//! The parameters should computed by compute_terms(). 
	//! They can be reused to compute exponential map family.
	//!
	//! \param alpha alpha value
	//! \param beta beta value
	//! \param theta_sqr square of the theta
	//! \param xi_xidot dot product of the vector with xidot or \f$ \xi^T \dot{\xi} \f$
	//! \param xidot time-derivative of the vector or \f$ \dot{\xi} \f$
	//!
	//! \return The 3-by-3 time-derivative of the differential-of-exponential matrix 
	//!
	//! \sa Eigen::Matrix3d
	//! \sa compute_terms()
	//  ----------------------------------------------------------
	template<typename OtherDerived>
	inline Eigen::Matrix3d ddt_dexp_inv(double alpha, double beta, double theta_sqr, double xi_xidot, Eigen::MatrixBase<OtherDerived> const & xidot) const 
	{
		if (theta_sqr > 0)
		{
			const double gamma = alpha/beta;
			const double d = (1 - gamma)/theta_sqr;			
			const double b = (1/beta + gamma - 2)/theta_sqr*xi_xidot/theta_sqr;
			return internal::_common_expression_2(0, b, -0.5, d, derived(), xidot);
		}
		else
		{
			return internal::_ceil_expression(-0.5*xidot(0), -0.5*xidot(1), -0.5*xidot(2)); 
		}
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! compute the terms \f$ \alpha \f$, \f$ \beta \f$, and \f$ \theta^2 \f$ for the vector \f$ \xi \f$
	//! to compute exponential map family in order to prevent repeated computations
	//!
	//! \details
	//! The parameters are defined as: \f$ \alpha = s c \f$, \f$ \beta = s^2 \f$, and \f$ \theta = \left\| \xi \right\| \f$
	//! where \f$ s = \frac{\sin(\theta/2)}{\theta/2} \f$ and \f$ c = \cos(\theta/2) \f$.
	//!
	//! \param alpha alpha value or \f$ \alpha \f$
	//! \param beta beta value or \f$ \beta \f$
	//! \param theta_sqr the squared norm of theta \f$ \theta^2 \f$
	//  ----------------------------------------------------------
	inline void compute_terms(double* alpha, double* beta, double* theta_sqr) const
	{
		*theta_sqr = squaredNorm();		
		if (*theta_sqr > 0)
		{
			double theta = ::sqrt(*theta_sqr);
			double theta_2 = theta*0.5;

			double s = ::sin(theta_2)/theta_2;
			double c = ::cos(theta_2);
		
			*alpha = s*c;
			*beta = s*s;
		}
		else
		{
			*alpha = 1;
			*beta = 1;
		}
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! rotates the vector by the rotation matrix
	//! 
	//! \details
	//!	The vector is updated by \f$ v = R v \f$.
	//! The function is defined in Rotation.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Vector3D v; 
	//! LieGroup::Rotation R;
	//! ...
	//! v.rotate(R); // This sets v = R*v
	//!	\endcode
	//!
	//! \param R The three-dimensional rotation matrix
	//!
	//! \sa LieGroup::Rotation
	//! \sa LieGroup::Rotation::rotate(Vector3D const & v)
	//  ----------------------------------------------------------
	inline void rotate(Rotation const & R);;

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! rotates the vector by the inverse of the rotation matrix
	//! 
	//! \details
	//!	The vector is updated by \f$ v = R^T v \f$.
	//! The function is defined in Rotation.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Vector3D v; 
	//! LieGroup::Rotation R;
	//! ...
	//! v.irotate(R); // This sets v = R.transpose()*v
	//!	\endcode
	//!
	//! \param R The three-dimensional rotation matrix
	//!
	//! \sa LieGroup::Rotation
	//! \sa LieGroup::Rotation::irotate(Vector3D const & v)
	//  ----------------------------------------------------------
	inline void irotate(Rotation const & R);;

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! transforms the vector by the homogeneous transformation matrix
	//! 
	//! \details
	//! The vector is updated by \f$ v = R_T v \f$ 
	//! for \f$ T = \begin{bmatrix} R_T & r_T \\ 0 & 1 \end{bmatrix} \f$. 
	//! Note that the displacement \f$ r_T \f$ is irrelevant.
	//! The function is defined in HTransform.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Vector3D v; 
	//! LieGroup::HTransform T;
	//! ...
	//! v.transform(T); // This sets v = T.R()*v
	//! \endcode
	//!
	//! \param T The homogeneous transformation matrix
	//!
	//! \sa LieGroup::HTransform
	//! \sa LieGroup::HTransform::transform(Vector3D const & v)
	//  ----------------------------------------------------------
	inline void transform(const HTransform& T);

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! transforms the vector by the inverse homogeneous transformation matrix
	//! 
	//! \details
	//! The vector is updated by \f$ v = R_T^T v \f$ 
	//! for \f$ T = \begin{bmatrix} R_T & r_T \\ 0 & 1 \end{bmatrix} \f$. 
	//! Note that the displacement \f$ r_T \f$ is irrelevant.
	//! The function is defined in HTransform.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Vector3D v; 
	//! LieGroup::HTransform T;
	//! ...
	//! v.itransform(T); // This sets v = T.R().transpose()*v
	//! \endcode
	//!
	//! \param T The homogeneous transformation matrix
	//!
	//! \sa LieGroup::HTransform
	//! \sa LieGroup::HTransform::itransform(Vector3D const & v)
	//  ----------------------------------------------------------
	inline void itransform(const HTransform& T);;
};

}
