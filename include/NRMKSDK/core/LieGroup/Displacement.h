//  ---------------------- Doxygen info ----------------------
//! \file Displacement.h
//!
//! \brief
//! Header file for the class Displacement (API of the LieGroup Libraries)
//!
//! \details
//! This file implements a displacement (3D vector for displacement and translation) class
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
//! \class Displacement
//!
//! \brief
//! This implements a displacement class 
//! which represents three-dimensional displacement and translation vectors.
//! 
//! \note
//! In homogeneous four-diemnsional representation it has '1' as the fourth row.
//! Contrary to Eigen::Vector3d it is properly initialized.
//! 
//! \sa Eigen::Vector3d
//  ----------------------------------------------------------
class Displacement : public Eigen::Vector3d
{
public:
	RMATH_EIGEN_DERIVE(Eigen::Vector3d, Displacement)

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a displacement object with specified x, y, and z position 
	//! \f$ r = \begin{bmatrix} x \\ y \\ z \end{bmatrix} \f$
	//!
	//! \details
	//! When arguments are not passed, 
	//! it constructs the default zero displacement vector, i.e. \f$ r = 0 \f$. 
	//!
	//! \param x x-coordinate value, default = 0
	//! \param y y-coordinate value, default = 0
	//! \param z z-coordinate value, default = 0
	//  ----------------------------------------------------------
	inline Displacement(double x = 0, double y = 0, double z = 0) : Eigen::Vector3d(x, y, z) 
	{
	} 

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the identity element
	//  ----------------------------------------------------------
	inline static  Displacement const Zero() 
	{
		return Displacement();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the values for the displacement vector, i.e. \f$ r = \begin{bmatrix} u \\ v \\ w \end{bmatrix} \f$
	//!
	//! \param u x-coordinate value
	//! \param v y-coordinate value
	//! \param w z-coordinate value
	//  ----------------------------------------------------------
	inline void set(double u, double v, double w) 
	{ 
		//*this << x, y, z;
		x() = u; y() = v; z() = w; 
	}

	//  ---------------------- Doxygen info ----------------------
	//!	\brief
	//! generates the cross-product matrix (a.k.a. ceiled matrix), denoted by \f$ \left\lceil r \right\rceil \f$,
	//! of the displacement vector by ceil operator
	//!
	//!	\details
	//! The matrix is of the form 
	//! \f$ \left\lceil r \right\rceil = \begin{bmatrix} 0 & -z & y \\ z & 0 & -x \\ -y & x & 0 \end{bmatrix} \f$. 
	//! It satisfies \f$ \left \lceil r \right \rceil v = r \times v \f$. 
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
	//! generates the symmetric matrix multiplying two ceiled matrices
	//! of the displacement vector 
	//!
	//! \details
	//! The matrix is of the form 
	//! \f$ \left \lceil r \right \rceil \left \lceil rhs \right \rceil +  \left \lceil rhs \right \rceil \left \lceil r \right \rceil \f$.
	//!
	//! \param rhs The 3-dimensional vector 
	//!
	//! \tparam Derived derived type
	//!
	//! \return The 3-by-3 symmetric matrix
	//!
	//! \sa Eigen::Matrix3d
	//  ----------------------------------------------------------
	template<typename Derived>
	inline Eigen::Matrix3d ceil(Eigen::MatrixBase<Derived> const & rhs) const
	{
		//EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(OtherDerived,3)

		Eigen::Matrix3d res;

		const double t11 = rhs.x() * this->x(); 
		const double t22 = rhs.y() * this->y();
		const double t33 = rhs.z() * this->z();

		const double t12 = rhs.x() * this->y() + rhs.y() * this->x();
		const double t13 = rhs.x() * this->z() + rhs.z() * this->x();
		const double t23 = rhs.y() * this->z() + rhs.z() * this->y();

		res.coeffRef(0,0) = -2*(t22 + t33);
		res.coeffRef(0,1) = t12;
		res.coeffRef(0,2) = t13;
		res.coeffRef(1,0) = t12;
		res.coeffRef(1,1) = -2*(t11 + t33);
		res.coeffRef(1,2) = t23;
		res.coeffRef(2,0) = t13;
		res.coeffRef(2,1) = t23;
		res.coeffRef(2,2) = -2*(t11 + t22);

		return res;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the squared matrix of the ceiled matrix
	//! of the displacement vector 
	//!
	//! \details
	//! The matrix is of the form \f$ \left \lceil r \right\rceil^2 \f$, 
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
	//! rotates the displacement vector by the rotation matrix
	//! 
	//! \details
	//!	The displacement vector is updated by \f$r = Rr\f$.
	//! The function is defined in Rotation.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Displacement r; 
	//! LieGroup::Rotation R;
	//! ...
	//! r.rotate(R); // This sets r = R*r
	//!	\endcode
	//!
	//! \param R The 3-dimensional rotation matrix
	//!
	//! \sa LieGroup::Rotation
	//! \sa LieGroup::Rotation::rotate(Displacement const & r)
	//  ----------------------------------------------------------
	inline void rotate(Rotation const & R);;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! rotates the displacement vector by the inverse of the rotation matrix
	//! 
	//! \details
	//! The displacement vector is updated by \f$r = R^T r\f$.
	//!	The function is defined in Rotation.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Displacement r; 
	//! LieGroup::Rotation R;
	//! ...
	//! r.irotate(R); // This sets r = R.transpose()*r
	//!	\endcode
	//!
	//! \param R The 3-dimensional rotation matrix
	//!
	//! \sa LieGroup::Rotation
	//! \sa LieGroup::Rotation::irotate(Displacement const & r)
	//  ----------------------------------------------------------
	inline void irotate(Rotation const & R);;

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! transforms the displacement vector by the homogeneous transformation matrix
	//! 
	//! \details
	//! The displacement vector is updated by \f$r = R_T r + r_T\f$ 
	//! for \f$ T = \begin{bmatrix} R_T & r_T \\ 0 & 1 \end{bmatrix}\f$.
	//! The function is defined in HTransform.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Displacement r; 
	//! LieGroup::HTransform T;
	//! ...
	//! r.transform(T); // This sets r = T.R()*r + T.r()
	//!	\endcode
	//!
	//! \param T The homogeneous transformation matrix
	//!
	//! \sa LieGroup::HTransform
	//! \sa LieGroup::HTransform::transform(Displacement const & r)
	//  ----------------------------------------------------------
	inline void transform(HTransform const & T);;

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! transforms the displacement vector by the inverse homogeneous transformation matrix
	//! 
	//! \details
	//! The displacement vector is updated by \f$r = R_T^T (r - r_T) \f$ 
	//! for \f$ T = \begin{bmatrix} R_T & r_T \\ 0 & 1 \end{bmatrix} \f$.
	//! The function is defined in HTransform.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Displacement r; 
	//! LieGroup::HTransform T;
	//! ...
	//! r.itransform(T); // This sets r = T.R().transpose()*(r - T.r())
	//!	\endcode
	//!
	//! \param T The homogeneous transformation matrix
	//!
	//! \sa LieGroup::HTransform
	//! \sa LieGroup::HTransform::itransform(Displacement const & r)
	//  ----------------------------------------------------------
	inline void itransform(HTransform const & T);;
};

}