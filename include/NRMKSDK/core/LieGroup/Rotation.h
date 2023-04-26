//  ---------------------- Doxygen info ----------------------
//! \file Rotation.h
//!
//! \brief
//! Header file for the class Rotation (API of the LieGroup Libraries)
//!
//! \details
//! This file implements a three-dimensional rotation matrix class
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
//! \date July 2014
//! 
//! \version 1.7.2
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013-2014 Neuromeka Co., Ltd.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#include "Eigen/Dense"

namespace LieGroup
{

class Displacement;
class Vector3D;

//  ---------------------- Doxygen info ----------------------
//! \class Rotation
//!
//! \brief
//! This implements a three-dimensional rotation matrix class which
//! represents the orientation of a coordinate frame (or the body having the frame). 
//! We use three-dimensional square matrix representation for rotation matrices, 
//! which have mutually orthogonal columns having unit norm, and unit determinant.
//! 
//! \note
//! Compared to Eigen::Matrix3d, it is properly initialized having identity rotation.
//! 
//! \sa Eigen::Matrix3d
//  ----------------------------------------------------------
class Rotation : public Eigen::Matrix3d
{
public: 
	RMATH_EIGEN_DERIVE(Eigen::Matrix3d, Rotation)

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the identity rotation matrix
	//!
	//! \details
	//! It is initialized as the identity rotation matrix
	//  ----------------------------------------------------------
	inline Rotation() : Eigen::Matrix3d(Eigen::Matrix3d::Identity()) 
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the identity element
	//  ----------------------------------------------------------
	inline static  Rotation const Zero() 
	{
		return Rotation();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the rotation matrix using the Eigen::RotationBase object
	//!
	//! \param R The 3-dimensional Eigen::RotationBase object
	//  ----------------------------------------------------------
	template<class Derived>
	inline Rotation(const Eigen::RotationBase<Derived, 3>& R) : Eigen::Matrix3d(R) 
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the rotation matrix in terms of its nine elements.
	//! That is, the resulting rotation matrix is 
	//! \f$ R = \begin{bmatrix} r_{11} & r_{12} & r_{13} \\ r_{21} & r_{22} & r_{23} \\ r_{31} & r_{32} & r_{33} \end{bmatrix} \f$.
	//! 
	//! \details
	//! The elements are specified in row-major order. It is mandatory that 
	//! the elements comprise a proper rotation matrix.
	//!
	//! \param r11 \f$ r_{11} \f$
	//! \param r12 \f$ r_{12} \f$
	//! \param r13 \f$ r_{13} \f$
	//! \param r21 \f$ r_{21} \f$
	//! \param r22 \f$ r_{22} \f$
	//! \param r23 \f$ r_{23} \f$
	//! \param r31 \f$ r_{31} \f$
	//! \param r32 \f$ r_{32} \f$
	//! \param r33 \f$ r_{33} \f$
	//  ----------------------------------------------------------
	inline Rotation(double r11, double r12, double r13, double r21, double r22, double r23, double r31, double r32, double r33)
		:  Eigen::Matrix3d()
	{
		*this << r11, r12, r13, r21, r22, r23, r31, r32, r33;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the rotation matrix in terms of rotation around an x,y, or z axis
	//!
	//! \param axis axis of rotation (All axis index should be among 0, 1, and 2 for x-, y-, and z-axis)
	//! \param theta angles of rotation (in radian)
	//  ----------------------------------------------------------
	inline Rotation(int axis, double theta)
	{		
		const double s = ::sin(theta);
		const double c = ::cos(theta);
		
		switch (axis)
		{
		case 0:
			 *this << 1, 0, 0, 0, c, -s, 0, s, c;
			break;

		case 1:
			*this << c, 0, s, 0, 1, 0, -s, 0, c;
			break;

		case 2:
			*this << c, -s, 0, s, c, 0, 0, 0, 1;
			break;
		}
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the rotation matrix in terms of Euler angles representation
	//!
	//! \details 
	//! Euler angles are represented by one three-dimensional vector containing Euler angles
	//! and three indices indicating the rotation axes. The resulting rotation matrix is: 
	//! \f$ R = Rot(\theta_1; \hat{e}_{k_1}) Rot(\theta_2; \hat{e}_{k_2}) Rot(\theta_3; \hat{e}_{k_3}) \f$,
	//! where \f$ \theta = \begin{bmatrix} \theta_1 \\ \theta_2 \\ \theta_3 \end{bmatrix} \f$ is the euler angle vector
	//! whose \f$ i \f$-th element \f$ \theta_i \f$ is the rotation angle 
	//! for the \f$ i \f$-th rotation axis \f$ \hat{e}_{k_i} \f$, where \f$ \hat{e}_j \f$ is the unit vector having \f$ 1 \f$ 
	//! at the \f$ j \f$-th entry. 
	//!
	//! \param ea Euler angles (any three-dimensional vector) (in radian) 
	//! \param a0 first axis of rotation (All axis index should be among 0, 1, and 2 for x-, y-, and z-axis)
	//! \param a1 second axis of rotation
	//! \param a2 last axis of rotation
	//  ----------------------------------------------------------
	inline Rotation(const Eigen::Vector3d& ea, int a0, int a1, int a2)
		: Eigen::Matrix3d(Eigen::AngleAxisd(ea[0], Eigen::Vector3d::Unit(a0))
						  *Eigen::AngleAxisd(ea[1], Eigen::Vector3d::Unit(a1))
						  *Eigen::AngleAxisd(ea[2], Eigen::Vector3d::Unit(a2)))
	{		
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the exponential coordinates of the rotation matrix, that is 
	//! it returns the three-dimensional vector \f$ \xi \f$ such that \f$ \exp(\xi) = R \f$.
	//!
	//! \return The three-dimensional exponential coordinate vector
	//  ----------------------------------------------------------
	inline Vector3D expCoord() const
	{
		Eigen::AngleAxisd aa(*this);
		return Vector3D(aa.angle()*aa.axis());
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the unit quaternion from the rotation matrix
	//!
	//! \details 
	//! Unit quaternion is represented by \f$ q = w + ix + jy + kz \f$ from its four coefficients \f$w\f$, \f$x\f$, \f$y\f$, and \f$z\f$.
	//  ----------------------------------------------------------
	inline Eigen::Quaterniond quaternion() const
	{
		return Eigen::Quaterniond(*this); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the Euler angles from the rotation matrix
	//!
	//! \details 
	//! Euler angles are represented by one three-dimensional vector containing Euler angles
	//! and three indices indicating the rotation axes. The resulting rotation matrix is: 
	//! \f$ R = Rot(\theta_1; \hat{e}_{k_1}) Rot(\theta_2; \hat{e}_{k_2}) Rot(\theta_3; \hat{e}_{k_3}) \f$,
	//! where \f$ \theta = \begin{bmatrix} \theta_1 \\ \theta_2 \\ \theta_3 \end{bmatrix} \f$ is the euler angle vector
	//! whose \f$ i \f$-th element \f$ \theta_i \f$ is the rotation angle 
	//! for the \f$ i \f$-th rotation axis \f$ \hat{e}_{k_i} \f$, where \f$ \hat{e}_j \f$ is the unit vector having \f$ 1 \f$ 
	//! at the \f$ j \f$-th entry. 
	//!
	//! \param a0 first axis of rotation (All axis index should be among 0, 1, and 2 for x-, y-, and z-axis)
	//! \param a1 second axis of rotation
	//! \param a2 last axis of rotation
	//  ----------------------------------------------------------
	inline Eigen::Vector3d eulerAngles(int a0, int a1, int a2) const
	{
		return Eigen::Matrix3d::eulerAngles(a0, a1, a2);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the Euler angles Jacobian
	//!
	//! \details 
	//! Euler angles are function of rotation matrix, i.e. \f$ \theta = \theta(R) \f$. 
	//! In general, the (induced) body angular velocity \f$ \omega \f$ is represented 
	//! by \f$ \omega = J(\theta) \dot{\theta} \f$, where \f$ J \f$ is called the Euler angle Jacobian. 
	//! Its inversion is used to compute the Euler angle velocity fir the body angular velocity, i.e.
	//! \f$ \dot{\theta} = J^{-1}(\theta) \omega \f$. It has singularity in some configuration which is known
	//! as gimbal lock or representation singularity. 
	//!
	//! \param ea Euler angles (any three-dimensional vector) (in radian) 
	//! \param a0 first axis of rotation (All axis index should be among 0, 1, and 2 for x-, y-, and z-axis)
	//! \param a1 second axis of rotation
	//! \param a2 last axis of rotation
	//!
	//! \return the Euler angle Jacobian
	//  ----------------------------------------------------------
	static inline Eigen::Matrix3d EulerAnglesJacobian(Eigen::Vector3d const &  ea, int a0, int a1, int a2)
	{
		Eigen::Matrix3d J; 
		
		J.col(2) = Eigen::Vector3d::Unit(a2);

		Rotation R3(a2, ea(2));		
		J.col(1).transpose() =  R3.row(a1);

		Rotation R2(a1, ea(1));		
		J.col(0).transpose() =  R2.row(a0)*R3;

		return J;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the Euler angles Jacobian and its derivative
	//!
	//! \details 
	//! Euler angles are function of rotation matrix, i.e. \f$ \theta = \theta(R) \f$. 
	//! In general, the (induced) body angular velocity \f$ \omega \f$ is represented 
	//! by \f$ \omega = J(\theta) \dot{\theta} \f$, where \f$ J \f$ is called the Euler angle Jacobian. 
	//! This  computes its derivative of \f$ J \f$ when the Jacobian is given.
	//!
	//! \param ea Euler angles (any three-dimensional vector) (in radian) 
	//! \param ea_dot Euler angles derivative (any three-dimensional vector) 
	//! \param a0 first axis of rotation (All axis index should be among 0, 1, and 2 for x-, y-, and z-axis)
	//! \param a1 second axis of rotation
	//! \param a2 last axis of rotation
	//! \param J Euler angle Jacobian 
	//! \param Jdot the Euler angle Jacobian derivative
	//  ----------------------------------------------------------
	static inline Eigen::Matrix3d EulerAnglesJacobianDerivative(Eigen::Vector3d const &  ea, Eigen::Vector3d const &  ea_dot, int a0, int a1, int a2, Eigen::Matrix3d const & J)
	{
		Eigen::Matrix3d Jdot;

		Jdot.col(2).setZero();

		//Jdot.col(1) = -LieGroup::Vector3D::Unit(2).cross(J.col(1));
		Eigen::Vector3d w3 = ea_dot(2)*LieGroup::Vector3D::Unit(a2);
		Jdot.col(1) = J.col(1).cross(w3);

		Jdot.col(0) = J.col(0).cross(w3 + ea_dot(1)*J.col(1));

		return Jdot;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! updates the rotation matrix by rotation about its own x-axis
	//!
	//! \details
	//! The rotation matrix is updated to 
	//! \f$ R = R Rot(\theta; \hat{e}_1) = R \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos(\theta) & -\sin(\theta) \\ 0 & \sin(\theta) & \cos(\theta) \end{bmatrix} \f$.
	//!
	//!	@param theta the rotation angle (in radian)
	//  ----------------------------------------------------------
	inline void postRotateX(double theta)
	{
		const double s = ::sin(theta);
		const double c = ::cos(theta);

		Eigen::Matrix3d R;
		R << 1, 0, 0, 0, c, -s, 0, s, c;

		*this *= R;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! updates the rotation matrix by rotation about its own y-axis
	//!
	//! \details
	//! The rotation matrix is updated to 
	//! \f$ R = R Rot(\theta; \hat{e}_2) = R \begin{bmatrix} \cos(\theta) & 0 & \sin(\theta) \\ 0 &  1 & 0 \\ -\sin(\theta) & 0 &  \cos(\theta) \end{bmatrix} \f$.
	//!
	//!	@param theta the rotation angle (in radian)
	//  ----------------------------------------------------------
	inline void postRotateY(double theta)
	{
		const double s = ::sin(theta);
		const double c = ::cos(theta);

		Eigen::Matrix3d R;
		R << c, 0, s, 0, 1, 0, -s, 0, c;

		*this *= R;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! updates the rotation matrix by rotation about its own z-axis
	//!
	//! \details
	//! The rotation matrix is updated to 
	//! \f$ R = R Rot(\theta; \hat{e}_3) = R \begin{bmatrix} \cos(\theta) & \sin(\theta) & 0 \\ \sin(\theta) & \cos(\theta) & 0 \\ 0 & 0 & 1 \end{bmatrix} \f$.
	//!
	//!	@param theta the rotation angle (in radian)
	//  ----------------------------------------------------------
	inline void postRotateZ(double theta)
	{
		const double s = ::sin(theta);
		const double c = ::cos(theta);

		Eigen::Matrix3d R;
		R << c, -s, 0, s, c, 0, 0, 0, 1;

		*this *= R;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! updates the rotation matrix by rotation about the reference x-axis
	//!
	//! \details
	//! The rotation matrix is updated to 
	//! \f$ R = Rot(\theta; \hat{e}_1) R  = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos(\theta) & -\sin(\theta) \\ 0 & \sin(\theta) & \cos(\theta) \end{bmatrix} R \f$.
	//!
	//!	@param theta the rotation angle (in radian)
	//  ----------------------------------------------------------
	inline void preRotateX(double theta)
	{
		const double s = ::sin(theta);
		const double c = ::cos(theta);

		Eigen::Matrix3d R;
		R << 1, 0, 0, 0, c, -s, 0, s, c;

		*this = R * (*this);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! updates the rotation matrix by rotation about the reference y-axis
	//!
	//! \details
	//! The rotation matrix is updated to 
	//! \f$ R = Rot(\theta; \hat{e}_2) R = \begin{bmatrix} \cos(\theta) & 0 & \sin(\theta) \\ 0 &  1 & 0 \\ -\sin(\theta) & 0 &  \cos(\theta) \end{bmatrix} R \f$.
	//!
	//!	@param theta the rotation angle (in radian)
	//  ----------------------------------------------------------
	inline void preRotateY(double theta)
	{
		const double s = ::sin(theta);
		const double c = ::cos(theta);

		Eigen::Matrix3d R;
		R << c, 0, s, 0, 1, 0, -s, 0, c;

		*this = R * (*this);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! updates the rotation matrix by rotation about the reference z-axis
	//!
	//! \details
	//! The rotation matrix is updated to 
	//! \f$ R = Rot(\theta; \hat{e}_3) R = R \begin{bmatrix} \cos(\theta) & \sin(\theta) & 0 \\ \sin(\theta) & \cos(\theta) & 0 \\ 0 & 0 & 1 \end{bmatrix} R \f$.
	//!
	//!	@param theta the rotation angle (in radian)
	//  ----------------------------------------------------------
	inline void preRotateZ(double theta)
	{
		const double s = ::sin(theta);
		const double c = ::cos(theta);

		Eigen::Matrix3d R;
		R << c, -s, 0, s, c, 0, 0, 0, 1;

		*this = R * (*this);
	}
 
	//  ---------------------- Doxygen info ----------------------
	//! \brief 	
	//! generates the cascaded (or composite) rotation matrix by
	//! post-multiplying the rhs rotation matrix
	//!
	//! \details 	 	
	//! When the rotation matrix R_12 stands for the rotation from {1} to {2}, or \f$ ^1R_{2} \f$,
	//! and R_23 for {2} and {3},or \f$ ^2R_{3} \f$, R_12.cascade(R_23) represents 
	//! the rotation from {1} to {3}, or \f$ ^1R_{3} \f$, since \f$ ^1R_{3} = {^1R_{2}} {^2R_{3} } \f$.
	//!
	//! \code{.cpp}
	//! LieGroup::Rotation R_12; 
	//! LieGroup::Rotation R_23; 
	//! ...
	//! LieGroup::Rotation R_13 = R_12.cascade(R_23); // This sets R_13 = R_12*R_13
	//! \endcode
	//!
	//! \param rhs the rotation matrix
	//!
	//! \return Cascaded rotation matrix
	//  ----------------------------------------------------------
	Rotation cascade(const Rotation& rhs) const
	{
		return Rotation((*this)*rhs);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief 	
	//! generates the cascaded (or composite) rotation matrix by
	//! post-multiplying the rhs rotation matrix to the inverse of the rotation matrix
	//!
	//! \details 	 	
	//! When the rotation matrix R_12 stands for the rotation from {1} to {2}, or \f$ ^1R_{2} \f$,
	//! and R_13 for {1} and {3},or \f$ ^1R_{3} \f$, R_12.icascade(R_13) represents 
	//! the rotation from {2} to {3}, or \f$ ^2R_{3} \f$, since \f$ ^2R_{3} = {^1R_{2}^T} {^2R_{3} } \f$.
	//!
	//! \code{.cpp}
	//! LieGroup::Rotation R_12; 
	//! LieGroup::Rotation R_13; 
	//! ...
	//! LieGroup::Rotation R_23 = R_12.icascade(R_13); // This sets R_23 = R_12.transpose()*R_13
	//! \endcode
	//!
	//! \param rhs the rotation matrix
	//!
	//! \return cascaded rotation matrix
	//  ----------------------------------------------------------
	Rotation icascade(const Rotation& rhs) const
	{
		return Rotation(this->transpose()*rhs);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the rotated displacement vector by the rotation matrix
	//! 
	//! \details
	//!	The displacement vector \f$ R r \f$ is returned.
	//! 
	//! \code{.cpp}
	//! LieGroup::Rotation R; 
	//! LieGroup::Displacement r;
	//! ...
	//! LieGroup::Displacement r2 = R.rotate(r); // This sets r2 = R * r
	//!	\endcode
	//!
	//! \param r The displacement vector 
	//! 
	//! \return The rotated displacement vector
	//!
	//! \sa LieGroup::Displacement
	//  ----------------------------------------------------------
	inline Displacement rotate(Displacement const & r) const
	{
		return Displacement(*this*r);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the rotated vector by the rotation matrix
	//! 
	//! \details
	//!	The vector \f$ R v \f$ is returned.
	//! 
	//! \code{.cpp}
	//! LieGroup::Rotation R; 
	//! LieGroup::Vector3D v;
	//! ...
	//! LieGroup::Vector3D v2 = R.rotate(v); // This sets v2 = R * v
	//! \endcode
	//!
	//! \param v The vector 
	//! 
	//! \return The rotated vector
	//!
	//! \sa LieGroup::Vector3D
	//  ----------------------------------------------------------
	inline Vector3D rotate(Vector3D const & v) const
	{
		return Vector3D(*this*v);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the rotated displacement vector by the inverse of the rotation matrix
	//! 
	//! \details
	//!	The displacement vector \f$ R^T r \f$ is returned.
	//! 
	//! \code{.cpp}
	//! LieGroup::Rotation R; 
	//! LieGroup::Displacement r;
	//! ...
	//! LieGroup::Displacement r2 = R.irotate(r); // This sets r2 = R.transpose() * r
	//!	\endcode
	//!
	//! \param r The displacement vector 
	//! 
	//! \return The rotated displacement vector
	//!
	//! \sa LieGroup::Displacement
	//  ----------------------------------------------------------
	inline Displacement irotate(Displacement const & r) const
	{
		return Displacement(this->transpose()*r);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the rotated vector by the inverse of the rotation matrix
	//! 
	//! \details
	//!	The vector \f$ R^T v \f$ is returned.
	//! 
	//! \code{.cpp}
	//! LieGroup::Rotation R; 
	//! LieGroup::Vector3D v;
	//! ...
	//! LieGroup::Vector3D v2 = R.irotate(v); // This sets v2 = R.transpose() * v
	//! \endcode
	//!
	//! \param v The vector 
	//! 
	//! \return The rotated vector
	//!
	//! \sa LieGroup::Vector3D
	//  ----------------------------------------------------------
	inline Vector3D irotate(Vector3D const & v) const
	{
		return Vector3D(this->transpose()*v);
	}
	
// 	/**
// 	* generate the congruence matrix
// 	* @param other the right-hand side matrix of dimension three-by-three
// 	* @param return the resulting matrix, i.e self*other*self^T
// 	*/
// 	inline Eigen::Matrix3d congruence(Eigen::Matrix3d const & rhs) const
// 	{
// 		return (*this)*rhs*(this->transpose());
// 	}
// 
// 	/**
// 	* generate the inverse-congruence matrix
// 	* @param other the right-hand side matrix of dimension three-by-three
// 	* @param return the resulting matrix, i.e self^T*other*self
// 	*/
// 	inline Eigen::Matrix3d icongruence(Eigen::Matrix3d const & rhs) const
// 	{
// 		return (this->transpose())*rhs*(*this);
// 	}
};

//! \sa LieGroup::Vector3D::exp() const
inline Rotation Vector3D::exp() const
{
	double alpha;
	double beta; 
	double theta_sqr;

	compute_terms(&alpha, &beta, &theta_sqr);
	return exp(alpha, beta, theta_sqr);
}

//! \sa LieGroup::Vector3D::exp() const
inline Rotation Vector3D::exp(double alpha, double beta, double theta_sqr) const
{
	if (theta_sqr > 0)
		return Rotation(internal::_common_expression_1(alpha, beta*0.5, derived()));
	else
		return Rotation();
}

//! \sa LieGroup::Vector3D::rotate()
inline void Vector3D::rotate(const Rotation& R)
{
	*this = R*(*this);
}

//! \sa LieGroup::Vector3D::irotate()
inline void Vector3D::irotate(const Rotation& R)
{
	*this = R.transpose()*(*this);
}

//! \sa LieGroup::Displacement::rotate()
inline void Displacement::rotate(const Rotation& R)
{
	*this = R*(*this);
}

//! \sa LieGroup::Displacement::irotate()
inline void Displacement::irotate(const Rotation& R)
{
	*this = R.transpose()*(*this);
}

//! \sa LieGroup::Twist::rotate()
inline void Twist::rotate(const Rotation& R) 
{
	v() = R*v();
	w() = R*w();
}

//! \sa LieGroup::Twist::irotate()
inline void Twist::irotate(const Rotation& R) 
{
	v() = R.transpose()*v();
	w() = R.transpose()*w();
}

//! \sa LieGroup::Wrench::rotate()
inline void Wrench::rotate(const Rotation& R) 
{
	f() = R*f();
	n() = R*n();
}

//! \sa LieGroup::Wrench::irotate()
inline void Wrench::irotate(const Rotation& R) 
{
	f() = R.transpose()*f();
	n() = R.transpose()*n();
}

}