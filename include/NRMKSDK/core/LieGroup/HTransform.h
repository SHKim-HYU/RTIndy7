//  ---------------------- Doxygen info ----------------------
//! \file HTransform.h
//!
//! \brief
//! Header file for the class HTransform (API of the LieGroup Libraries)
//!
//! \details
//! This file implements a three-dimensional homogeneous transformation matrix class
//! to be used for the interface of LieGroup library
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
//! \date April 2013
//! 
//! \version 0.1
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

#include "Eigen/Dense"

namespace LieGroup
{

class Displacement;
class Vector3D;
class Twist;
class Wrench;
class Rotation;

//  ---------------------- Doxygen info ----------------------
//! \class HTransform
//!
//! \brief
//! This implements a three-dimensional homogeneous transformation matrix class 
//! which represents the rigid body displacement of a coordinate frame (or the body having the frame).
//! We use four-dimensional square matrix representation for homogeneous transformation matrices 
//! of the form \f$ T = \begin{bmatrix} R & r \\ 0 & 1 \end{bmatrix} \f$.
//! 
//! \note
//! Compared to Eigen::Matrix4d, it is initialized properly.
//! 
//! \sa Eigen::Matrix4d
//  ----------------------------------------------------------
class HTransform : public Eigen::Matrix4d
{
public:
	RMATH_EIGEN_DERIVE(Eigen::Matrix4d, HTransform)

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the identity transformation matrix
	//!
	//! \details
	//! It is initialized as the identity homogeneous transformation matrix
	//  ----------------------------------------------------------
	inline HTransform() : Eigen::Matrix4d(Eigen::Matrix4d::Identity()) 
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the identity element
	//  ----------------------------------------------------------
	inline static  HTransform const Zero() 
	{
		return HTransform();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the transformation matrix in terms of any three-dimensional matrix (for rotation matrix)
	//! and any three-dimensional vector (for displacement vector).
	//!
	//! \param R The 3-by-3 dimensional (rotation) matrix
	//! \param r The 3-dimensional (displacement) vector
	//!
	//! \note Sanity of the first argument (standing for rotation matrix) is not checked.
	//! User should pass the proper rotation matrix.
	//  ----------------------------------------------------------
	template<typename Derived1, typename Derived2>
	inline HTransform(Eigen::MatrixBase<Derived1> const & R, Eigen::MatrixBase<Derived2> const & r) 
	{ 
		set(R, r); 
		row(3) << 0, 0, 0, 1; //= Eigen::Matrix4d::Unit(3);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the transformation matrix in terms of the Eigen::RotationBase object
	//! and any three-dimensional vector (for displacement vector).
	//!
	//! \param R The 3-dimensional Eigen::RotationBase object
	//! \param r The 3-dimensional (displacement) vector
	//  ----------------------------------------------------------
	template<typename Derived1, typename Derived2>
	inline HTransform(Eigen::RotationBase<Derived1, 3> const & R, Eigen::MatrixBase<Derived2> const & r) 
	{ 
		set(R, r); 
		row(3) << 0, 0, 0, 1; //= Eigen::Matrix4d::Unit(3);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the transformation matrix in terms of the Rotation object
	//! and the Displacement object
	//!
	//! \param R Rotation object
	//! \param r Displacement object
	//  ----------------------------------------------------------
	inline HTransform(Rotation const & R, Displacement const & r) 
	{ 
		set(R, r); 
		row(3) << 0, 0, 0, 1; //= Eigen::Matrix4d::Unit(3);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the transformation matrix using the Eigen::Affine3d object
	//!
	//! \param T The 3-dimensional Eigen::Affine3d object
	//  ----------------------------------------------------------
	inline HTransform(Eigen::Affine3d const & T) : Eigen::Matrix4d(T.matrix()) 
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the transformation matrix in terms of any three-dimensional matrix (for rotation matrix)
	//! and any three-dimensional vector (for displacement vector).
	//!
	//! \param rot The 3-by-3 dimensional (rotation) matrix
	//! \param disp The 3-dimensional (displacement) vector
	//!
	//! \note Sanity of the first argument (standing for rotation matrix) is not checked.
	//! User should pass the proper rotation matrix.
	//  ----------------------------------------------------------
	template<typename Derived1, typename Derived2>
	inline void set(Eigen::MatrixBase<Derived1> const & rot, Eigen::MatrixBase<Derived2> const & disp)
	{
		R() = rot;
		r() = disp; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the transformation matrix in terms of the Eigen::RotationBase object
	//! and any three-dimensional vector (for displacement vector).
	//!
	//! \param rot The 3-dimensional Eigen::RotationBase object
	//! \param disp The 3-dimensional (displacement) vector
	//  ----------------------------------------------------------
	template<typename Derived1, typename Derived2>
	inline void set(Eigen::RotationBase<Derived1, 3> const & rot, Eigen::MatrixBase<Derived2> const & disp)
	{
		R() = rot;
		r() = disp; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the transformation matrix in terms of the Rotation object
	//! and the Displacement object
	//!
	//! \param rot Rotation object
	//! \param disp Displacement object
	//  ----------------------------------------------------------
	inline void set(Rotation const & rot, Displacement const & disp)
	{
		R() = rot;
		r() = disp;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the rotation matrix block for writing
	//  ----------------------------------------------------------
	inline Eigen::Block<Eigen::Matrix4d, 3, 3> R() 
	{ 
		return topLeftCorner<3, 3>(); 
	}  

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the rotation matrix block for reading
	//  ----------------------------------------------------------
	inline const Eigen::Block<const Eigen::Matrix4d, 3, 3> R() const 
	{ 
		return topLeftCorner<3, 3>(); 
	}  

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the displacement vector block for writing
	//  ----------------------------------------------------------
	inline Eigen::Block<Eigen::Matrix4d, 3, 1> r() 
	{ 
		return topRightCorner<3,1>(); 
	}  

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the displacement vector block for reading
	//  ----------------------------------------------------------
	inline const Eigen::Block<const Eigen::Matrix4d, 3, 1> r() const 
	{
		return topRightCorner<3,1>(); 
	}  
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Returns the x-axis vector block
	//  ----------------------------------------------------------
	inline Eigen::Block<Eigen::Matrix4d, 3, 1> x() 
	{ 
		return block<3,1>(0,0); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the x-axis vector block for reading
	//  ----------------------------------------------------------
	inline const Eigen::Block<const Eigen::Matrix4d, 3, 1> x() const 
	{ 
		return block<3, 1>(0, 0);
	}  
	
	//  ---------------------- Doxygen info ----------------------
	//! \fn Eigen::Block<Eigen::Matrix4d, 3, 1> y()
	//! 
	//! \brief
	//! Returns the y-axis vector block
	//  ----------------------------------------------------------
	inline Eigen::Block<Eigen::Matrix4d, 3, 1> y() 
	{ 
		return block<3,1>(0,1);
	}  
 
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the y-axis vector block for reading
	//  ----------------------------------------------------------
	inline const Eigen::Block<const Eigen::Matrix4d, 3, 1> y() const 
	{ 
		return block<3, 1>(0, 1);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \fn Eigen::Block<Eigen::Matrix4d, 3, 1> z()
	//! 
	//! \brief
	//! Returns the z-axis vector block
	//  ----------------------------------------------------------
	inline Eigen::Block<Eigen::Matrix4d, 3, 1> z() 
	{ 
		return block<3,1>(0,2);
	}

	//! \brief returns the vector form (UVWXYZ - euler)
	inline Vector6d asVector() const
	{
		Vector6d res;
		LieGroup::Vector3D rotAsVector;

		res.tail<3>() = r();

		rotAsVector = R().eulerAngles(2, 1, 0);
		res[0] = rotAsVector[2];
		res[1] = rotAsVector[1];
		res[2] = rotAsVector[0];

		return res;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the z-axis vector block for reading
	//  ----------------------------------------------------------
	inline const Eigen::Block<const Eigen::Matrix4d, 3, 1> z() const 
	{ 
		return block<3, 1>(0, 2); 
	}  

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the exponential coordinates of the homogeneous transformation matrix, that is 
	//! it returns the twist vector \f$ \lambda \f$ such that \f$ \exp(\lambda) = T \f$.
	//!
	//! \details 
	//! The first three elements of the twist vector stands for the linear part, while 
	//! the last three for the rotational part.
	//!
	//! \return The six-dimensional exponential coordinate vector
	//
	//! \sa LieGroup::Twist
	//  ----------------------------------------------------------
	inline Twist expCoord() const
	{
// 		Vector6d res;
// 		res.tail<3>() = Rotation(R()).expCoord();
// 		res.head<3>() = Vector3D(res.tail<3>()).dexp_inv()*r();
// 
// 		return res;

		Vector3D xi = Rotation(R()).expCoord();
		return Twist(xi.dexp_inv()*r(), xi);
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the inverse of the homogeneous transformation matrix of the form 
	//! \f$ \begin{bmatrix} R^T & -R^T r \\ 0 & 1 \end{bmatrix} \f$
	//!
	//! \return Inverse homogeneous transformation matrix
	//  ----------------------------------------------------------
	inline HTransform inverse() const
	{
		return HTransform(R().transpose(), -R().transpose()*r());
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the cascaded (or composite) transformation matrix by
	//! post-multiplying the rhs transformation matrix
	//! 
	//! \details
	//! \code{.cpp}
	//! LieGroup::HTransform T1; 
	//! LieGroup::HTransform T2;
	//! ...
	//! HTransform T3 = T1.cascade(T2); // This sets T3 = T1 * T2
	//! \endcode
	//!
	//! \return Cascaded homogeneous transformation
	//  ----------------------------------------------------------
	inline HTransform cascade(HTransform const & T) const
	{
		return HTransform(R()*T.R(), r() + R()*T.r());
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the cascaded (or composite) transformation matrix by
	//! post-multiplying the rhs transformation matrix to the inverse of the transformation matrix
	//! 
	//! \details 	
	//! \code{.cpp}
	//! LieGroup::HTransform T1; 
	//! LieGroup::HTransform T2;
	//! ...
	//! HTransform T3 = T1.icascade(T2); // This sets T3 = T1.inverse() * T2
	//! \endcode
	//!
	//! \return Cascaded homogeneous transformation
	//  ----------------------------------------------------------
	inline HTransform icascade(HTransform const & T) const
	{
		return HTransform(R().transpose()*T.R(), R().transpose()*(T.r() - r()));
	}
 	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the transformed displacement vector by the homogeneous transformation matrix
	//! 
	//! \details
	//! The displacement vector \f$R rhs + r \f$ is returned. 
	//! 
	//! \code{.cpp}
	//! LieGroup::HTransform T; 
	//! LieGroup::Displacement r;
	//! ...
	//! LieGroup::Displacement r2 = T.transform(r); 
	//!	\endcode
	//!
	//! \param rhs The displacement vector
	//!
	//! \return The transformed displacement vector
	//!
	//! \sa LieGroup::Displacement
	//  ----------------------------------------------------------
	inline Displacement transform(Displacement const & rhs) const
	{
		return Displacement(r() + R()*rhs);
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the transformed vector by the homogeneous transformation matrix
	//! 
	//! \details
	//! The vector \f$R v \f$ is returned. 
	//! 
	//! \code{.cpp}
	//! LieGroup::HTransform T; 
	//! LieGroup::Vector3D v;
	//! ...
	//! LieGroup::Vector3D v2 = T.transform(v); 
	//!	\endcode
	//!
	//! \param v The three-dimensional vector
	//!
	//! \return The transformed vector
	//!
	//! \sa LieGroup::Vector3D
	//  ----------------------------------------------------------
	inline Vector3D transform(Vector3D const & v) const
	{
		return Vector3D(R()*v);
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the transformed displacement vector by the inverse of the homogeneous transformation matrix
	//! 
	//! \details
	//! The displacement vector \f$ R^T (rhs - r) \f$ is returned. 
	//! 
	//! \code{.cpp}
	//! LieGroup::HTransform T; 
	//! LieGroup::Displacement r;
	//! ...
	//! LieGroup::Displacement r2 = T.itransform(r); 
	//!	\endcode
	//!
	//! \param rhs The displacement vector
	//!
	//! \return The transformed displacement vector
	//!
	//! \sa LieGroup::Displacement
	//  ----------------------------------------------------------
	inline Displacement itransform(Displacement const & rhs) const
	{
		return Displacement(R().transpose()*(rhs - r()));
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the transformed vector by the homogeneous transformation matrix
	//! 
	//! \details
	//! The vector \f$R^T v \f$ is returned. 
	//! 
	//! \code{.cpp}
	//! LieGroup::HTransform T; 
	//! LieGroup::Vector3D v;
	//! ...
	//! LieGroup::Vector3D v2 = T.itransform(v); 
	//!	\endcode
	//!
	//! \param v The three-dimensional vector
	//!
	//! \return The transformed vector
	//!
	//! \sa LieGroup::Vector3D
	//  ----------------------------------------------------------
	inline Vector3D itransform(Vector3D const & v) const
	{
		return Vector3D(R().transpose()*v);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the twist by the adjoint transformation of the homogeneous transformation matrix
	//! 
	//! \details
	//! It returns the twist \f$ {\rm Adj}_T V \f$. 
	//! When the homogeneous transformation matrix T stands 
	//! for the coordinate transformation from {ref} to {tar}, 
	//! T.transform(eta) transforms the twist eta represented wrt {tar}
	//! to the equivalent twist represented wrt {ref}.
	//! 
	//! \code{.cpp}
	//! LieGroup::HTransform T; 
	//! LieGroup::Twist eta;
	//! ...
	//! LieGroup::Twist eta2 = T.transform(eta); // This sets eta2 = T.adj() * eta
	//! \endcode
	//!
	//! \param V Body twist
	//!
	//! \return Equivalent twist
	//!
	//! \sa LieGroup::Twist
	//  ----------------------------------------------------------
	inline Twist transform(Twist const & V) const
	{
		Twist res(R()*V.v(), R()*V.w());
		res.v() += r().cross(res.w());
		
		return res;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the twist by the inverse adjoint transformation of the homogeneous transformation matrix
	//! 
	//! \details
	//! It returns the twist \f$ {\rm Adj}_T^{-1} V \f$. 
	//! When the homogeneous transformation matrix T stands 
	//! for the coordinate transformation from {ref} to {tar}, 
	//! T.itransform(eta) transforms the twist eta represented wrt {ref}
	//! to the equivalent twist represented wrt {tar}.
	//! 
	//! \code{.cpp}
	//! LieGroup::HTransform T; 
	//! LieGroup::Twist eta;
	//! ...
	//! LieGroup::Twist eta2 = T.itransform(eta); // This sets eta2 = T.iadj() * eta
	//! \endcode
	//!
	//! \param V Body twist
	//!
	//! \return Equivalent twist
	//!
	//! \sa LieGroup::Twist
	//  ----------------------------------------------------------
	inline Twist itransform(Twist const & V) const
	{
// 		Twist res(R().transpose()*rhs.v(), R().transpose()*rhs.w());
// 		res.v() -= (R().transpose()*r()).cross(res.w());
// 
// 		return res;
		return Twist(R().transpose()*(V.v() - r().cross(V.w())), R().transpose()*V.w());
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the wrench by the inverse-transposed adjoint transformation of the homogeneous transformation matrix
	//! 
	//! \details
	//! It returns the wrench \f$ {\rm Adj}_T^{-T} F \f$.
	//! When the homogeneous transformation matrix T stands 
	//! for the coordinate transformation from {ref} to {tar}, 
	//! T.transform(F) transforms the wrench F represented wrt {tar}
	//! to the equivalent twist represented wrt {ref}.
	//! 
	//! \code{.cpp}
	//! LieGroup::HTransform T; 
	//! LieGroup::Wrench F;
	//! ...
	//! LieGroup::Wrench F2 = T.transform(F); // This sets F2 = T.tiadj() * F
	//!	\endcode
	//!
	//! \param F Body wrench
	//!
	//! \return Equivalent wrench
	//!
	//! \sa LieGroup::Wrench
	//  ----------------------------------------------------------
	inline Wrench transform(Wrench const & F) const
	{
		Wrench res(R()*F.f(), R()*F.n());
		res.n() += r().cross(res.f());

		return res;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the wrench by the transposed adjoint transformation of the homogeneous transformation matrix
	//! 
	//! \details
	//! The wrench is updated by \f$ F = {\rm Adj}_T^{T} F \f$.	 	
	//! When the homogeneous transformation matrix T stands 
	//! for the coordinate transformation from {ref} to {tar}, 
	//! T.itransform(F) transforms the wrench F represented wrt {ref}
	//! to the equivalent twist represented wrt {tar}.
	//! 
	//! \code{.cpp}
	//! LieGroup::HTransform T; 
	//! LieGroup::Wrench F;
	//! ...
	//! LieGroup::Wrench F2 = T.itransform(F); // This sets F2 = T.tadj() * F
	//!	\endcode
	//!
	//! \param F Body wrench
	//!
	//! \return Equivalent wrench
	//!
	//! \sa LieGroup::Wrench
	//  ----------------------------------------------------------
	inline Wrench itransform(Wrench const & F) const
	{
// 		Wrench res(R().transpose()*rhs.f(), R().transpose()*rhs.n());
// 		res.n() -= (R().transpose()*r()).cross(res.f());
// 
// 		return res;

		return Wrench(R().transpose()*F.f(), R().transpose()*(F.n() - r().cross(F.f())));
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the adjoint transformation matrix multiplied by any matrix (having six rows and arbitrary number of columns)
	//!
	//! \details 
	//!	The resulting matrix is 
	//! \f$ {\rm Adj}_T \begin{bmatrix} U \\ L \end{bmatrix} = \begin{bmatrix} R U + \left \lceil r \right \rceil R L \\ R L \end{bmatrix} \f$.
	//! 
	//! \param rhs the matrix (of size 6-by-N)
	//! \param res the resulting matrix (of size 6-by-N)
	//  ----------------------------------------------------------
	template <typename Derived, typename OtherDerived>
	inline void transform(Eigen::MatrixBase<Derived> const & rhs, Eigen::MatrixBase<OtherDerived> const & res) const
	{
		Eigen::MatrixBase<OtherDerived> & _res = const_cast<Eigen::MatrixBase<OtherDerived> &>(res);

		_res.bottomRows<3>().noalias() = R()*rhs.bottomRows<3>();
		
		internal::_cross(r()[0], r()[1], r()[2], _res.bottomRows<3>(), _res.topRows<3>());
		_res.topRows<3>() += R()*rhs.topRows<3>();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the inverse adjoint transformation matrix multiplied by any matrix (having six rows and arbitrary number of columns)
	//!
	//! \details 
	//!	The resulting matrix is 
	//! \f$ {\rm Adj}_T^{-1} \begin{bmatrix} U \\ L \end{bmatrix} = \begin{bmatrix} R^T ( U - \left \lceil r \right \rceil L ) \\ R^T L \end{bmatrix} \f$.
	//! 
	//! \param rhs the matrix (of size 6-by-N)
	//! \param res the resulting matrix (of size 6-by-N)
	//  ----------------------------------------------------------
	template <typename Derived, typename OtherDerived>
	inline void itransform(Eigen::MatrixBase<Derived> const & rhs, Eigen::MatrixBase<OtherDerived> const & res) const
	{
		Eigen::MatrixBase<OtherDerived> & _res = const_cast<Eigen::MatrixBase<OtherDerived> &>(res);
	
		_res.bottomRows<3>().noalias() = R().transpose()*rhs.bottomRows<3>();
	
		//_res.topRows<3>().noalias() = rhs.topRows<3>() - internal::_cross(r()[0], r()[1], r()[2], rhs.bottomRows<3>());
		internal::_cross(r()[0], r()[1], r()[2], rhs.bottomRows<3>(), _res.topRows<3>());
		_res.topRows<3>() = rhs.topRows<3>() - _res.topRows<3>();
		_res.topRows<3>() = R().transpose()*_res.topRows<3>();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the similarity transformation of mapping from twist to twist, 
	//! e.g. adjoint operator. It returns the six-dimensional square matrix defined by 
	//! \f$ {\rm Adj}_T P {\rm Adj}_T^{-1} \f$ for a given six-dimensional square matrix \f$ P \f$.
	//! 
	//! \details 	 	
	//! When the homogeneous transformation matrix T_12 stands 
	//! for the coordinate transformation from {1} to {2}, 
	//! T_12.similarity(V_23.adjoint()) generates the equivalent adjoint operator
	//! from {2} to {3} represented with respect to {1}, where 
	//! V_23 is the twist from {2} to {3}.
	//!
	//! \code{.cpp}
	//! LieGroup::HTransform T_12; 
	//! LieGroup::Twist V_23; 
	//! ...
	//! Matrix6d adj_1_23 = T_12.similarity(V_23.adj()); 
	//!   // This sets adj_1_23 = T_12.adj() * (V_23.adjoint()) * T_12.iadj()
	//! \endcode
	//!
	//! \param rhs The six-dimensional square matrix
	//!
	//! \return The six-dimensional transformed matrix
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d similarity(Matrix6d const & rhs) const
	{
		Eigen::Matrix3d A11;
		Eigen::Matrix3d A12;
		Eigen::Matrix3d A21;
		Eigen::Matrix3d A22;
		
		Eigen::Matrix3d rA11t;
		Eigen::Matrix3d rA21tr;
		Eigen::Matrix3d rA21;
		Eigen::Matrix3d rA21t;
		Eigen::Matrix3d rA22;

		A11.noalias() = R()*rhs.topLeftCorner<3, 3>()*R().transpose();
		A12.noalias() = R()*rhs.topRightCorner<3, 3>()*R().transpose();
		A21.noalias() = R()*rhs.bottomLeftCorner<3, 3>()*R().transpose();
		A22.noalias() = R()*rhs.bottomRightCorner<3, 3>()*R().transpose();

		rA21 << r().cross(A21.col(0)), r().cross(A21.col(1)), r().cross(A21.col(2));
		rA21t << r().cross(A21.row(0)), r().cross(A21.row(1)), r().cross(A21.row(2));
		rA11t << r().cross(A11.row(0)), r().cross(A11.row(1)), r().cross(A11.row(2));
		rA22 << r().cross(A22.col(0)), r().cross(A22.col(1)), r().cross(A22.col(2));
		rA21tr << r().cross(rA21t.row(0)) , r().cross(rA21t.row(1)), r().cross(rA21t.row(2));
		
		Matrix6d res;

		res.topLeftCorner<3, 3>() = A11 + rA21;
		res.topRightCorner<3, 3>() = A12 + rA22 + rA11t.transpose() + rA21tr;
		res.bottomLeftCorner<3, 3>() = A21;
		res.bottomRightCorner<3, 3>() = A22 + rA21t.transpose();

		return res;
	}

// 	//  ---------------------- Doxygen info ----------------------
// 	//! \fn Matrix6d isimilarity(Matrix6d const & rhs) const
// 	//! 
// 	//! \brief
// 	//! Generates the similarity transformation of mapping from twist to twist, 
// 	//! e.g. adjoint operator
// 	//! 
// 	//! \details 	 	
// 	//! When the homogeneous transformation matrix T_12 stands 
// 	//! for the coordinate transformation from {1} to {2}, 
// 	//! T_12.similarity(T_13.adj()) generates the equivalent adjoint transformation 
// 	//! from {1} to {3} represented with respect to {2}, where 
// 	//! T_13 is the coordinate transformation from {1} to {3}.
// 	//!
// 	//! LieGroup::HTransform T_12; 
// 	//! LieGroup::HTransform T_13; 
// 	//! ...
// 	//! Matrix6d adj_2_13 = T_12.isimilarity(T_13.adj()); 
// 	//!   // This sets adj_2_13 = T_12.iadj() * (T_13.adj()) * T_12.adj()
// 	//!
// 	//! \return The transformed mapping matrix
// 	//!
// 	//! \sa Matrix6d
// 	//  ----------------------------------------------------------
// 	//inline Matrix6d isimilarity(Matrix6d const & rhs) const;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Generates the co-similarity transformation of mapping from wrench to wrench, 
	//! e.g. coadjoint operator. It returns the six-dimensional square matrix defined by 
	//! \f$ {\rm Adj}_T^{-T} P {\rm Adj}_T^{T} \f$ for a given six-dimensional square matrix \f$ P \f$.
	//! 
	//! \details 	 	
	//! When the homogeneous transformation matrix T_12 stands 
	//! for the coordinate transformation from {1} to {2}, 
	//! T_12.cosimilarity(V_23.coadjoint()) generates the equivalent coadjoint operator
	//! from {2} to {3} represented with respect to {1}, where 
	//! V_23 is the twist from {2} to {3}.
	//!
	//! \code{.cpp}
	//! LieGroup::HTransform T_12; 
	//! LieGroup::Twist V_23; 
	//! ...
	//! Matrix6d coadj_1_23 = T_12.cosimilarity(V_23.coadjoint()); 
	//!   // This sets coadj_1_23 = T_12.tiadj() * (V_23.coadjoint()) * T_12.tadj()
	//! \endcode
	//!
	//! \param rhs The six-dimensional square matrix
	//!
	//! \return The six-dimensional transformed mapping matrix
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d cosimilarity(Matrix6d const & rhs) const
	{
		Eigen::Matrix3d A11;
		Eigen::Matrix3d A12;
		Eigen::Matrix3d A21;
		Eigen::Matrix3d A22;
		
		Eigen::Matrix3d rA12t;
		Eigen::Matrix3d rA12;
		Eigen::Matrix3d rA11;
		Eigen::Matrix3d rA22t;
		Eigen::Matrix3d rA12tr;

		A11.noalias() = R()*rhs.topLeftCorner<3, 3>()*R().transpose();
		A12.noalias() = R()*rhs.topRightCorner<3, 3>()*R().transpose();
		A21.noalias() = R()*rhs.bottomLeftCorner<3, 3>()*R().transpose();
		A22.noalias() = R()*rhs.bottomRightCorner<3, 3>()*R().transpose();

		// FIXME Check whether each cross product results in column vector
		rA12t << r().cross(A12.row(0)), r().cross(A12.row(1)), r().cross(A12.row(2));
		rA12 << r().cross(A12.col(0)), r().cross(A12.col(1)), r().cross(A12.col(2));
		rA11 << r().cross(A11.col(0)), r().cross(A11.col(1)), r().cross(A11.col(2));
		rA22t << r().cross(A22.row(0)), r().cross(A22.row(1)), r().cross(A22.row(2));
		rA12tr << r().cross(rA12t.row(0)) , r().cross(rA12t.row(1)), r().cross(rA12t.row(2));
		
		Matrix6d res;

		res.topLeftCorner<3, 3>() = A11 + rA12t.transpose();
		res.topRightCorner<3, 3>() = A12;
		res.bottomLeftCorner<3, 3>() = A21 + rA11 + rA22t.transpose() + rA12tr;
		res.bottomRightCorner<3, 3>() = A22 + rA12;

		return res;
	}

// 	//  ---------------------- Doxygen info ----------------------
// 	//! \fn Matrix6d icosimilarity(Matrix6d const & rhs) const
// 	//! 
// 	//! \brief
// 	//! Generates the co-similarity transformation of mapping from wrench to wrench, 
// 	//! e.g. coadjoint operator (such as tadj() and tiadj())
// 	//! 
// 	//! \details 	 	
// 	//! When the homogeneous transformation matrix T_12 stands 
// 	//! for the coordinate transformation from {1} to {2}, 
// 	//! T_12.icosimilarity(T_13.tiadj()) generates the equivalent coadjoint transformation 
// 	//! from {1} to {3} represented with respect to {2}, where 
// 	//! T_13 is the coordinate transformation from {1} to {3}.
// 	//!
// 	//! LieGroup::HTransform T_12; 
// 	//! LieGroup::HTransform T_13; 
// 	//! ...
// 	//! Matrix6d coadj_2_13 = T_12.similarity(T_13.tiadj()); 
// 	//!   // This sets coadj_2_13 = T_12.tadj() * (T_13.tiadj()) * T_12.tiadj()
// 	//!
// 	//! \return The transformed mapping matrix
// 	//!
// 	//! \sa Matrix6d
// 	//  ----------------------------------------------------------
// 	//inline Matrix6d icosimilarity(Matrix6d const & rhs) const;

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the congruence transformation of mapping from twist to wrench, 
	//! e.g. inertia. It returns the six-dimensional square matrix defined by 
	//! \f$ {\rm Adj}_T^{-T} P {\rm Adj}_T^{-1} \f$ for a given six-dimensional square matrix \f$ P \f$.
	//! 
	//! \details 	 	
	//! When the homogeneous transformation matrix T_12 stands 
	//! for the coordinate transformation from {1} to {2}, 
	//! T_12.congruence(M_2) generates the effective inertia of M_2 
	//! (represented wrt {2}) with respect to {1}.
	//!
	//! \code{.cpp}
	//! LieGroup::HTransform T_12; 
	//! Matrix6d M_2; 
	//! ...
	//! Matrix6d M_1_2 = T_12.congruence(M_2); 
	//!   // This sets M_1_2 = T_12.tiadj() * M_2 * T_12.iadj()
	//! \endcode
	//!
	//! \param rhs The six-dimensional square matrix
	//!
	//! \return The six-dimensional transformed mapping matrix
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d congruence(Matrix6d const & rhs) const
	{
		Eigen::Matrix3d A11;
		Eigen::Matrix3d A12;
		Eigen::Matrix3d A22;
		Eigen::Matrix3d rA11;
		Eigen::Matrix3d rA11r;
		Eigen::Matrix3d rA12;

		A11.noalias() = R()*rhs.topLeftCorner<3, 3>().selfadjointView<Eigen::Upper>()*R().transpose();
		A12.noalias() = R()*rhs.topRightCorner<3, 3>()*R().transpose();
		A22.noalias() = R()*rhs.bottomRightCorner<3, 3>().selfadjointView<Eigen::Upper>()*R().transpose();

		// FIXME Check whether each cross product results in column vector
		rA11 << r().cross(A11.col(0)), r().cross(A11.col(1)), r().cross(A11.col(2));
		rA11r << r().cross(rA11.row(0)) , r().cross(rA11.row(1)), r().cross(rA11.row(2));
		rA12 << r().cross(A12.col(0)), r().cross(A12.col(1)), r().cross(A12.col(2));

		Matrix6d res;

		res.topLeftCorner<3, 3>() = A11;
		res.bottomLeftCorner<3, 3>() = A12.transpose() + rA11;
		res.bottomRightCorner<3, 3>() = A22 + rA12 + rA12.transpose() + rA11r;

		res = res.selfadjointView<Eigen::Lower>();

		return res;
	}

// 	//  ---------------------- Doxygen info ----------------------
// 	//! \fn Matrix6d icongruence(Matrix6d const & rhs) const
// 	//! 
// 	//! \brief
// 	//! Generates the congruence transformation of mapping from twist to wrench, 
// 	//! e.g. inertia
// 	//! 
// 	//! \details 	 	
// 	//! When the homogeneous transformation matrix T_12 stands 
// 	//! for the coordinate transformation from {1} to {2}, 
// 	//! T_12.icongruence(M_1) generates the effective inertia of M_1 
// 	//! (represented wrt {1}) with respect to {2}.
// 	//!
// 	//! LieGroup::HTransform T_12; 
// 	//! Matrix6d M_1; 
// 	//! ...
// 	//! Matrix6d M_1_2 = T_12.icongruence(M_1); 
// 	//!   // This sets M_1_2 = T_12.tadj() * M_1 * T_12.tiadj()
// 	//!
// 	//! \return The transformed mapping matrix
// 	//!
// 	//! \sa Matrix6d
// 	//  ----------------------------------------------------------
// 	//inline Matrix6d icongruence(Matrix6d const & rhs) const;

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the co-congruence transformation of mapping from wrench to twist, 
	//! e.g. inverse inertia. It returns the six-dimensional square matrix defined by 
	//! \f$ {\rm Adj}_T P {\rm Adj}_T^{T} \f$ for a given six-dimensional square matrix \f$ P \f$.
	//! 
	//! \details 	 	
	//! When the homogeneous transformation matrix T_12 stands 
	//! for the coordinate transformation from {1} to {2}, 
	//! T_12.cocongruence(Lambda_2) generates the effective inverse inertia of Lambda_2 
	//! (represented wrt {2}) with respect to {1}.
	//!
	//! \code{.cpp}
	//! LieGroup::HTransform T_12; 
	//! Matrix6d Lambda_2; 
	//! ...
	//! Matrix6d Lambda_1_2 = T_12.cocongruence(Lambda_2); 
	//!   // This sets Lambda_1_2 = T_12.adj() * M_2 * Lambda_2.tadj()
	//! \endcode
	//!
	//! \param rhs The six-dimensional square matrix
	//!
	//! \return The six-dimensional transformed mapping matrix
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d cocongruence(Matrix6d const & rhs) const
	{
		Eigen::Matrix3d A11;
		Eigen::Matrix3d A21;
		Eigen::Matrix3d A22;
		Eigen::Matrix3d rA22;
		Eigen::Matrix3d rA22r;
		Eigen::Matrix3d rA21;

		A11.noalias() = R()*rhs.topLeftCorner<3, 3>().selfadjointView<Eigen::Upper>()*R().transpose();
		A21.noalias() = R()*rhs.bottomLeftCorner<3, 3>()*R().transpose();
		A22.noalias() = R()*rhs.bottomRightCorner<3, 3>().selfadjointView<Eigen::Upper>()*R().transpose();

		rA21 << r().cross(A21.col(0)), r().cross(A21.col(1)), r().cross(A21.col(2));
		rA22 << r().cross(A22.col(0)), r().cross(A22.col(1)), r().cross(A22.col(2));
		rA22r << r().cross(rA22.row(0)) , r().cross(rA22.row(1)), r().cross(rA22.row(2));
		
		Matrix6d res;

		res.topLeftCorner<3, 3>() = A11 + rA21 + rA21.transpose() + rA22r;
		res.bottomLeftCorner<3, 3>() = A21 + rA22.transpose();
		res.bottomRightCorner<3, 3>() = A22;

		res = res.selfadjointView<Eigen::Lower>();

		return res;
	}

// 	//  ---------------------- Doxygen info ----------------------
// 	//! \fn Matrix6d icocongruence(Matrix6d const & rhs) const
// 	//! 
// 	//! \brief
// 	//! Generates the cocongruence transformation of mapping from wrench to twist, 
// 	//! e.g. inverse inertia
// 	//! 
// 	//! \details 	 	
// 	//! When the homogeneous transformation matrix T_12 stands 
// 	//! for the coordinate transformation from {1} to {2}, 
// 	//! T_12.icocongruence(Lambda_1) generates the effective inverse inertia of Lambda_1 
// 	//! (represented wrt {1}) with respect to {2}.
// 	//!
// 	//! LieGroup::HTransform T_12; 
// 	//! Matrix6d Lambda_1; 
// 	//! ...
// 	//! Matrix6d Lambda_2_1 = T_12.icocongruence(Lambda_2); 
// 	//!   // This sets Lambda_2_1 = T_12.iadj() * M_2 * Lambda_2.tiadj()
// 	//!
// 	//! \return The transformed mapping matrix
// 	//!
// 	//! \sa Matrix6d
// 	//  ----------------------------------------------------------
// 	//inline Matrix6d icocongruence(Matrix6d const & rhs) const;

 	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the adjoint transformation matrix, defined by 
	//! \f$ {\rm Adj}_T = \begin{bmatrix} R & \left \lceil r \right \rceil R \\ 0 & R \end{bmatrix} \f$
	//! 
	//! \details 
	//! When the homogeneous transformation matrix T stands 
	//! for the coordinate transformation from {ref} to {tar}, 
	//! T.adj() transforms the twist represented wrt {tar}
	//! to the equivalent twist represented wrt {ref}.
	//!
	//! \return six-dimensional adjoint transformation matrix
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d adj() const 
	{
		Matrix6d res;
		res.topLeftCorner<3, 3>() = R(); 
		res.topRightCorner<3, 3>() << r().cross(x()), r().cross(y()), r().cross(z()); 
		res.bottomLeftCorner<3, 3>().setZero();
		res.bottomRightCorner<3, 3>() = R(); 

		return res;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the inverse adjoint transformation matrix, defined by 
	//! \f$ {\rm Adj}_T^{-1} = \begin{bmatrix} R^T & -R^T \left \lceil r \right \rceil \\ 0 & R^T \end{bmatrix} \f$
	//! 
	//! \details 
	//! When the homogeneous transformation matrix T stands 
	//! for the coordinate transformation from {ref} to {tar}, 
	//! T.iadj() transforms the twist represented wrt {ref}
	//! to the equivalent twist represented wrt {tar}.
	//!
	//! \return six-dimensional adjoint transformation matrix
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d iadj() const
	{
		Matrix6d res;
		res.topLeftCorner<3, 3>()= R().transpose(); 
		res.topRightCorner<3, 3>().transpose() << r().cross(x()), r().cross(y()), r().cross(z()); 
		res.bottomLeftCorner<3, 3>().setZero();
		res.bottomRightCorner<3, 3>() = R().transpose(); 

		return res;
	}
 	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the transposed adjoint transformation matrix, defined by 
	//! \f$ {\rm Adj}_T^{T} = \begin{bmatrix} R^T & 0 \\ -R^T \left \lceil r \right \rceil & R^T \end{bmatrix} \f$
	//! 
	//! \details 
	//! When the homogeneous transformation matrix T stands 
	//! for the coordinate transformation from {ref} to {tar}, 
	//! T.tadj() transforms the wrench represented wrt {ref}
	//! to the equivalent wrench represented wrt {tar}.
	//!
	//! \return six-dimensional transposed adjoint transformation matrix
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d tadj() const
	{
		Matrix6d res;
		res.topLeftCorner<3, 3>() = R().transpose(); 
		res.topRightCorner<3, 3>().setZero();
		res.bottomLeftCorner<3, 3>().transpose() << r().cross(x()), r().cross(y()), r().cross(z()); 
		res.bottomRightCorner<3, 3>() = R().transpose(); 

		return res;
	}
 	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the inverse transposed adjoint transformation matrix, defined by 
	//! \f$ {\rm Adj}_T^{-T} = \begin{bmatrix} R & 0 \\ \left \lceil r \right \rceil R  & R \end{bmatrix} \f$
	//! 
	//! \details 
	//! When the homogeneous transformation matrix T stands 
	//! for the coordinate transformation from {ref} to {tar}, 
	//! T.tadj() transforms the wrench represented wrt {tar}
	//! to the equivalent wrench represented wrt {ref}.
	//!
	//! \return six-dimensional inverse transposed adjoint transformation matrix
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d tiadj() const
	{
		Matrix6d res;
		res.topLeftCorner<3, 3>() = R(); 
		res.topRightCorner<3, 3>().setZero();
		res.bottomLeftCorner<3, 3>() << r().cross(x()), r().cross(y()), r().cross(z()); 

		res.bottomRightCorner<3, 3>() = R(); 

		return res;
	}
	
};

//! \sa LieGroup::Twist::exp() const
inline HTransform Twist::exp() const
{
	double alpha;
	double beta; 
	double theta_sqr;
	compute_terms(&alpha, &beta, &theta_sqr);

	return exp(alpha, beta, theta_sqr);
}

//! \sa LieGroup::Twist::exp() const
inline HTransform Twist::exp(double alpha, double beta, double theta_sqr) const
{
	Vector3D w_(w());

	// 		Eigen::Matrix4d T;
	// 		T.topLeftCorner<3,3>() = w_.exp(alpha, beta, theta_sqr);
	// 		Eigen::Matrix3d dexp = w_.dexp(alpha, beta, theta_sqr);
	// 		T.topRightCorner<3,1>() = dexp*v();
	// 
	// 		return T;

	return HTransform(w_.exp(alpha, beta, theta_sqr), Displacement(w_.dexp(alpha, beta, theta_sqr)*v()));	
}

//! \sa LieGroup::Displacement::transform()
inline void Displacement::transform(const HTransform& T)
{
	*this = T.transform(*this);	
}

//! \sa LieGroup::Displacement::itransform()
inline void Displacement::itransform(const HTransform& T)
{
	*this = T.itransform(*this);	
}

//! \sa LieGroup::Vector3D::transform()
inline void Vector3D::transform(const HTransform& T)
{
	*this = T.transform(*this);	
}

//! \sa LieGroup::Vector3D::itransform()
inline void Vector3D::itransform(const HTransform& T)
{
	*this = T.itransform(*this);	
}

//! \sa LieGroup::Twist::transform()
inline void Twist::transform(const HTransform& T)
{
	*this = T.transform(*this);	
}

//! \sa LieGroup::Twist::itransform()
inline void Twist::itransform(const HTransform& T)
{
	*this = T.itransform(*this);	
}

//! \sa LieGroup::Wrench::transform()
inline void Wrench::transform(const HTransform& T)
{
	*this = T.transform(*this);	
}

//! \sa LieGroup::Wrench::itransform()
inline void Wrench::itransform(const HTransform& T)
{
	*this = T.itransform(*this);	
}

//! \sa LieGroup::Wrench::applyAt()
inline Wrench Wrench::applyAt(HTransform const & T) const
{
	return T.transform(*this);
}

//! \sa LieGroup::Wrench::applyAt()
inline Wrench Wrench::applyAt(Displacement const & r) const
{
	return Wrench(f(), n() +  r.cross(f()));
}

}
