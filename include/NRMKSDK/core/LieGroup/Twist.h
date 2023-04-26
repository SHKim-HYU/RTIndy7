//  ---------------------- Doxygen info ----------------------
//! \file Twist.h
//!
//! \brief
//! Header file for the class Twist (API of the LieGroup Libraries)
//!
//! \details
//! This file implements a three-dimensional twist 
//! (6D vector comprised of linear and angular velocity) class
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

class Vector3D;
class Rotation;
class HTransform;
class Wrench;

//  ---------------------- Doxygen info ----------------------
//! \class Twist
//!
//! \brief
//! This implements a three-dimensional twist which is a six-dimensional vector
//! cascading linear and angular velocity (both of type the three-dimensional vector).
//! This also represents six-dimensional Lie algebra elements associated with the homogeneous transformation matrices. 
//! 
//! \details
//! For two three-dimensional vector \f$ v \f$ and \f$ \omega \f$ representing 
//! the linear and angular velocity, respectively, the twist is defined by 
//! \f$ V = \begin{bmatrix} v \\ \omega \end{bmatrix} \f$. When it stands for the Lie algebra element, 
//! we use the notation \f$ \lambda \f$, and it is denoted by \f$ \lambda = \begin{bmatrix} \eta \\ \xi \end{bmatrix} \f$.
//! 
//! \note
//! Linear part always precedes angular part.
//! Compared to Vector6d, it is initialized properly.
//! 
//! \sa Vector6d
//  ----------------------------------------------------------
class Twist : public Vector6d
{
public:
	RMATH_EIGEN_DERIVE(Vector6d, Twist)

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the zero twist
	//  ----------------------------------------------------------
	inline Twist() : Vector6d(Vector6d::Zero()) 
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the identity element
	//  ----------------------------------------------------------
	inline static  Twist const Zero() 
	{
		return Twist();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the twist in terms of six elements by
	//! \f$ V = \begin{bmatrix} x \\ y \\ z \\ u \\ v \\ w \end{bmatrix} \f$.
	//!
	//! \param x x-component of the linear velocity
	//! \param y y-component of the linear velocity
	//! \param z z-component of the linear velocity
	//! \param u x-component of the angular velocity
	//! \param v y-component of the angular velocity
	//! \param w z-component of the angular velocity
	//  ----------------------------------------------------------
	inline Twist(double x, double y, double z, double u, double v, double w) 
	{ 
		set(x, y, z, u, v, w); 
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the twist in terms of linear and angular velocity vectors (both of type Vector3D)
	//! by \f$ V = \begin{bmatrix} v \\ w \end{bmatrix} \f$.
	//!
	//! \param v linear velocity vector
	//! \param w angular velocity vector 
	//  ----------------------------------------------------------
	inline Twist(Vector3D const & v, Vector3D const & w) 
	{ 
		set(v, w); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the twist in terms of any three-dimensional vectors
	//! by \f$ V = \begin{bmatrix} v \\ w \end{bmatrix} \f$.
	//!
	//! \param v linear velocity
	//! \param w angular velocity
	//  ----------------------------------------------------------
	template<typename Derived1, typename Derived2>
	inline Twist(Eigen::VectorBlock<Derived1, 3> const & v, Eigen::VectorBlock<Derived2, 3> const & w)
	{ 
		set(v, w); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the linear velocity block for writing
	//  ----------------------------------------------------------
	inline Eigen::VectorBlock<Vector6d, 3> v() 
	{ 
		return head<3>(); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the linear velocity block for reading
	//  ----------------------------------------------------------
	inline const Eigen::VectorBlock<const Vector6d, 3> v() const 
	{ 
		return head<3>();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the angular velocity block for writing
	//  ----------------------------------------------------------
	inline Eigen::VectorBlock<Vector6d, 3> w() 
	{
		return tail<3>();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the angular velocity block for reading
	//  ----------------------------------------------------------
	inline const Eigen::VectorBlock<const Vector6d, 3> w() const 
	{ 
		return tail<3>(); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the twist in terms of six elements by
	//! \f$ V = \begin{bmatrix} x \\ y \\ z \\ u \\ v \\ w \end{bmatrix} \f$.
	//!
	//! \param x x-component of the linear velocity
	//! \param y y-component of the linear velocity
	//! \param z z-component of the linear velocity
	//! \param u x-component of the angular velocity
	//! \param v y-component of the angular velocity
	//! \param w z-component of the angular velocity
	//  ----------------------------------------------------------
	inline void set(double x, double y, double z, double u, double v, double w) 
	{ 
		*this << x, y, z, u, v, w; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the twist in terms of linear and angular velocity (both of type Vector3D)
	//! by \f$ V = \begin{bmatrix} v \\ w \end{bmatrix} \f$.
	//!
	//! \param v linear velocity vector
	//! \param w angular velocity vector 
	//  ----------------------------------------------------------
	inline void set(Vector3D const & v, Vector3D const & w) 
	{ 
		head<3>() = v; 
		tail<3>() = w; 
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! constructs the twist in terms of any three-dimensional vectors
	//! by \f$ V = \begin{bmatrix} v \\ w \end{bmatrix} \f$.
	//!
	//! \param v linear velocity
	//! \param w angular velocity
	//  ----------------------------------------------------------
	template<typename Derived1, typename Derived2>
	inline void set(Eigen::VectorBlock<Derived1, 3> const & v, Eigen::VectorBlock<Derived2, 3> const & w)
	{ 
		head<3>() = v; 
		tail<3>() = w; 
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the homogeneous representation of the twist (a.k.a. ceiled matrix), 
	//! denoted by \f$ \left\lceil V \right\rceil \f$, of the twist by ceil operator
	//!
	//!	\details
	//! The matrix is of the form 
	//! \f$ \left\lceil V \right\rceil = \begin{bmatrix} \left\lceil \omega \right\rceil & v \\ 0 & 0 \end{bmatrix} \f$. 
	//!
	//! \return The 4-by-4 matrix
	//!
	//! \sa Eigen::Matrix4d
	//  ----------------------------------------------------------
	inline Eigen::Matrix4d ceil() const 
	{
		Eigen::Matrix4d res;

		res.topLeftCorner<3,3>() = internal::_ceil_expression(coeff(3), coeff(4), coeff(5));
		res.col(3).head<3>() = head<3>();
		res.row(3).setZero();

		return res;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the linear velocity at a (body-fixed) point \f$ p \f$
	//! defined by \f$ v + \omega \times p \f$, 
	//! 
	//! \return linear velocity
	//!
	//! \sa Liegroup::Vector3D
	//  ----------------------------------------------------------
	inline LieGroup::Vector3D velocityAt(LieGroup::Displacement const & p) const
	{ 
		return v() + w().cross(p); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the adjoint operator of the twist (in six-dimensional square matrix), denoted by \f$ {\rm adj}_V \f$,
	//! which is equivalent to the cross-product matrix of Vector3D objects
	//!
	//! \details
	//! The resulting adjoint operator matrix is of the form 
	//! \f$ {\rm adj}_V = \begin{bmatrix}  \left \lceil \omega \right \rceil &  \left \lceil v \right \rceil \\ 0 &  \left \lceil \omega \right \rceil \end{bmatrix} \f$.
	//! 
	//! \note
	//! This operates on twists.
	//!
	//! \return The adjoint operator matrix (in six-dimensional square matrix)
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d adjoint() const
	{
		Matrix6d res;

		res.topLeftCorner<3,3>() = internal::_ceil_expression(coeff(3), coeff(4), coeff(5));
		res.topRightCorner<3,3>() = internal::_ceil_expression(coeff(0), coeff(1), coeff(2));
		res.bottomLeftCorner<3,3>().setZero();
		res.bottomRightCorner<3,3>() = internal::_ceil_expression(coeff(3), coeff(4), coeff(5));
		
		return res;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the coadjoint operator of the twist (in six-dimensional square matrix), defined by \f$ -{\rm adj}_V^T \f$
	//!
	//! \details
	//! The resulting coadjoint operator matrix is of the form 
	//! \f$ {\rm adj}_V = \begin{bmatrix}  \left \lceil \omega \right \rceil & 0 \\ \left \lceil v \right \rceil & \left \lceil \omega \right \rceil \end{bmatrix} \f$.
	//!
	//! \note 
	//! This operates on wrenches.
	//!
	//! \return The coadjoint operator matrix (in six-dimensional square matrix)
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d coadjoint() const
	{
		Matrix6d res;

		res.topLeftCorner<3,3>() = internal::_ceil_expression(coeff(3), coeff(4), coeff(5));
		res.topRightCorner<3,3>().setZero(); 
		res.bottomLeftCorner<3,3>() = internal::_ceil_expression(coeff(0), coeff(1), coeff(2));;
		res.bottomRightCorner<3,3>() = internal::_ceil_expression(coeff(3), coeff(4), coeff(5));
		
		return res;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the adjoint product of the twist to a given twist
	//! which is equivalent to the cross-product matrix of Vector3D objects
	//!
	//! \details
	//! \code{.cpp}
	//! LieGroup::Twist V1;
	//! LieGroup::Twist V2;
	//!
	//! LieGroup::Twist V3 = V1.adjoint(V2); // V3 = V1.adjoint()*V2;
	//! \endcode
	//!
	//! \param V the twist
	//!
	//! \return Twist
	//!
	//! \sa adjoint()
	//  ----------------------------------------------------------
	inline Twist adjoint(Twist const & V) const
	{
		return Twist(w().cross(V.v()) + v().cross(V.w()), w().cross(V.w()));
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the adjoint matrix multiplied by any matrix (having six rows and arbitrary number of columns)
	//!
	//! \details 
	//!	The resulting matrix is 
	//! \f$ {\rm adj}_V \begin{bmatrix} U \\ L \end{bmatrix} = \begin{bmatrix} \left \lceil \omega \right \rceil U + \left \lceil v \right \rceil L \\ \left \lceil \omega \right \rceil L \end{bmatrix} \f$.
	//! 
	//! \param rhs the matrix (of size 6-by-N)
	//! \param res the resulting matrix (of size 6-by-N)
	//  ----------------------------------------------------------
	template <typename Derived, typename OtherDerived>
	inline void adjoint(Eigen::MatrixBase<Derived> const & rhs, Eigen::MatrixBase<OtherDerived> const & res) const
	{
		Eigen::MatrixBase<OtherDerived> & _res = const_cast<Eigen::MatrixBase<OtherDerived> &>(res);

		internal::_cross(w()[0], w()[1], w()[2], rhs.topRows<3>(), _res.topRows<3>());
		internal::_cross(v()[0], v()[1], v()[2], rhs.bottomRows<3>(), _res.bottomRows<3>());
		_res.topRows<3>() += _res.bottomRows<3>();

		internal::_cross(w()[0], w()[1], w()[2], rhs.bottomRows<3>(), _res.bottomRows<3>());
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the co-adjoint product of the twist to a given wrench
	//! which is equivalent to the cross-product matrix of Vector3D objects
	//!
	//! \details
	//! \code{.cpp}
	//! LieGroup::Twist V1;
	//! LieGroup::Wrench F1;
	//!
	//! LieGroup::Wrench F2 = V1.coadjoint(F1); // F2 = V1.coadjoint()*F1;
	//! \endcode
	//!
	//! \param F the wrench
	//!
	//! \return Wrench
	//!
	//! \sa adjoint()
	//  ----------------------------------------------------------
	inline Wrench coadjoint(Wrench const & F) const;

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the coadjoint matrix multiplied by any matrix (having six rows and arbitrary number of columns)
	//!
	//! \details 
	//!	The resulting matrix is 
	//! \f$ {\rm adj}_V^* \begin{bmatrix} U \\ L \end{bmatrix} = \begin{bmatrix} \left \lceil \omega \right \rceil U \\ \left \lceil v \right \rceil U + \left \lceil \omega \right \rceil L \end{bmatrix} \f$.
	//! 
	//! \param rhs the matrix (of size 6-by-N)
	//! \param res the resulting matrix (of size 6-by-N)
	//  ----------------------------------------------------------
	template <typename Derived, typename OtherDerived>
	inline void coadjoint(Eigen::MatrixBase<Derived> const & rhs, Eigen::MatrixBase<OtherDerived> const & res) const
	{
		Eigen::MatrixBase<OtherDerived> & _res = const_cast<Eigen::MatrixBase<OtherDerived> &>(res);

		internal::_cross(w()[0], w()[1], w()[2], rhs.bottomRows<3>(), _res.bottomRows<3>());
		internal::_cross(v()[0], v()[1], v()[2], rhs.topRows<3>(), _res.topRows<3>());
		_res.bottomRows<3>() += _res.topRows<3>();

		internal::_cross(w()[0], w()[1], w()[2], rhs.topRows<3>(), _res.topRows<3>());
	}

	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! compute the terms \f$ \alpha \f$, \f$ \beta \f$, and \f$ \theta^2 \f$ for the twist \f$ \lambda \f$
	//! to compute exponential map family in order to prevent repeated computations
	//!
	//! \details
	//! The parameters are defined as: \f$ \alpha = s c \f$, \f$ \beta = s^2 \f$, and \f$ \theta = \left\| \xi \right\| \f$
	//! where \f$ s = \frac{\sin(\theta/2)}{\theta/2} \f$ and \f$ c = \cos(\theta/2) \f$.
	//! They depend only on the angular velocity part, i.e. \f$ \xi \f$.
	//!
	//! \param alpha alpha value or \f$ \alpha \f$
	//! \param beta beta value or \f$ \beta \f$
	//! \param theta_sqr the squared norm of theta \f$ \theta^2 \f$
	//!
	//! \sa LieGroup::Vector3D::compute_terms()
	//  ----------------------------------------------------------
	inline void compute_terms(double* alpha, double* beta, double* theta_sqr) const
	{
		Vector3D(w()).compute_terms(alpha, beta, theta_sqr);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \fn Eigen::Matrix4d exp() const 
	//! 
	//! \brief
	//! generates the homogeneous transformation matrix of the twist by exponential map on \f$ SE(3) \f$.
	//! That is, it returns \f$ \exp(\lambda) \f$ for the twist \f$ \lambda \f$.
	//!
	//! \return The homogeneous transformation matrix
	//!
	//! \sa LieGroup::HTransform
	//  ----------------------------------------------------------
	inline HTransform exp() const;

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the homogeneous transformation matrix of the twist by exponential map  on \f$ SE(3) \f$
	//! in terms of the precomputed parameters. That is, it returns \f$ \exp(\lambda) \f$ for the twist \f$ \lambda \f$.
	//!
	//! \details
	//! The parameters should computed by compute_terms(). 
	//! They can be reused to compute exponential map family.
	//!
	//! \param alpha alpha value
	//! \param beta beta value
	//! \param theta_sqr theta_sqr value
	//!
	//! \return The homogeneous transformation matrix
	//!
	//! \sa LieGroup::HTransform
	//  ----------------------------------------------------------
	inline HTransform exp(double alpha, double beta, double theta_sqr) const;

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the differential-of-exponential matrix on \f$ SE(3) \f$ of the twist.
	//! That is, it returns \f$ {\rm dexp}_{\lambda} \f$ for the twist \f$ \lambda \f$.
	//!
	//! \details
	//! The matrix is denoted by \f$ {\rm dexp}_{\lambda} \f$ satisfies the relationship 
	//!   \f$ V = {\rm dexp}_{-\lambda} \dot{\lambda} \f$, where \f$ \lambda \f$ is the twist,
	//! where \f$ V \f$ is the twist of the transformation matrix such that 
	//!   \f$ \left \lceil V \right \rceil = \exp(\lambda)^{-1} \frac{d}{dt} \exp(\lambda) \f$. 
	//! It is defined by
	//! \f$ {\rm dexp}_{\lambda} = \begin{bmatrix} {\rm dexp}_{\xi} & C_{\xi}(\eta) \\ 0 & {\rm dexp}_{\xi} \end{bmatrix} \f$.
	//! Note the minus sign in computing \f$ {\rm dexp}_{-\lambda} \f$. It is worth noting that 
	//! \f$ {\rm dexp}_{-\lambda} = \begin{bmatrix} {\rm dexp}_{-\xi} & C_{-\xi}(-\eta) \\ 0 & {\rm dexp}_{-\xi} \end{bmatrix} = \begin{bmatrix} {\rm dexp}_{\xi}^T & C_{\xi}^T(\eta) \\ 0 & {\rm dexp}_{\xi}^T \end{bmatrix}\f$.
	//!
	//! \return The 6-by-6 differential-of-exponential matrix 
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d dexp() const
	{
		double alpha;
		double beta; 
		double theta_sqr;
		compute_terms(&alpha, &beta, &theta_sqr);

		return dexp(alpha, beta, theta_sqr);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the differential-of-exponential matrix on \f$ SE(3) \f$ of the twist
	//! in terms of the precomputed parameters. 
	//! That is, it returns \f$ {\rm dexp}_{\lambda} \f$ for the twist \f$ \lambda \f$.
	//!
	//! \details
	//! The parameters should computed by compute_terms(). 
	//! They can be reused to compute exponential map family.
	//!
	//! \param alpha alpha value
	//! \param beta beta value
	//! \param theta_sqr square of the theta
	//!
	//! \return The 6-by-6 (differential-of-exponential) matrix 
	//!
	//! \sa Matrix6d
	//! \sa compute_terms()
	//  ----------------------------------------------------------
	inline Matrix6d dexp(double alpha, double beta, double theta_sqr) const
	{
		Matrix6d res;

		Vector3D w_(w());
		Eigen::Matrix3d de = w_.dexp(alpha, beta, theta_sqr);
		Eigen::Matrix3d ddt_de = w_.ddt_dexp(alpha, beta, theta_sqr, w().dot(v()), v());

		res << de, ddt_de, Eigen::Matrix3d::Zero(), de;

		return res;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the inverse of differential-of-exponential matrix on \f$ SE(3) \f$ of the twist.
	//! That is, it returns \f$ {\rm dexp}_{\lambda}^{-1} \f$ for the twist \f$ \lambda \f$.
	//!
	//! \details
	//! The matrix always exists because the differential-of-exponential matrix is always invertible. 
	//!
	//! \return The 6-by-6 inverse of differential-of-exponential matrix 
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d dexp_inv() const
	{
		double alpha;
		double beta; 
		double theta_sqr;
		compute_terms(&alpha, &beta, &theta_sqr);

		return dexp_inv(alpha, beta, theta_sqr);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the inverse of differential-of-exponential matrix on \f$ SE(3) \f$ of the twist
	//! in terms of the precomputed parameters.
	//! That is, it returns \f$ {\rm dexp}_{\lambda}^{-1} \f$ for the twist \f$ \lambda \f$.
	//!
	//! \details
	//! The parameters should computed by compute_terms(). 
	//! They can be reused to compute exponential map family.
	//!
	//! \param alpha alpha value
	//! \param beta beta value
	//! \param theta_sqr square of the theta
	//!
	//! \return The 6-by-6 inverse of differential-of-exponential matrix 
	//!
	//! \sa Matrix6d
	//! \sa compute_terms()
	//  ----------------------------------------------------------
	inline Matrix6d dexp_inv(double alpha, double beta, double theta_sqr) const
	{	
		Vector3D w_(w());
		Eigen::Matrix3d de_inv = w_.dexp_inv(alpha, beta, theta_sqr);
		Eigen::Matrix3d ddt_de_inv = w_.ddt_dexp_inv(alpha, beta, theta_sqr, w().dot(v()), v());

		Matrix6d res;
		res << de_inv, ddt_de_inv, Eigen::Matrix3d::Zero(), de_inv;

		return res;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the time-derivative of the differential-of-exponential matrix on \f$ SO(3) \f$ of the twist \f$ \lambda \f$
	//! for the given time-derivative of the twist \f$ \dot{\lambda} \f$. That is, it returns 
	//! \f$ \frac{d}{dt} {\rm dexp}_{\lambda} (\dot{\lambda}) \f$.
	//!
	//! \param lambda_dot time-derivative of the twist or  \f$ \dot{\lambda} \f$
	//!
	//! \return The 6-by-6 time-derivative of the differential-of-exponential matrix 
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	template<typename OtherDerived>
	inline Matrix6d ddt_dexp(Eigen::MatrixBase<OtherDerived> const & lambda_dot) const
	{
		double alpha;
		double beta; 
		double theta_sqr;
		compute_terms(&alpha, &beta, &theta_sqr);

		const double xi_eta = w().dot(v());
		const double xi_xidot = w().dot(lambda_dot.tail<3>());
		const double xi_etadot = w().dot(lambda_dot.head<3>());
		const double xidot_eta = v().dot(lambda_dot.tail<3>());
		
		return ddt_dexp(alpha, beta, theta_sqr, xi_eta, xi_xidot, xi_etadot, xidot_eta, lambda_dot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the time-derivative of the differential-of-exponential matrix on \f$ SO(3) \f$ of the twist \f$ \lambda \f$
	//! for the given time-derivative of the twist \f$ \dot{\lambda} \f$ in terms of the precomputed parameters. 
	//! That is, it returns \f$ \frac{d}{dt} {\rm dexp}_{\lambda} (\dot{\lambda}) \f$.
	//!
	//! \details
	//! The parameters should computed by compute_terms(). 
	//! They can be reused to compute exponential map family.
	//!
	//! \param alpha alpha value
	//! \param beta beta value
	//! \param theta_sqr square of the theta
	//! \param xi_eta dot product of the linear and angular components, i.e. \f$ \xi^T \eta \f$
	//! \param xi_xidot dot product of the angular component and its derivative, i.e. \f$ \xi^T \dot{\xi} \f$
	//! \param xi_etadot dot product of the angular component and the derivative of the linear component, i.e. \f$ \xi^T \dot{\eta} \f$
	//! \param xidot_eta dot product of the derivative of angular component and the linear component, i.e. \f$ \dot{\xi}^T \eta \f$
	//! \param lambda_dot time-derivative of the twist or  \f$ \dot{\lambda} \f$
	//!
	//! \return The 6-by-6 time-derivative of the differential-of-exponential matrix 
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	template<typename OtherDerived>
	inline Matrix6d ddt_dexp(double alpha, double beta, double theta_sqr, 
		double xi_eta, double xi_xidot, double xi_etadot, double xidot_eta,  
		Eigen::MatrixBase<OtherDerived> const & lambda_dot) const		
	{
		Eigen::Matrix3d C = Vector3D(w()).ddt_dexp(alpha, beta, theta_sqr, xi_xidot, lambda_dot.tail<3>());
		Eigen::Matrix3d Cdot;

		if (theta_sqr == 0)
		{
			Cdot = internal::_common_expression_4(0.5, 1.0/6.0, w(), lambda_dot.tail<3>(), v(), lambda_dot.head<3>());
		}
		else
		{
			const double beta_2 = beta/2.0;
			const double zeta = xi_eta*xi_xidot/theta_sqr;
			const double common1 = (1.0 - alpha)/theta_sqr;
			const double common2 = (alpha - beta)/theta_sqr;
			const double common3 = (beta_2 - 3*common1)/theta_sqr;
			const double factor = xidot_eta + xi_etadot - 4*zeta;
			
			const double a = common2*factor + (common1 - beta_2)*zeta; 
			const double e = common2*zeta + common3*(factor - zeta);
			
			Cdot = 	internal::_common_expression_3(a, common2*xi_eta, common2*xi_xidot, beta_2, 
													e, common3*xi_eta, common3*xi_xidot, common1, common1, 
													w(), lambda_dot.tail<3>(), v(), lambda_dot.head<3>());			
		}

		Matrix6d res; 
		res << C, Cdot, Eigen::Matrix3d::Zero(), C;

		return res;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the time-derivative of the inverse of differential-of-exponential matrix on \f$ SO(3) \f$ of the twist \f$ \lambda \f$
	//! for the given time-derivative of the twist \f$ \dot{\lambda} \f$. That is, it returns 
	//! \f$ \frac{d}{dt} {\rm dexp}_{\lambda}^{-1} (\dot{\lambda}) \f$.
	//!
	//! \param lambda_dot time-derivative of the twist or \f$ \dot{\lambda} \f$
	//!
	//! \return The 6-by-6 time-derivative of the differential-of-exponential matrix 
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	template<typename OtherDerived>
	inline Matrix6d ddt_dexp_inv(Eigen::MatrixBase<OtherDerived> const & lambda_dot) const
	{
		double alpha;
		double beta; 
		double theta_sqr;
		compute_terms(&alpha, &beta, &theta_sqr);

		const double xi_eta = w().dot(v());
		const double xi_xidot = w().dot(lambda_dot.tail<3>());
		const double xi_etadot = w().dot(lambda_dot.head<3>());
		const double xidot_eta = v().dot(lambda_dot.tail<3>());

		return ddt_dexp_inv(alpha, beta, theta_sqr, xi_eta, xi_xidot, xi_etadot, xidot_eta, lambda_dot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the time-derivative of the inverse of differential-of-exponential matrix on \f$ SO(3) \f$ of the twist \f$ \lambda \f$
	//! for the given time-derivative of the twist \f$ \dot{\lambda} \f$ in terms of the precomputed parameters. 
	//! That is, it returns \f$ \frac{d}{dt} {\rm dexp}_{\lambda}^{-1} (\dot{\lambda}) \f$.
	//!
	//! \details
	//! The parameters should computed by compute_terms(). 
	//! They can be reused to compute exponential map family.
	//!
	//! \param alpha alpha value
	//! \param beta beta value
	//! \param theta_sqr square of the theta
	//! \param xi_eta dot product of the linear and angular components, i.e. \f$ \xi^T \eta \f$
	//! \param xi_xidot dot product of the angular component and its derivative, i.e. \f$ \xi^T \dot{\xi} \f$
	//! \param xi_etadot dot product of the angular component and the derivative of the linear component, i.e. \f$ \xi^T \dot{\eta} \f$
	//! \param xidot_eta dot product of the derivative of angular component and the linear component, i.e. \f$ \dot{\xi}^T \eta \f$
	//! \param lambda_dot time-derivative of the twist or  \f$ \dot{\lambda} \f$
	//!
	//! \return The 6-by-6 time-derivative of the differential-of-exponential matrix 
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	template<typename OtherDerived>
	inline Matrix6d ddt_dexp_inv(double alpha, double beta, double theta_sqr, 
		double xi_eta, double xi_xidot, double xi_etadot, double xidot_eta,  
		Eigen::MatrixBase<OtherDerived> const & lambda_dot) const
	{
		Eigen::Matrix3d D = Vector3D(w()).ddt_dexp_inv(alpha, beta, theta_sqr, xi_xidot, lambda_dot.tail<3>());
		Eigen::Matrix3d Ddot;

		if (theta_sqr == 0)
		{
			Ddot = internal::_common_expression_4(-0.5, 1.0/12.0, w(), lambda_dot.tail<3>(), v(), lambda_dot.head<3>());
		}
		else
		{
			const double gamma = alpha/beta;
			const double zeta = xi_eta*xi_xidot/theta_sqr;
			const double theta_quad = theta_sqr*theta_sqr;
			const double common = (1.0/beta + gamma - 2.0)/theta_quad;

			const double e = (2.0/theta_quad)*(1.0 - gamma / beta)*zeta + common*(xidot_eta + xi_etadot - 3.0*zeta);
			const double h = (1.0 - gamma)/theta_sqr;
			Ddot = 	internal::_common_expression_3(0, 0, 0, -0.5, e, common*xi_eta, common*xi_xidot, h, h, 
				w(), lambda_dot.tail<3>(), v(), lambda_dot.head<3>());			
		}

		Matrix6d res; 
		res << D, Ddot, Eigen::Matrix3d::Zero(), D;

		return res;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! premultiplies the block-wise transpose of an upper block-triangular matrix and the twist
	//! 
	//! \details
	//! The upper block-triangular matrix is defined by \f$ \begin{bmatrix} A & B \\ 0 & C \end{bmatrix} \f$ where each block is three-dimensiaonl 
	//! square matrix. Then, the block-wise transposed multiplication of this upper block-triangular matrix with the twist is defined by 
	//! \f$ \begin{bmatrix} A^T v + B^T \omega \\ C^T \omega \end{bmatrix} \f$ for \f$ V = \begin{bmatrix} v \\ \omega \end{bmatrix} \f$.
	//! This function is necessary to compute differential-of-exponential families for \f$ - \xi \f$.
	//! 
	//! \param UBT The six-dimensional upper block-triangular matrix
	//  ----------------------------------------------------------
	inline LieGroup::Twist upperBlockTriTransposedMult(Matrix6d const & UBT) const
	{
		LieGroup::Twist res;

		res << UBT.topLeftCorner<3,3>().transpose()*v() + UBT.topRightCorner<3,3>().transpose()*w(), 
			   UBT.bottomRightCorner<3,3>().transpose()*w();

		return res;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! rotates the twist by the rotation matrix
	//! 
	//! \details
	//! The twist is updated by \f$ \begin{bmatrix} R v \\ R \omega \end{bmatrix} \f$ 
	//!	for \f$ V = \begin{bmatrix} v \\ \omega \end{bmatrix} \f$.
	//! The function is defined in Rotation.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Twist V; 
	//! LieGroup::Rotation R;
	//! ...
	//! V.rotate(R); // This sets V = [ R*v(); R*w() ]
	//!	\endcode
	//!
	//! \param R The three-dimensional rotation matrix
	//!
	//! \sa LieGroup::Rotation
	//  ----------------------------------------------------------
	inline void rotate(Rotation const & R);

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! rotates the twist by the inverse of the rotation matrix
	//! 
	//! \details
	//! The twist is updated by \f$ \begin{bmatrix} R^T v \\ R^T \omega \end{bmatrix} \f$ 
	//!	for \f$ V = \begin{bmatrix} v \\ \omega \end{bmatrix} \f$.
	//! The function is defined in Rotation.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Twist V; 
	//! LieGroup::Rotation R;
	//! ...
	//! V.irotate(R); // This sets V = [ R.transpose()*v(); R.transpose()*w() ]
	//!	\endcode
	//!
	//! \param R The three-dimensional rotation matrix
	//!
	//! \sa LieGroup::Rotation
	//  ----------------------------------------------------------
	inline void irotate(Rotation const & R);

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! transforms the twist by the adjoint transformation of the homogeneous transformation matrix.
	//! 
	//! \details
	//! The twist is updated by \f$ V = {\rm Adj}_T V \f$.
	//! The function is defined in HTransform.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Twist V; 
	//! LieGroup::HTransform T;
	//! ...
	//! V.transform(T); // This sets V = T.adj()*V
	//! \endcode
	//!
	//! \param T The homogeneous transformation matrix
	//!
	//! \sa LieGroup::HTransform
	//! \sa LieGroup::HTransform::transform(Twist const & v)
	//  ----------------------------------------------------------
	inline void transform(HTransform const & T);

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! transforms the twist by the inverse adjoint transformation of the homogeneous transformation matrix
	//! 
	//! \details
	//! The twist is updated by \f$ V = {\rm Adj}_T^{-1} V \f$.
	//! The function is defined in HTransform.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Twist V; 
	//! LieGroup::HTransform T;
	//! ...
	//! V.itransform(T); // This sets V = T.iadj()*V
	//! \endcode
	//!
	//! \param T The homogeneous transformation matrix
	//!
	//! \sa LieGroup::HTransform
	//! \sa LieGroup::HTransform::itransform(Twist const & v)
	//  ----------------------------------------------------------
	inline void itransform(HTransform const & T);
};

}
