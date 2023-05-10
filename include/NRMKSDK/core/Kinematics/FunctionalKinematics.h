//  ---------------------- Doxygen info ----------------------
//! \file FunctionalKinematics.h
//!
//! \brief
//! Header file for the class FunctionalKinematics (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a functional kinematics class
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
//! \date February 2016
//! 
//! \version 1.9.3
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013-2016 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

// This file is part of NRMKFoundation, a lightweight C++ template library
// for robot motion control.
//
// Copyright (C) 2013-2014 Neuromeka <crossover69@gmail.com>

#pragma once

#include "Kinematics.h"
#include "../Util/TrigonometricEquations.h"

namespace NRMKFoundation
{
namespace internal
{
	// from a vector v = [v(0), v(1), v(2), ..., v(n-1)]
	// generate the vector a = [v(0), v(0) + v(1), v(0) + v(1) + v(2), ..., v(0) + v(1) + ... + v(n-2) + v(n-1)]
	template<typename JointVec>
	inline void _GenAccumulatedSum(JointVec const & v, JointVec & a) 
	{
		a[0] = v[0];
		for (int k = 1; k < v.SizeAtCompileTime; k++)
			a[k] = a[k-1] + v[k];
	}

	// from a vector v = [v(0), v(1), v(2), ..., v(n-1)]
	// generate the vector a = [v(n-1), v(n-1) + v(n-2), v(n-1) + v(n-2) + v(n-3), ..., v(n-1) + v(n-2) + v(n-3) + ... + v(1) + v(0)]
	template<typename JointVec>
	inline void _GenReverseAccumulatedSum(JointVec const & v, JointVec & a)
	{
		a[0] = v[JointVec::SizeMinusOne];
		for (int k = 1, j = JointVec::SizeMinusOne - 1; k < v.SizeAtCompileTime; k++, j--)
			a[k] = a[k-1] + v[j];
	}

	inline double _clipAngle(double th, double lbound, double ubound)
	{
		double factor = ubound - lbound;
		while (th > ubound)
			th -= factor;

		while (th <= lbound)
			th += factor;

		return th;
	}
}

//  ---------------------- Doxygen info ----------------------
//! \class FunctionalKinematics
//!
//! \brief
//! This implements a functional task kinematics. 
//! 
//! \details 
//! Functional tasks are described in terms of task position represented in terms of a vector \f$ p \in \mathbb{R}^m \f$
//! and task velocity vector \f$ \dot{p} \in \mathbb{R}^m \f$. In addition, they are related by the joint position and velocity vector, 
//! \f$ q \in \mathbb{R}^n \f$ and \f$ \dot{q} \in \mathbb{R}^n \f$ by \f$ p = k(q) \f$ and \f$ \dot{p} = J(q) \dot{q} \f$, 
//! where \f$ J(q) \in \mathbb{R}^{m \times n} \f$ is the task Jacobian defined by \f$ \frac{\partial k}{\partial q^T} \f$.
//! User should provide the algorithm to compute \f$ p \f$, \f$ \dot{p} \f$, \f$ J \f$, and \f$ \dot{J} \f$ 
//! to implement a concrete functional task.
//!
//! \note
//! Users should inherit this class to define a customized functional task kinematics.
//! See Position2DKinematics as an example.
//! 
//! \tparam SubsysType Type of the subsys
//! \tparam DIM Dimension of the task variable
//! \tparam FunctionalKinematicsType Derived functional kinematics class 
//!
//! \sa Kinematics
//! \sa Position2DKinematics
//  ----------------------------------------------------------
template <typename SubsysType, int DIM, typename FunctionalKinematicsType>
class FunctionalKinematics : public Kinematics<SubsysType, DIM, FunctionalKinematics<SubsysType, DIM, FunctionalKinematicsType> >
{
public:
	typedef typename Kinematics<SubsysType, DIM, FunctionalKinematics<SubsysType, DIM, FunctionalKinematicsType> >::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename Kinematics<SubsysType, DIM, FunctionalKinematics<SubsysType, DIM, FunctionalKinematicsType> >::TaskJac TaskJac; //!< Typedef of task Jacobian

	//! \returns Reference to the derived object 
	inline FunctionalKinematicsType& derived() { return *static_cast<FunctionalKinematicsType*>(this); }
	//! \returns Constant reference to the derived object 
	inline const FunctionalKinematicsType& derived() const { return *static_cast<const FunctionalKinematicsType*>(this); }
	
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the functional task kinematics
	//!
	//! \details
	//! The task position \f$ p \f$ and velocity vector  \f$ \dot{p} \f$ should be computed using the system's state. 
	//! They should be reserved in the arguments p and pdot, respectively.
	//!
	//! \param subsys System object
	//! \param p Task position vector or \f$ p \f$
	//! \param pdot Task velocity vector or \f$ \dot{p} \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, TaskVec & p, TaskVec & pdot) const
	{
		derived().kinematics(subsys, p, pdot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the functional task kinematics
	//!
	//! \details
	//! The task position \f$ p \f$ should be computed using the system's state. 
	//! They should be reserved in the arguments p.
	//!
	//! \param subsys System object
	//! \param p Task position vector or \f$ p \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, TaskVec & p) const
	{
		derived().kinematics(subsys, p);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the functional task kinematics with the task Jacobian and its derivative
	//!
	//! \details
	//! The task position \f$ p \f$ and velocity vector  \f$ \dot{p} \f$ should be computed using the system's state. 
	//! They should be reserved in the arguments p and pdot, respectively.
	//! Also, the task Jacobian \f$ J \f$ and its derivative \f$ \dot{J} \f$ should be computed and passed to the arguments J and Jdot, respectively.
	//!
	//! \param subsys System object
	//! \param p Task position vector or \f$ p \f$
	//! \param pdot Task velocity vector or \f$ \dot{p} \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//! \param Jdot Task Jacobian derivative matrix or \f$ \dot{J} \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, TaskVec & p, TaskVec & pdot, TaskJac & J, TaskJac & Jdot) const
	{
		derived().jacobian(subsys, p, pdot, J, Jdot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the task position with the task Jacobian 
	//!
	//! \details
	//! The task position \f$ p \f$ and the task Jacobian \f$ J \f$ should be computed and passed to arguments p and J, respectively.
	//!
	//! \param subsys System object
	//! \param p Task position vector or \f$ p \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, TaskVec & p, TaskJac & J) const
	{
		derived().jacobian(subsys, p, J);
	}
};

//  ---------------------- Doxygen info ----------------------
//! \class Position2DKinematics
//!
//! \brief
//! This implements a planar 2D position kinematics, a concrete functional task class.
//! 
//! \details
//! The planar position kinematics is defined for the planar manipulators by the kinematics 
//! \f$ p = \begin{bmatrix} x \\ y \end{bmatrix} = \begin{bmatrix} l_1*cos(q_1) + l_2*cos(q_1+q_2) + \cdots \\ l_1*sin(q_1) + l_2*sin(q_1+q_2) + \cdots \end{bmatrix} \f$.
//! Then \f$ \dot{p} = \begin{bmatrix} \dot{x} \\ \dot{y} \end{bmatrix} = \begin{bmatrix} -l_1*sin(q_1) \dot{q}_1 - l_2*sin(q_1+q_2) (\dot{q}_1+\dot{q}_2) + \cdots \\ l_1*cos(q_1) \dot{q}_1 + l_2*cos(q_1+q_2) (\dot{q}_1+\dot{q}_2) + \cdots \end{bmatrix} \f$.
//! In order to compute the task Jacobian matrix, one has to compute \f$ \frac{\partial p}{\partial q^T} \f$ and it is given by
//! \f$ J = \begin{bmatrix} -l_1*sin(q_1) - l_2*sin(q_1+q_2) - \cdots & -l_2*sin(q_1+q_2) - \cdots & \cdots \\ l_1*cos(q_1) + l_2*cos(q_1+q_2) + \cdots & l_2*cos(q_1+q_2) + \cdots &  \cdots \end{bmatrix} \f$.
//! Its derivative can be computed in a straightforward manner.
//
//! \tparam SubsysType Type of the subsys
//!
//! \sa FunctionalKinematics
//  ----------------------------------------------------------
template <typename SubsysType>
class Position2DKinematics : public FunctionalKinematics<SubsysType, 2, Position2DKinematics<SubsysType> >
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \enum
	//! 
	//! \brief
	//! Provides compile-time constants
	//  ----------------------------------------------------------
	enum 
	{	
		DIM = 2,	//!< Task dimension
		NUM_JOINTS = SubsysType::NUM_JOINTS,  //!< The number of joints
	};

	typedef typename SubsysType::JointVec JointVec;

	typedef typename FunctionalKinematics<SubsysType, 2, Position2DKinematics<SubsysType> >::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename FunctionalKinematics<SubsysType, 2, Position2DKinematics<SubsysType> >::TaskJac TaskJac; //!< Typedef of task Jacobian

	typedef typename FunctionalKinematics<SubsysType, 2, Position2DKinematics<SubsysType> >::PosType PosType; //!< Typedef of task position
	typedef typename FunctionalKinematics<SubsysType, 2, Position2DKinematics<SubsysType> >::VelType VelType; //!< Typedef of task velocity
	typedef typename FunctionalKinematics<SubsysType, 2, Position2DKinematics<SubsysType> >::AccType AccType; //!< Typedef of task acceleration
	typedef typename FunctionalKinematics<SubsysType, 2, Position2DKinematics<SubsysType> >::ForceType ForceType; //!< Typedef of task force

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the kinematics
	//!
	//! \param subsys System object
	//! \param l Link length vector
	//  ----------------------------------------------------------
	inline Position2DKinematics(SubsysType const & subsys, JointVec const & l)
		: FunctionalKinematics<SubsysType, DIM, Position2DKinematics>()
		, _l(l)
	{
		_qoffset.setZero();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the kinematics
	//!
	//! \param subsys System object
	//  ----------------------------------------------------------
	inline Position2DKinematics(SubsysType const & subsys)
		: FunctionalKinematics<SubsysType, DIM, Position2DKinematics>()
	{
		_qoffset.setZero();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the link length vector
	//  ----------------------------------------------------------
	inline void set(JointVec const & l)
	{
		_l = l;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the zero configuration offset vector
	//  ----------------------------------------------------------
	inline void setZeroConfigurationOffset(JointVec const & qoffset)
	{
		_qoffset = qoffset;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the task kinematics for a planar 2D kinematics
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, TaskVec & p, TaskVec & pdot) const
	{
		JointVec q_sum;
		JointVec qdot_sum;

		// FIXME implement accumulated sum vector
		JointVec q = subsys.q() + _qoffset;
		internal::_GenAccumulatedSum(q, q_sum);
		internal::_GenAccumulatedSum(subsys.qdot(), qdot_sum);

		// 		q_sum[0] = q[0];
		// 		qdot_sum[0] = qdot[0];
		// 
		// 		for (int k = 1; k < NUM_BODIES; k++)
		// 		{
		// 			q_sum[k] = q_sum[k-1] + q[k];
		// 			qdot_sum[k] = qdot_sum[k-1] + qdot[k];
		// 		}

		JointVec c = _l.array()*q_sum.array().cos();
		JointVec s = _l.array()*q_sum.array().sin();

		JointVec cdot = -s.cwiseProduct(qdot_sum);
		JointVec sdot = c.cwiseProduct(qdot_sum);

		JointVec c_accum;
		JointVec s_accum;

		internal::_GenReverseAccumulatedSum(c, c_accum);
		internal::_GenReverseAccumulatedSum(s, s_accum);

		JointVec cdot_accum;
		JointVec sdot_accum;

		internal::_GenReverseAccumulatedSum(cdot, cdot_accum);
		internal::_GenReverseAccumulatedSum(sdot, sdot_accum);

		// 		p << c.sum(), s.sum(); 
		// 		pdot << cdot.sum(), sdot.sum();

		p << c_accum[JointVec::SizeMinusOne], s_accum[JointVec::SizeMinusOne]; 
		pdot << cdot_accum[JointVec::SizeMinusOne], sdot_accum[JointVec::SizeMinusOne]; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the task kinematics for a planar 2D kinematics
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, TaskVec & p) const
	{
		JointVec q_sum;

		// FIXME implement accumulated sum vector
		JointVec q = subsys.q() + _qoffset;
		internal::_GenAccumulatedSum(q, q_sum);

		// 		q_sum[0] = q[0];
		// 		qdot_sum[0] = qdot[0];
		// 
		// 		for (int k = 1; k < NUM_BODIES; k++)
		// 		{
		// 			q_sum[k] = q_sum[k-1] + q[k];
		// 			qdot_sum[k] = qdot_sum[k-1] + qdot[k];
		// 		}

		JointVec c = _l.array()*q_sum.array().cos();
		JointVec s = _l.array()*q_sum.array().sin();

		JointVec c_accum;
		JointVec s_accum;

		internal::_GenReverseAccumulatedSum(c, c_accum);
		internal::_GenReverseAccumulatedSum(s, s_accum);

		p << c_accum[JointVec::SizeMinusOne], s_accum[JointVec::SizeMinusOne]; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the task kinematics for a planar 2D kinematics 
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, TaskVec & p, TaskVec & pdot, TaskJac & J, TaskJac & Jdot) const
	{
		JointVec q_sum;
		JointVec qdot_sum;

		// FIXME implement accumulated sum vector
		JointVec q = subsys.q() + _qoffset;
		internal::_GenAccumulatedSum(q, q_sum);
		internal::_GenAccumulatedSum(subsys.qdot(), qdot_sum);

		// 		q_sum[0] = q[0];
		// 		qdot_sum[0] = qdot[0];
		// 
		// 		for (int k = 1; k < NUM_BODIES; k++)
		// 		{
		// 			q_sum[k] = q_sum[k-1] + q[k];
		// 			qdot_sum[k] = qdot_sum[k-1] + qdot[k];
		// 		}

		JointVec c = _l.array()*q_sum.array().cos();
		JointVec s = _l.array()*q_sum.array().sin();

		JointVec cdot = -s.cwiseProduct(qdot_sum);
		JointVec sdot = c.cwiseProduct(qdot_sum);

		JointVec c_accum;
		JointVec s_accum;

		internal::_GenReverseAccumulatedSum(c, c_accum);
		internal::_GenReverseAccumulatedSum(s, s_accum);

		JointVec cdot_accum;
		JointVec sdot_accum;

		internal::_GenReverseAccumulatedSum(cdot, cdot_accum);
		internal::_GenReverseAccumulatedSum(sdot, sdot_accum);

		// 		p << c.sum(), s.sum(); 
		// 		pdot << cdot.sum(), sdot.sum();

		p << c_accum[JointVec::SizeMinusOne], s_accum[JointVec::SizeMinusOne]; 
		pdot << cdot_accum[JointVec::SizeMinusOne], sdot_accum[JointVec::SizeMinusOne]; 

		for (int k = 0, j = NUM_JOINTS - 1; k < NUM_JOINTS; k++, j--)
		{
			J.col(k) << -s_accum[j], c_accum[j];
			Jdot.col(k) << -sdot_accum[j], cdot_accum[j];
		}
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the task kinematics for a planar 2D kinematics
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, TaskVec & p, TaskJac & J) const
	{
		JointVec q_sum;

		// FIXME implement accumulated sum vector
		JointVec q = subsys.q() + _qoffset;
		internal::_GenAccumulatedSum(q, q_sum);

		JointVec c = _l.array()*q_sum.array().cos();
		JointVec s = _l.array()*q_sum.array().sin();

		JointVec c_accum;
		JointVec s_accum;

		internal::_GenReverseAccumulatedSum(c, c_accum);
		internal::_GenReverseAccumulatedSum(s, s_accum);

		p << c_accum[JointVec::SizeMinusOne], s_accum[JointVec::SizeMinusOne]; 

		for (int k = 0, j = NUM_JOINTS - 1; k < NUM_JOINTS; k++, j--)
			J.col(k) << -s_accum[j], c_accum[j];
	}

	inline int ikin(double x, double y, double * q1, double * q2)
	{
		return TrigonometricEquations::Planar2DEquation(_l[0], _l[1], x, y, q1, q2);
	}

// private:
// 	// from a vector v = [v(0), v(1), v(2), ..., v(n-1)]
// 	// generate the vector a = [v(0), v(0) + v(1), v(0) + v(1) + v(2), ..., v(0) + v(1) + ... + v(n-2) + v(n-1)]
// 	inline void _genAccumulatedSum(JointVec const & v, JointVec & a) const
// 	{
// 		a[0] = v[0];
// 		for (int k = 1; k < v.SizeAtCompileTime; k++)
// 			a[k] = a[k-1] + v[k];
// 	}
// 
// 	// from a vector v = [v(0), v(1), v(2), ..., v(n-1)]
// 	// generate the vector a = [v(n-1), v(n-1) + v(n-2), v(n-1) + v(n-2) + v(n-3), ..., v(n-1) + v(n-2) + v(n-3) + ... + v(1) + v(0)]
// 	inline void _genReverseAccumulatedSum(JointVec const & v, JointVec & a) const
// 	{
// 		a[0] = v[JointVec::SizeMinusOne];
// 		for (int k = 1, j = JointVec::SizeMinusOne - 1; k < v.SizeAtCompileTime; k++, j--)
// 			a[k] = a[k-1] + v[j];
// 	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Link length vector of the system
	//  ----------------------------------------------------------
	JointVec _l;

	/// FIXME@20160224
	//  ---------------------- Doxygen info ----------------------
	//! \brief Zero configuration offset
	//  ----------------------------------------------------------
	JointVec _qoffset;
};

//  ---------------------- Doxygen info ----------------------
//! \class Planar3DKinematics
//!
//! \brief
//! This implements a planar 3D kinematics, a concrete functional task class.
//! 
//! \details
//! The planar 3D kinematics is defined for the planar manipulators by the kinematics 
//! \f$ p = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix} = \begin{bmatrix} l_1*cos(q_1) + l_2*cos(q_1+q_2) + \cdots \\ l_1*sin(q_1) + l_2*sin(q_1+q_2) + \cdots \\ q_1 + q_2 + \cdots \end{bmatrix} \f$.
//! Then \f$ \dot{p} = \begin{bmatrix} \dot{x} \\ \dot{y} \end{bmatrix} = \begin{bmatrix} -l_1*sin(q_1) \dot{q}_1 - l_2*sin(q_1+q_2) (\dot{q}_1+\dot{q}_2) + \cdots \\ l_1*cos(q_1) \dot{q}_1 + l_2*cos(q_1+q_2) (\dot{q}_1+\dot{q}_2) + \cdots \\ \dot{q}_1 + \dot{q}_2 + \cdots  \end{bmatrix} \f$.
//! In order to compute the task Jacobian matrix, one has to compute \f$ \frac{\partial p}{\partial q^T} \f$ and it is given by
//! \f$ J = \begin{bmatrix} -l_1*sin(q_1) - l_2*sin(q_1+q_2) - \cdots & -l_2*sin(q_1+q_2) - \cdots & \cdots \\ l_1*cos(q_1) + l_2*cos(q_1+q_2) + \cdots & l_2*cos(q_1+q_2) + \cdots &  \cdots \\ 1 & 1 & \cdots \end{bmatrix} \f$.
//! Its derivative can be computed in a straightforward manner.
//
//! \tparam SubsysType Type of the subsys
//!
//! \sa FunctionalKinematics
//  ----------------------------------------------------------
template <typename SubsysType>
class Planar3DKinematics : public FunctionalKinematics<SubsysType, 3, Planar3DKinematics<SubsysType> >
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \enum
	//! 
	//! \brief
	//! Provides compile-time constants
	//  ----------------------------------------------------------
	enum 
	{	
		DIM = 3,	//!< Task dimension
		NUM_JOINTS = SubsysType::NUM_JOINTS,  //!< The number of joints
	};

	typedef typename SubsysType::JointVec JointVec;

	typedef typename FunctionalKinematics<SubsysType, 3, Planar3DKinematics<SubsysType> >::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename FunctionalKinematics<SubsysType, 3, Planar3DKinematics<SubsysType> >::TaskJac TaskJac; //!< Typedef of task Jacobian

	typedef typename FunctionalKinematics<SubsysType, 3, Planar3DKinematics<SubsysType> >::PosType PosType; //!< Typedef of task position
	typedef typename FunctionalKinematics<SubsysType, 3, Planar3DKinematics<SubsysType> >::VelType VelType; //!< Typedef of task velocity
	typedef typename FunctionalKinematics<SubsysType, 3, Planar3DKinematics<SubsysType> >::AccType AccType; //!< Typedef of task acceleration
	typedef typename FunctionalKinematics<SubsysType, 3, Planar3DKinematics<SubsysType> >::ForceType ForceType; //!< Typedef of task force

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the kinematics
	//!
	//! \param subsys System object
	//! \param l Link length vector
	//  ----------------------------------------------------------
	inline Planar3DKinematics(SubsysType const & subsys, JointVec const & l)
		: FunctionalKinematics<SubsysType, DIM, Planar3DKinematics>()
		, _l(l)
	{
		_qoffset.setZero();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the kinematics
	//!
	//! \param subsys System object
	//  ----------------------------------------------------------
	inline Planar3DKinematics(SubsysType const & subsys)
		: FunctionalKinematics<SubsysType, DIM, Planar3DKinematics>()
	{
		_qoffset.setZero();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the link length vector
	//  ----------------------------------------------------------
	inline void set(JointVec const & l)
	{
		_l = l;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the zero configuration offset vector
	//  ----------------------------------------------------------
	inline void setZeroConfigurationOffset(JointVec const & qoffset)
	{
		_qoffset = qoffset;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the task kinematics for a planar 3D kinematics
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, TaskVec & p, TaskVec & pdot) const
	{
		JointVec q_sum;
		JointVec qdot_sum;

		// FIXME implement accumulated sum vector
		JointVec q = subsys.q() + _qoffset;
		internal::_GenAccumulatedSum(q, q_sum);
		internal::_GenAccumulatedSum(subsys.qdot(), qdot_sum);

		JointVec c = _l.array()*q_sum.array().cos();
		JointVec s = _l.array()*q_sum.array().sin();

		JointVec cdot = -s.cwiseProduct(qdot_sum);
		JointVec sdot = c.cwiseProduct(qdot_sum);

		JointVec c_accum;
		JointVec s_accum;

		internal::_GenReverseAccumulatedSum(c, c_accum);
		internal::_GenReverseAccumulatedSum(s, s_accum);

		JointVec cdot_accum;
		JointVec sdot_accum;

		internal::_GenReverseAccumulatedSum(cdot, cdot_accum);
		internal::_GenReverseAccumulatedSum(sdot, sdot_accum);

		p << c_accum[JointVec::SizeMinusOne], s_accum[JointVec::SizeMinusOne], q_sum[JointVec::SizeMinusOne]; 
		pdot << cdot_accum[JointVec::SizeMinusOne], sdot_accum[JointVec::SizeMinusOne], qdot_sum[JointVec::SizeMinusOne]; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the task kinematics for a planar 2D kinematics
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, TaskVec & p) const
	{
		JointVec q_sum;

		// FIXME implement accumulated sum vector
		JointVec q = subsys.q() + _qoffset;
		internal::_GenAccumulatedSum(q, q_sum);

		JointVec c = _l.array()*q_sum.array().cos();
		JointVec s = _l.array()*q_sum.array().sin();

		JointVec c_accum;
		JointVec s_accum;

		internal::_GenReverseAccumulatedSum(c, c_accum);
		internal::_GenReverseAccumulatedSum(s, s_accum);

		p << c_accum[JointVec::SizeMinusOne], s_accum[JointVec::SizeMinusOne], q_sum[JointVec::SizeMinusOne]; 

		/// FIXME @20160228
		p[2] = internal::_clipAngle(p[2], -M_PI, M_PI);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the task kinematics for a planar 2D kinematics 
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, TaskVec & p, TaskVec & pdot, TaskJac & J, TaskJac & Jdot) const
	{
		JointVec q_sum;
		JointVec qdot_sum;

		// FIXME implement accumulated sum vector
		JointVec q = subsys.q() + _qoffset;
		internal::_GenAccumulatedSum(q, q_sum);
		internal::_GenAccumulatedSum(subsys.qdot(), qdot_sum);

		JointVec c = _l.array()*q_sum.array().cos();
		JointVec s = _l.array()*q_sum.array().sin();

		JointVec cdot = -s.cwiseProduct(qdot_sum);
		JointVec sdot = c.cwiseProduct(qdot_sum);

		JointVec c_accum;
		JointVec s_accum;

		internal::_GenReverseAccumulatedSum(c, c_accum);
		internal::_GenReverseAccumulatedSum(s, s_accum);

		JointVec cdot_accum;
		JointVec sdot_accum;

		internal::_GenReverseAccumulatedSum(cdot, cdot_accum);
		internal::_GenReverseAccumulatedSum(sdot, sdot_accum);

		p << c_accum[JointVec::SizeMinusOne], s_accum[JointVec::SizeMinusOne], q_sum[JointVec::SizeMinusOne]; 
		pdot << cdot_accum[JointVec::SizeMinusOne], sdot_accum[JointVec::SizeMinusOne], qdot_sum[JointVec::SizeMinusOne]; 

		/// FIXME @20160228
		p[2] = internal::_clipAngle(p[2], -M_PI, M_PI);

		for (int k = 0, j = NUM_JOINTS - 1; k < NUM_JOINTS; k++, j--)
		{
			J.col(k) << -s_accum[j], c_accum[j], 1;
			Jdot.col(k) << -sdot_accum[j], cdot_accum[j], 0;
		}
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the task kinematics for a planar 2D kinematics
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, TaskVec & p, TaskJac & J) const
	{
		JointVec q_sum;

		// FIXME implement accumulated sum vector
		JointVec q = subsys.q() + _qoffset;
		internal::_GenAccumulatedSum(q, q_sum);

		JointVec c = _l.array()*q_sum.array().cos();
		JointVec s = _l.array()*q_sum.array().sin();

		JointVec c_accum;
		JointVec s_accum;

		internal::_GenReverseAccumulatedSum(c, c_accum);
		internal::_GenReverseAccumulatedSum(s, s_accum);

		p << c_accum[JointVec::SizeMinusOne], s_accum[JointVec::SizeMinusOne], q_sum[JointVec::SizeMinusOne]; 

		/// FIXME @20160228
		p[2] = internal::_clipAngle(p[2], -M_PI, M_PI);

		for (int k = 0, j = NUM_JOINTS - 1; k < NUM_JOINTS; k++, j--)
			J.col(k) << -s_accum[j], c_accum[j], 1;
	}
	
	inline int ikin(double x, double y, double theta, double * q1, double * q2, double * q3)
	{		
		x -= _l[2]*cos(theta);
		y -= _l[2]*sin(theta);

		int num_roots = TrigonometricEquations::Planar2DEquation(_l[0], _l[1], x, y, q1, q2);
		
		for (int i = 0; i < num_roots; i++)
			q3[i] = theta - q1[i] - q2[i];

		return num_roots;
	}
	
	inline JointVec const & qoffset() const
	{
		return _qoffset;
	}

public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Link length vector of the system
	//  ----------------------------------------------------------
	JointVec _l;

	/// FIXME@20160224
	//  ---------------------- Doxygen info ----------------------
	//! \brief Zero configuration offset
	//  ----------------------------------------------------------
	JointVec _qoffset;
};

} // namespace NRMKFoundation
