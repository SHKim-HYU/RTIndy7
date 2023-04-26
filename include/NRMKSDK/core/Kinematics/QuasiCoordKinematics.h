//  ---------------------- Doxygen info ----------------------
//! \file QuasiCoordKinematics.h
//!
//! \brief
//! Header file for the class QuasiCoordKinematics (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a quasi-coordinate kinematics class
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
//! \date April 2016
//! 
//! \version 1.9.4
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note 
//!  - v1.9.4(20160406) : implemented the gradient projection method
//!
//! \note Copyright (C) 2013-2016 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

// This file is part of NRMKFoundation, a lightweight C++ template library
// for robot motion control.
//
// Copyright (C) 2013-2013 Neuromeka <coolcat@neuromeka.com>

#pragma once

#include <iostream>

#include "Kinematics.h"

namespace NRMKFoundation
{
namespace internal
{
}

//  ---------------------- Doxygen info ----------------------
//! \class QuasiCoordKinematics
//!
//! \brief
//! This implements a quasi-coordinate kinematics class.
//!
//! \details
//!	Quasi-coordinate tasks are such that the task velocity is described by vectors and the task position 
//! can not be defined. The task velocity is described by a vector \f$ v \in \mathbb{R}^m \f$. 
//! For quasi-coordinate tasks its integral does not have any concrete meaning. 
//! The task velocity is related by the joint velocity \f$ \dot{q} \in \mathbb{R}^n \f$ by \f$ \dot{p} = J(q) \dot{q} \f$, 
//! where \f$ J(q) \in \mathbb{R}^{m \times n} \f$ is the task Jacobian. 
//!
//! User should provide the algorithm to compute \f$ v \f$, \f$ J \f$, and \f$ \dot{J} \f$ 
//! to implement a concrete quasi-coordinate task.
//! 
//! \note
//! Users should inherit this class to define a customized quasi-coordinate task kinematics.
//! (See NullVelocityKinematics).
//! 
//! \tparam SubsysType Type of the subsys
//! \tparam DIM Dimension of the task variable
//! \tparam QuasiCoordKinematicsType Derived quasi-coordinate kinematics class 
//!
//! \sa Kinematics
//! \sa NullVelocityKinematics
//  ----------------------------------------------------------
template <typename SubsysType, int DIM, typename QuasiCoordKinematicsType>
class QuasiCoordKinematics : public Kinematics<SubsysType, DIM, QuasiCoordKinematics<SubsysType, DIM, QuasiCoordKinematicsType> >
{
public:
	typedef typename Kinematics<SubsysType, DIM, QuasiCoordKinematics<SubsysType, DIM, QuasiCoordKinematicsType> >::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename Kinematics<SubsysType, DIM, QuasiCoordKinematics<SubsysType, DIM, QuasiCoordKinematicsType> >::TaskJac TaskJac; //!< Typedef of task Jacobian

	//! \returns Reference to the derived object 
	inline QuasiCoordKinematicsType& derived() { return *static_cast<QuasiCoordKinematicsType*>(this); }
	//! \returns Constant reference to the derived object 
	inline const QuasiCoordKinematicsType& derived() const { return *static_cast<const QuasiCoordKinematicsType*>(this); }

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the quasi-coordinate task kinematics
	//!
	//! \details
	//! The task velocity vector \f$ v \f$ should be computed using the system's state. 
	//! It should be reserved in the arguments v.
	//!
	//! \note 
	//! The second argument for task position vector is not relevant within the context of quasi-coordinate task kinematics.
	//!
	//! \param subsys System object
	//! \param v Task velocity vector or \f$ v \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, TaskVec & x, TaskVec & v) const
	{
		derived().kinematics(subsys, x, v);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the quasi-coordinate task kinematics
	//!
	//! \details
	//! Do nothing..
	//!
	//! \note 
	//! The second argument for task position vector is not relevant within the context of quasi-coordinate task kinematics.
	//!
	//! \param subsys System object
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, TaskVec & x) const
	{
		derived().kinematics(subsys, x);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the quasi-coordinate task kinematics with the task Jacobian and its derivative
	//!
	//! \details
	//! The task velocity vector \f$ v \f$  should be computed using the system's state and passed to the arguments v.
	//! Also, the task Jacobian \f$ J \f$ and its derivative \f$ \dot{J} \f$ should be computed and passed to the arguments J and Jdot, respectively.
	//!
	//! \note 
	//! The second argument for task position vector is not relevant within the context of quasi-coordinate task kinematics.
	//! Hence it need not be updated.
	//!
	//! \param subsys System object
	//! \param v Task velocity vector or \f$ v \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//! \param Jdot Task Jacobian derivative matrix or \f$ \dot{J} \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, TaskVec & x, TaskVec & v, TaskJac & J, TaskJac & Jdot) const
	{
		derived().jacobian(subsys, x, v, J, Jdot);
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the quasi-coordinate task kinematics with the task Jacobian 
	//!
	//! \details
	//! The task Jacobian \f$ J \f$ should be computed and passed to the arguments J.
	//!
	//! \note 
	//! The second argument for task position vector is not relevant within the context of quasi-coordinate task kinematics.
	//! Hence it need not be updated.
	//!
	//! \param subsys System object
	//! \param v Task velocity vector or \f$ v \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, TaskVec & x, TaskJac & J) const
	{
		derived().jacobian(subsys, x, J);
	}
};

//  ---------------------- Doxygen info ----------------------
//! \class NullVelocityKinematics
//!
//! \brief
//! This implements a null velocity kinematics, a concrete quasi-coordinate task class.
//! 
//! \details 
//! Null velocity kinematics is defined by the null Jacobian of a task Jacobian. In particular, for a task kinematics 
//! having \f$ J \in \mathbb{R}^{m \times n} \f$, the null velocity kineatics is defined by 
//! \f$ \nu = Z(q) W(q) \dot{q} = Z_W(q) \f$, where \f$ Z(q) \in \mathbb{R}^{r \times n} \f$ for \f$ r = n - m \f$ 
//! is the null space bases matrix of \f$ J(q) \f$, such that \f$ I - J^+ J = Z^+ Z \f$ and \f$ W(q) \f$ is the 
//! positive-definite symmetric weight matrix. In the equation \f$ A^+ \f$ denotes the pseudoinverse of a matrix \f$ A \f$.
//! The number \f$ r \f$ is called the degrees-of-redundancy for the task kinematics. 
//!
//! Therefore, the task Jacobian for the null velocity kinematics is given by \f$ Z_W(q) = Z(q) W(q) \f$.
//! Its derivative is given by \f$ \dot{Z}_W(q) = \dot{Z}(q, \dot{q}) W(q) + Z(q) \dot{W}(q, \dot{q}) \f$.
//! In order to obtain the null space bases matrix \f$ Z(q) \f$ the column-pivoting Householder QR decomposition is used. 
//! Then, its derivative \f$ \dot{Z} \f$ is computed by \f$ \dot{Z}^T = - J^+ \dot{J} Z^T \f$.
//!
//! Since this is defined with respect to a certain task kinematics, the task kinematics type is passed as a template argument.
//!
//! \tparam SubsysType Type of the subsys
//! \tparam TaskKinematicsType the task kinematics type
//! 
//! \sa QuasiCoordKinematics
//! \sa Eigen::ColPivHouseholderQR
//  ----------------------------------------------------------
template <typename SubsysType, typename TaskKinematicsType>
class NullVelocityKinematics : public QuasiCoordKinematics<SubsysType, SubsysType::JOINT_DOF - TaskKinematicsType::DIM, NullVelocityKinematics<SubsysType, TaskKinematicsType> >
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
		TASK_DIM = TaskKinematicsType::DIM, //!< Task dimension		
		JOINT_DOF = SubsysType::JOINT_DOF, //!< Joint dof
		TASK_DOR = JOINT_DOF - TASK_DIM, //!< Degrees-of-redundancy to the task
			//QuasiCoordKinematics<SubsysType, SubsysType::JOINT_DOF - TaskKinematicsType::DIM, NullVelocityKinematics<SubsysType, TaskKinematicsType> >::DIM, 
		
	};
		
	typedef typename QuasiCoordKinematics<SubsysType, TASK_DOR, NullVelocityKinematics<SubsysType, TaskKinematicsType> >::TaskVec NullVec; //!< Typedef of null vector
	typedef typename QuasiCoordKinematics<SubsysType, TASK_DOR, NullVelocityKinematics<SubsysType, TaskKinematicsType> >::TaskJac NullJac; //!< Typedef of null Jacobian matrix
	//typedef typename TaskKinematicsType::TaskVec TaskVec;
	typedef typename TaskKinematicsType::TaskJac TaskJac; //!< Typedef of task Jacobian matrix
	typedef Eigen::Matrix<double, JOINT_DOF, TASK_DIM> TaskJacInv;  //!< Typedef of task Jacobian pseudoinverse matrix
	typedef Eigen::Matrix<double, JOINT_DOF, JOINT_DOF> JointMat; //!< Typedef of joint matrix
	typedef Eigen::Matrix<double, JOINT_DOF, 1> JointVec; //! Typedef of joint vector

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the default null velocity kinematics class
	//!
	//! \param subsys System object of type SubsysType
	//  ----------------------------------------------------------
	inline NullVelocityKinematics(SubsysType const & subsys)
		: QuasiCoordKinematics<SubsysType, TASK_DOR, NullVelocityKinematics>()
		, _kappa(1)
		, _isWeighted(false)
	{
		/// ADDED @20160406
		_Z.setZero();
		_Zdot.setZero();

		_W.setIdentity();
		_Wdot.setIdentity();
	}

#if 0
	/// FIXED @20160415 (Jinv is now member variable)
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! compute the null space bases matrix for the task Jacobian (by SVD)
	//!
	//! \param J Task Jacobian or \f$ J \f$ 
	//! \param Z Null space bases matrix  or \f$ Z \f$ 
	//  ----------------------------------------------------------
	inline void computeNullSpaceBases(TaskJac const & J, NullJac & Z) const
	{
		Eigen::JacobiSVD<Eigen::Matrix<double, TASK_DIM, JOINT_DOF> > svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
		
		JointMat V = svd.matrixV();
//#ifdef __GNUC__
		Z.transpose() = V.template rightCols<TASK_DOR>();
//#else
//		Z.transpose() = V.rightCols<TASK_DOR>();
//#endif

		// Jacobian pseudoinverse
		typename Eigen::JacobiSVD<Eigen::Matrix<double, TASK_DIM, JOINT_DOF> >::Index nonzeroSingVals = svd.nonzeroSingularValues();		
		_Jinv = svd.matrixV().leftCols(nonzeroSingVals); 
		_Jinv *= svd.singularValues().head(nonzeroSingVals).asDiagonal().inverse() * svd.matrixU().leftCols(nonzeroSingVals).transpose();
	}
#else
 	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! compute the null space bases matrix for the task Jacobian (by QR)
	//!
	//! \param J Task Jacobian or \f$ J \f$ 
	//! \param Z Null space bases matrix  or \f$ Z \f$ 
	//  ----------------------------------------------------------
	inline void computeNullSpaceBases(TaskJac const & J, NullJac & Z)  const
 	{
 		Eigen::ColPivHouseholderQR<TaskJacInv> qr(J.transpose());
		JointMat Q = qr.householderQ();
//#ifdef __GNUC__
 		Z.transpose() = Q.template rightCols<TASK_DOR>();
//#else
// 		Z.transpose() = Q.rightCols<TASK_DOR>();
// #endif
 
 		// Jacobian pseudoinverse
 		JointMat Id(JointMat::Identity());
 		_Jinv.transpose() = qr.solve(Id);
 	}
#endif

	/// FIXED @20160415 (Jinv is now member variable)
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! compute the null space bases matrix and its derivative for the task Jacobian and its derivative
	//!
	//! \param J Task Jacobian or \f$ J \f$ 
	//! \   Jdot Task Jacobian derivative or \f$ \dot{J} \f$ 
	//! \param Z Null space bases matrix  or \f$ Z \f$ 
	//! \param Zdot Derivative of Null space bases matrix or \f$ \dot{Z} \f$ 
	//  ----------------------------------------------------------
	inline void computeNullSpaceBases(TaskJac const & J, TaskJac const & Jdot, NullJac & Z, NullJac & Zdot) const
	{
		computeNullSpaceBases(J, Z);
		Zdot.transpose() = -_Jinv*Jdot*Z.transpose();

#if 0 // defined(__NRMK_DEBUG__)
		std::cout << "\nJ = " << J << std::endl;
 		std::cout << "\nZ = " << Z << std::endl;
 		std::cout << "\nJinv = " << _Jinv << std::endl;
 
 		std::cout << "\n J*Jinv = " << J*_Jinv << std::endl;
  		std::cout << "\n J*Z' = " << J*Z.transpose() << std::endl;
 
 		std::cout << "\n J*Zdot' + Jdot*Z'= " << J*Zdot.transpose() + Jdot*Z.transpose()<< std::endl;
#endif		
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the weight matrix
	//!
	//! \details
	//! This weight matrix corresponds to the weight matrix in computing weighted pseudo-inverse of the task Jacobian.
	//! Calling this function sets _isWeighted = true, which enables the weighted null velocity kinematics. 
	//! You can reset the flag by calling setWeighted(false). 
	//! 
	//! \param W Weight matrix or \f$ W \f$
	//  ----------------------------------------------------------
	inline void setWeightMatrix(JointMat const & W)
	{
		setWeighted(true);

		_W = W;
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the weight matrix and its derivative
	//!
	//! \details
	//! This weight matrix corresponds to the weight matrix in computing weighted pseudo-inverse of the task Jacobian.
	//! Calling this function sets _isWeighted = true, which enables the weighted null velocity kinematics. 
	//! You can reset the flag by calling setWeighted(false). 
	//! 
	//! \param W Weight matrix or \f$ W \f$
	//! \param Wdot Weight matrix derivative or \f$ \dot{W} \f$
	//  ----------------------------------------------------------
	inline void setWeightMatrix(JointMat const & W, JointMat const & Wdot)
	{
		setWeighted(true);

		_W = W;
		_Wdot = Wdot;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! enables the weighted pseudo-inverse
	//!
	//! \param weighted Flag to set weighted pseudo-inverse
	//  ----------------------------------------------------------
	inline void setWeighted(bool weighted = true)
	{
		_isWeighted = weighted;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief returns the weighted flag
	//  ----------------------------------------------------------
	inline bool weighted() const
	{
		return _isWeighted;
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the null velocity kinematics 
	//!
	//! \note
	//! Due to some implementation reason, it does nothing. So, call Jacobian() instead of this function
	//! if you need to update the null velocity vector.
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, NullVec &, NullVec & ) const
	{
// 		if (_isWeighted)
// 			Z = _Z*_W;	
// 		else
// 			Z = _Z;
// 		
// 		nu = Z*subsys.qdot();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the null velocity kinematics 
	//!
	//! \note
	//! Due to some implementation reason, it does nothing. So, call Jacobian() instead of this function
	//! if you need to update the null velocity vector.
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, NullVec &) const
	{
		// 		if (_isWeighted)
		// 			Z = _Z*_W;	
		// 		else
		// 			Z = _Z;
		// 		
		// 		nu = Z*subsys.qdot();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the null velocity kinematics with null kinematics Jacobian and its derivative
	//!
	//! \details
	//! This implements the null velocity kinematics for a general Jacobian and its derivative. 
	//! It will update the null velocity vector as well as the null velocity Jacobian and its derivative, 
	//! defined by the weight matrix and its derivative. 
	//!
	//! So, before calling this function, one should compute the null space bases matrix 
	//! by calling computeNullSpaceBases(). In presence of any weight matrix, 
	//! it should be assigned by setWeightMatrix(). 
	//! 
	//! \note 
	//! After calling computeNullSpaceBases(), the null space bases matrix and its derivative should be passed 
	//! as an input arguments. Then, they are updated on output.
	//!
	//! \param subsys Subsys object of SubsysType
	//! \param nu Null velocity vector or \f$ \nu \f$
	//! \param[in,out] Z Null velocity Jacobian
	//! \param[in,out] Zdot Null velocity Jacobian derivative
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, NullVec &, NullVec & nu, NullJac & Z, NullJac & Zdot) const
	{
		/// ADDED @20160406
		_Z = Z;
		_Zdot = Zdot;	

		if (_isWeighted)
		{
			//Zdot = _Zdot*_W + _Z*_Wdot;			
			Zdot *= _W;
			Zdot.noalias() += Z*_Wdot;

			// Z = Z*_W;
			Z *= _W;		
		}
	
		nu = Z*subsys.qdot();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the null velocity kinematics with null kinematics Jacobian 
	//!
	//! \details
	//! This implements the null velocity kinematics for a general Jacobian. 
	//! It will update the null velocity Jacobian defined by the weight matrix. 
	//!
	//! So, before calling this function, one should compute the null space bases matrix 
	//! by calling computeNullSpaceBases(). In presence of any weight matrix, 
	//! it should be assigned by setWeightMatrix(). 
	//! 
	//! \note 
	//! After calling computeNullSpaceBases(), the null space bases matrix should be passed 
	//! as an input arguments. Then, it is updated on output. 
	//! The second argument for task position vector is not relevant within the context of null velocity kinematics.
	//! Hence it is neglected.
	//!
	//! \param subsys Subsys object of SubsysType
	//! \param[in,out] Z Null velocity Jacobian
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, NullVec &, NullJac & Z) const
	{
		/// ADDED @20160406
		_Z = Z;	

		if (_isWeighted)
			Z *= _W;			
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the desired null velocity from the specified joint velocity 
	//!
	//! \details
	//! The desired null velocity is computed by projecting the specified joint velocity 
	//! by the null space bases matrix. In particular, 
	//! \f$ \nu_{des} = Z_W(q) \dot{q}_h \f$. 
	//! So, before calling this function, Jacobian() should be called properly. 
	//! 
	//! \param Z Null velocity Jacobian (i.e. \f$ Z_W \f$)
	//! \param Zdot Null velocity Jacobian derivative (i.e. \f$ \dot{Z}_W \f$)
	//! \param qdot_h Desired joint (self-motion) velocity or  \f$ \dot{q}_h \f$
	//! \param qddot_h Desired joint (self-motion) acceleration  or  \f$ \ddot{q}_h \f$
	//! \param nu_d Desired null velocity or  \f$ \nu_{des} \f$
	//! \param nudot_d Desired null acceleration or  \f$ \dot{\nu}_{des} \f$
	//  ----------------------------------------------------------
	inline void desMotion(NullJac const & Z, NullJac const & Zdot, JointVec const & qdot_h, JointVec const & qddot_h, NullVec & nu_d, NullVec & nudot_d) const
	{
		nu_d.noalias() = Z*qdot_h;
		nudot_d.noalias() = Z*qddot_h + Zdot*qdot_h;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the desired null velocity from the specified joint velocity 
	//!
	//! \details
	//! The desired null velocity is computed by projecting the specified joint velocity 
	//! by the null space bases matrix. In particular, 
	//! \f$ \nu_{des} = Z_W(q) \dot{q}_h \f$. 
	//! So, before calling this function, Jacobian() should be called properly. 
	//! 
	//! \param qdot_h Desired joint (self-motion) velocity or  \f$ \dot{q}_h \f$
	//! \param nu_d Desired null velocity or  \f$ \nu_{des} \f$
	//  ----------------------------------------------------------
	inline void desMotion(NullJac const & Z, JointVec const & qdot_h, NullVec & nu_d) const
	{
		nu_d.noalias() = Z*qdot_h;
	}

	/// ADDED @20160406
	// Implemented the Gradient Projection Method (GPM)

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the desired null velocity and acceleration from the performance measure by the gradient projection method
	//!
	//! \details
	//! The desired null velocity is computed by projecting the gradient
	//! by the null space bases matrix. In particular, 
	//! \f$ \nu_{des} = \kappa Z(q) \nable h(q) \f$. Notice that \f$ Z(q) \f$, not \f$ Z_W(q) \f$, is used.
	//! So, before calling this function, Jacobian() should be called properly. 
	//! The desired null acceleration is computed by 
	//! \f$ \dot{\nu}_{des} = \kappa Z(q) {\cal{H}} h(q) \dot{q} + \kappa \dot{Z} \nable h(q) \f$. 
	//! 
	//! \param subsys Subsys object of SubsysType
	//! \param measure Measure 
	//! \param nu_d Desired null velocity or  \f$ \nu_{des} \f$
	//! \param nudot_d Desired null acceleration or  \f$ \dot{\nu}_{des} \f$
	//  ----------------------------------------------------------
	template<typename PerformanceMeasureType>
	inline void desMotion(SubsysType const & subsys, PerformanceMeasureType const & measure, NullVec & nu_d, NullVec & nudot_d) const
	{
		JointVec grad;
		JointMat hess;
		measure.getGradient(subsys.q(), grad, hess);

		desMotion(_Z, _Zdot, _kappa*grad, _kappa*hess*subsys.qdot(), nu_d, nudot_d);		
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the desired null velocity and acceleration from the specified joint velocity and accelerations
	//!
	//! \details
	//! The desired null velocity is computed by projecting the specified joint velocity 
	//! by the null space bases matrix. In particular, 
	//! \f$ \nu_{des} = Z_W(q) \dot{q}_h \f$. 
	//! So, before calling this function, Jacobian() should be called properly. 
	//! 
	//! \param subsys Subsys object of SubsysType
	//! \param measure Measure 
	//! \param nu_d Desired null velocity or  \f$ \nu_{des} \f$
	//  ----------------------------------------------------------
	template<typename PerformanceMeasureType>
	inline void desMotion(SubsysType const & subsys, PerformanceMeasureType const & measure, NullVec & nu_d) const
	{
		JointVec grad;
		measure.getGradient(subsys.q(), grad);

		desMotion(_Z, _kappa*grad, nu_d);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the gradient gain
	//!
	//! \details
	//! The gradient gain \f$ \kappa \f$ should be positive (or negative) for maximizing (or minimizing) the measure. 
	//! 
	//! \param kappa Gradient gain
	//  ----------------------------------------------------------
	inline void setGradientGain(double kappa) 
	{
		_kappa = kappa;
	}

	/// ADDED @20160415
	//! \brief returns the pseudoinverse 
	inline JointMat const & Jinv() const 
	{
		return _Jinv;
	}

	//! \brief returns the null space bases matrix 
	inline NullJac const & Z() const 
	{
		return _Z;
	}

	//! \brief returns the inverse ny KDJSD
	inline void getKDJSDInverse(JointMat & Jinv) const 
	{
		Jinv.template leftCols<TASK_DIM>() = _Jinv;
		Jinv.template rightCols<TASK_DOR>() = _Z.transpose();
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	/// ADDED @20160406
	//! \brief Null space bases matrix
	mutable NullJac _Z;
	
	//! \brief Derivative of null space bases matrix
	mutable NullJac _Zdot; 	

	/// ADDED @20160415
	//! \brief pseudoinverse of the task Jacobian, i.e. \f$ J^{+} \f$ 
	mutable TaskJacInv _Jinv;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Weight matrix
	//  ----------------------------------------------------------
	JointMat _W;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Weight matrix derivative
	//  ----------------------------------------------------------
	JointMat _Wdot;

	double _kappa;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Flag whether the weight matrix is defined or not
	//  ----------------------------------------------------------
	bool _isWeighted;
};

} // namespace NRMKFoundation
