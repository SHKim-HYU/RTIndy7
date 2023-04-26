//! \file HTransformController.h
//! \brief Header file for the class HTransformController (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a homogeneous transform controller
//! to be used for the interface of NRMKFoundation library.
//! \n
//! \copydetails Neuromeka Foundation Library
//! \n
//! Neuromeka \n
//! South Korea\n
//! \n
//! http://www.neuromeka.com\n
//!
//! \date January 2017
//! 
//! \version 2.0.0
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//! \note v2.0.0 (20170117): initial implementation
//!
//! \note Copyright (C) 2013--2017 Neuromeka

#pragma once

#include "LieGroup/LieGroup.h"
#include "Controller.h"

namespace NRMKFoundation
{
namespace internal
{
}

//  ---------------------- Doxygen info ----------------------
//! \class HTransformController
//!
//! \brief
//! This implements a homogeneous transformation controller class.
//!
//! \details
//! Homogeneous transformation controllers is for three-dimensional rigid body motion task, which is described by 
//! the homogeneous transformation matrix as the task position, the body twist as the task velocity, and 
//! the body acceleration as the task acceleration. Hence, the task dimension is six. 
//! Homogeneous transformation matrices are represented by LieGroup::HTransform, and 
//! angular velocities and accelerations are represented by LieGroup::Twist. 
//! In particular, the current transformation is denoted by \f$ T \f$, and the current angular velocity and accelerations 
//! by \f$ V \f$ and \f$ \dot{V} \f$. 
//! 
//! Since homogeneous transformation matrices comprises the Lie group \f$ SE(3) \f$, 
//! the transformation error between a desired rotation matrix \f$ T_{des} \f$ and the current rotation \f$ T \f$
//! is defined in terms of the corresponding Lie algebra vector (of type LieGroup::Twist) \f$ \lambda = \begin{bmatrix} \eta \\ \xi \end{bmatrix} \f$
//! defined by \f$ \exp(\lambda) = T^{-1} T_d = \tilde{T} \f$. The velocity error \f$ \dot{\lambda} \f$ is then defined by
//! \f$ \dot{\lambda} = {\rm dexp}_{-\lambda}^{-1} \tilde{V} \f$ for the twist error
//! \f$ \tilde{V} = V_{des} - {\rm Adj}_{\tilde{T}}^{-1} V \f$.
//!
//! \sa Controller

//  ----------------------------------------------------------
class HTransformController : public Controller<6, HTransformController, LieGroup::HTransform, LieGroup::Twist, LieGroup::Twist, LieGroup::Wrench>
{	
public:
	typedef Controller<6, HTransformController, LieGroup::HTransform, LieGroup::Twist, LieGroup::Twist, LieGroup::Wrench>::TaskVec TaskVec; //!< Typedef of task vector
	typedef Controller<6, HTransformController, LieGroup::HTransform, LieGroup::Twist, LieGroup::Twist, LieGroup::Wrench>::TaskMat TaskMat; //!< Typedef of task matrix

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief 
	//! sets the gains of sub-task controllers (if any) 
	//! in terms of the specified gains, _kp, _kv, _ki, and _delT
	//! as well as _invL2sqr if needed.
	//! 
	//! \note
	//! It is really important to define this function 
	//! to have null statement explicitly because it has no subcontrollers.
	//! This prevents recursively defining this function. 
	//  ----------------------------------------------------------
	inline void setSubGains()
	{		
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the homogeneous transform reference velocity and acceleration
	//!
	//! \details
	//! This computes the homogeneous transformation reference velocity and acceleration by
	//!	 \f$ \dot{V}_{ref} = {\rm Adj}_{\tilde{T}} \left( \dot{V}_{des} - {\rm dexp}_{-\lambda} \ddot{\lambda}_{ref} - \frac{d}{dt} {\rm dexp}_{-\lambda} (-\dot{\lambda}) \dot{\lambda} + {\rm adj}_{\tilde{V}} V_{des} \right) \f$ 
	//! and \f$ V_{ref} = {\rm Adj}_{\tilde{T}} ( V_{des} - {\rm dexp}_{-\lambda} \dot{\lambda}_{ref} ) \f$.
	//! The Lie algebra reference velocity and acceleration are defined by
	//! \f$ \ddot{\lambda}_{ref} = - k_v \dot{\lambda} - k_p \lambda \f$ and 
	//! \f$ \dot{\lambda}_{ref} = - k_v \lambda - k_p \int \lambda dt \f$.
	//! 
	//! \param T Current homogeneous transformation matrix or \f$ T \f$
	//! \param V Current body twist or \f$ V \f$
	//! \param Td Desired homogeneous transformation matrix or \f$ T_{des} \f$
	//! \param Vd Desired body twist or \f$ V_{des} \f$
	//! \param Vdotd Desired body acceleration or \f$ \dot{V}_{des} \f$
	//! \param Vref Homogeneous transformation reference velocity or \f$ V_{ref} \f$
	//! \param Vdotref homogeneous transformation reference acceleration or \f$ \dot{V}_{ref} \f$
	//  ----------------------------------------------------------
	inline void refAcceleration(LieGroup::HTransform const & T, LieGroup::Twist const & V, 
						LieGroup::HTransform const & Td, LieGroup::Twist const & Vd, LieGroup::Twist const & Vdotd, 
						TaskVec & Vref, TaskVec & Vdotref)
	{
		TaskVec lambda;
		TaskVec lambdadot;

		refAcceleration(T, V, Td, Vd, Vdotd, Vref, Vdotref, lambda, lambdadot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the homogeneous transform reference velocity and acceleration as well as Lie algebra error and its derivative
	//!
	//! \details
	//! This computes the homogeneous transformation reference velocity and acceleration by
	//!	 \f$ \dot{V}_{ref} = {\rm Adj}_{\tilde{T}} \left( \dot{V}_{des} - {\rm dexp}_{-\lambda} \ddot{\lambda}_{ref} - \frac{d}{dt} {\rm dexp}_{-\lambda} (-\dot{\lambda}) \dot{\lambda} + {\rm adj}_{\tilde{V}} V_{des} \right) \f$ 
	//! and \f$ V_{ref} = {\rm Adj}_{\tilde{T}} ( V_{des} - {\rm dexp}_{-\lambda} \dot{\lambda}_{ref} ) \f$.
	//! The Lie algebra reference velocity and acceleration are defined by
	//! \f$ \ddot{\lambda}_{ref} = - k_v \dot{\lambda} - k_p \lambda \f$ and 
	//! \f$ \dot{\lambda}_{ref} = - k_v \lambda - k_p \int \lambda dt \f$.
	//! 
	//! \param T Current homogeneous transformation matrix or \f$ T \f$
	//! \param V Current body twist or \f$ V \f$
	//! \param Td Desired homogeneous transformation matrix or \f$ T_{des} \f$
	//! \param Vd Desired body twist or \f$ V_{des} \f$
	//! \param Vdotd Desired body acceleration or \f$ \dot{V}_{des} \f$
	//! \param Vref Homogeneous transformation reference velocity or \f$ V_{ref} \f$
	//! \param Vdotref homogeneous transformation reference acceleration or \f$ \dot{V}_{ref} \f$
	//! \param lambda Lie algebra position error or \f$ \lambda \f$
	//! \param lambdadot Lie algebra velocity error or \f$ \dot{\lambda} \f$
	//  ----------------------------------------------------------
	void refAcceleration(LieGroup::HTransform const & T, LieGroup::Twist const & V, 
		LieGroup::HTransform const & Td, LieGroup::Twist const & Vd, LieGroup::Twist const & Vdotd, 
		TaskVec & Vref, TaskVec & Vdotref, TaskVec & lambda, TaskVec & lambdadot);

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the homogeneous transform reference velocity and acceleration by impedance
	//!
	//! \details
	//! This computes the homogeneous transformation reference velocity and acceleration by
	//!	 \f$ \dot{V}_{ref} = {\rm Adj}_{\tilde{T}} \left( \dot{V}_{des} - {\rm dexp}_{-\lambda} \ddot{\lambda}_{ref} - \frac{d}{dt} {\rm dexp}_{-\lambda} (-\dot{\lambda}) \dot{\lambda} + {\rm adj}_{\tilde{V}} V_{des} \right) \f$ 
	//! and \f$ V_{ref} = {\rm Adj}_{\tilde{T}} ( V_{des} - {\rm dexp}_{-\lambda} \dot{\lambda}_{ref} ) \f$.
	//! The Lie algebra reference velocity and acceleration are defined by
	//! \f$ \ddot{\lambda}_{ref} = - K_v \dot{\lambda} - K_p \lambda \f + K_f \gamma $ and 
	//! \f$ \dot{\lambda}_{ref} = - K_v \lambda - K_p \int \lambda dt  + K_f \int \gamma dt \f$.
	//! where \f$ \lambda \f$ is the Lie algebra error and \f$ \gamma \f$ is the Lie algebra force error corresponding to \f$ \lambda \f$.
	//! They are computed by \f$ \exp(\gamma) = \tilde{T} \f$, \f$ \dot{\lambda} = {\rm dexp}_{-\lambda}^{-1} \tilde{V} \f$,
	//! and \f$ \gamma = {\rm dexp}_{-\lambda}^T \tilde{F} \f$ for the Lie group motion error \f$ \tilde{T} \f$, \f$ \tilde{V} \f$, and 
	//! Lie group force error \f$ \tilde{F} \f$, where
	//! \f$ \exp(\theta) = \tilde{T} = T^{-1} T_{des} \f$, \f$ \tilde{V} = V_{des} - {\rm Adj}_{\tilde{T}}^{-1} V \f$, 
	//! and \f$ \tilde{F} = F_{des} - {\rm Adj}_{\tilde{T}}^{T}  F \f$. Note that \f$ V \f$ and \f$ F \f$ are physically consistent pair defining the valid power.
	//!
	//! The impedance relationship is properly prescribed in terms of Lie group motion and force errors, that is 
	//! \f$ \tilde{F} = M_{V} \dot{\tilde{V}} + D_{V} \tilde{V} + K_{V} \lambda \f$.
	//! The impedance parameters are converted to the control gains \f$ K_v \f$, \f$ K_p \f$, and \f$ K_f \f$ used in generating the reference acceleration.
	//! 
	//! \param T Current homogeneous transformation matrix or \f$ T \f$
	//! \param V Current body twist or \f$ V \f$
	//! \param Td Desired homogeneous transformation matrix or \f$ T_{des} \f$
	//! \param Vd Desired body twist or \f$ V_{des} \f$
	//! \param Vdotd Desired body acceleration or \f$ \dot{V}_{des} \f$
	//! \param F Current body wrench or \f$ F \f$
	//! \param Fd Desired body wrench or \f$ F_{des} \f$
	//! \param Vref Homogeneous transformation reference velocity or \f$ V_{ref} \f$
	//! \param Vdotref homogeneous transformation reference acceleration or \f$ \dot{V}_{ref} \f$
	//  ----------------------------------------------------------
	inline void refImpedanceAcceleration(LieGroup::HTransform const & T, LieGroup::Twist const & V, 
		LieGroup::HTransform const & Td, LieGroup::Twist const & Vd, LieGroup::Twist const & Vdotd, 
		LieGroup::Wrench const & F, LieGroup::Wrench const & Fd, 
		TaskVec & Vref, TaskVec & Vdotref)
	{
		TaskVec lambda;
		TaskVec lambdadot;
		TaskVec gamma;

		refImpedanceAcceleration(T, V, Td, Vd, Vdotd, F, Fd, Vref, Vdotref, lambda, lambdadot, gamma);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the homogeneous transform reference velocity and acceleration as well as Lie algebra error and its derivative by impedance
	//!
	//! \details
	//! This computes the homogeneous transformation reference velocity and acceleration by
	//!	 \f$ \dot{V}_{ref} = {\rm Adj}_{\tilde{T}} \left( \dot{V}_{des} - {\rm dexp}_{-\lambda} \ddot{\lambda}_{ref} - \frac{d}{dt} {\rm dexp}_{-\lambda} (-\dot{\lambda}) \dot{\lambda} + {\rm adj}_{\tilde{V}} V_{des} \right) \f$ 
	//! and \f$ V_{ref} = {\rm Adj}_{\tilde{T}} ( V_{des} - {\rm dexp}_{-\lambda} \dot{\lambda}_{ref} ) \f$.
	//! The Lie algebra reference velocity and acceleration are defined by
	//! \f$ \ddot{\lambda}_{ref} = - K_v \dot{\lambda} - K_p \lambda \f + K_f \gamma $ and 
	//! \f$ \dot{\lambda}_{ref} = - K_v \lambda - K_p \int \lambda dt  + K_f \int \gamma dt \f$.
	//! where \f$ \lambda \f$ is the Lie algebra error and \f$ \gamma \f$ is the Lie algebra force error corresponding to \f$ \lambda \f$.
	//! They are computed by \f$ \exp(\gamma) = \tilde{T} \f$, \f$ \dot{\lambda} = {\rm dexp}_{-\lambda}^{-1} \tilde{V} \f$,
	//! and \f$ \gamma = {\rm dexp}_{-\lambda}^T \tilde{F} \f$ for the Lie group motion error \f$ \tilde{T} \f$, \f$ \tilde{V} \f$, and 
	//! Lie group force error \f$ \tilde{F} \f$, where
	//! \f$ \exp(\theta) = \tilde{T} = T^{-1} T_{des} \f$, \f$ \tilde{V} = V_{des} - {\rm Adj}_{\tilde{T}}^{-1} V \f$, 
	//! and \f$ \tilde{F} = F_{des} - {\rm Adj}_{\tilde{T}}^{T}  F \f$. Note that \f$ V \f$ and \f$ F \f$ are physically consistent pair defining the valid power.
	//!
	//! \param T Current homogeneous transformation matrix or \f$ T \f$
	//! \param V Current body twist or \f$ V \f$
	//! \param Td Desired homogeneous transformation matrix or \f$ T_{des} \f$
	//! \param Vd Desired body twist or \f$ V_{des} \f$
	//! \param Vdotd Desired body acceleration or \f$ \dot{V}_{des} \f$
	//! \param F Current body wrench or \f$ F \f$
	//! \param Fd Desired body wrench or \f$ F_{des} \f$
	//! \param Vref Homogeneous transformation reference velocity or \f$ V_{ref} \f$
	//! \param Vdotref homogeneous transformation reference acceleration or \f$ \dot{V}_{ref} \f$
	//! \param lambda Lie algebra position error or \f$ \lambda \f$
	//! \param lambdadot Lie algebra velocity error or \f$ \dot{\lambda} \f$
	//! \param gamma Lie algebra force error or \f$ \gamma \f$
	//  ----------------------------------------------------------
	void refImpedanceAcceleration(LieGroup::HTransform const & T, LieGroup::Twist const & V, 
		LieGroup::HTransform const & Td, LieGroup::Twist const & Vd, LieGroup::Twist const & Vdotd, 
		LieGroup::Wrench const & F, LieGroup::Wrench const & Fd, 
		TaskVec & Vref, TaskVec & Vdotref, 
		TaskVec & lambda, TaskVec & lambdadot,
		TaskVec & gamma);

	/// FIXME @20151015
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the homogeneous transform reference force as well as Lie algebra error and its derivative by compliance
	//!
	//! \details
	//! This computes the homogeneous transformation reference force by
	//!	 \f$ F_{ref} = {\rm Adj}_{\tilde{T}} \left( \dot{V}_{des} - {\rm dexp}_{-\lambda} \ddot{\lambda}_{ref} - \frac{d}{dt} {\rm dexp}_{-\lambda} (-\dot{\lambda}) \dot{\lambda} + {\rm adj}_{\tilde{V}} V_{des} \right) \f$.
	//! The Lie algebra reference velocity and acceleration are defined by
	//! \f$ \ddot{\lambda}_{ref} = - K_v \dot{\lambda} - K_p \lambda \f + K_f \gamma $ and 
	//! \f$ \dot{\lambda}_{ref} = - K_v \lambda - K_p \int \lambda dt  + K_f \int \gamma dt \f$.
	//! where \f$ \lambda \f$ is the Lie algebra error and \f$ \gamma \f$ is the Lie algebra force error corresponding to \f$ \lambda \f$.
	//! They are computed by \f$ \exp(\gamma) = \tilde{T} \f$, \f$ \dot{\lambda} = {\rm dexp}_{-\lambda}^{-1} \tilde{V} \f$,
	//! and \f$ \gamma = {\rm dexp}_{-\lambda}^T \tilde{F} \f$ for the Lie group motion error \f$ \tilde{T} \f$, \f$ \tilde{V} \f$, and 
	//! Lie group force error \f$ \tilde{F} \f$, where
	//! \f$ \exp(\theta) = \tilde{T} = T^{-1} T_{des} \f$, \f$ \tilde{V} = V_{des} - {\rm Adj}_{\tilde{T}}^{-1} V \f$, 
	//! and \f$ \tilde{F} = F_{des} - {\rm Adj}_{\tilde{T}}^{T}  F \f$. Note that \f$ V \f$ and \f$ F \f$ are physically consistent pair defining the valid power.
	//!
	//! \param T Current homogeneous transformation matrix or \f$ T \f$
	//! \param V Current body twist or \f$ V \f$
	//! \param Td Desired homogeneous transformation matrix or \f$ T_{des} \f$
	//! \param Vd Desired body twist or \f$ V_{des} \f$
	//! \param Vdotd Desired body acceleration or \f$ \dot{V}_{des} \f$
	//! \param F Current body wrench or \f$ F \f$
	//! \param Fd Desired body wrench or \f$ F_{des} \f$
	//! \param Fref homogeneous transformation reference force or \f$ F_{ref} \f$
	//  ----------------------------------------------------------
	void refComplianceForce(LieGroup::HTransform const & T, LieGroup::Twist const & V, 
		LieGroup::HTransform const & Td, LieGroup::Twist const & Vd, LieGroup::Twist const & Vdotd, 
		LieGroup::Wrench const & F, LieGroup::Wrench const & Fd, 
		TaskVec & Fref)
	{
		TaskVec lambda;
		TaskVec lambdadot;
		TaskVec gamma;

		refComplianceForce(T, V, Td, Vd, Vdotd, F, Fd, Fref, lambda, lambdadot, gamma);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the homogeneous transform reference force as well as Lie algebra error and its derivative by compliance
	//!
	//! \details
	//! This computes the homogeneous transformation reference force by
	//!	 \f$ F_{ref} = {\rm Adj}_{\tilde{T}} \left( \dot{V}_{des} - {\rm dexp}_{-\lambda} \ddot{\lambda}_{ref} - \frac{d}{dt} {\rm dexp}_{-\lambda} (-\dot{\lambda}) \dot{\lambda} + {\rm adj}_{\tilde{V}} V_{des} \right) \f$.
	//! The Lie algebra reference velocity and acceleration are defined by
	//! \f$ \ddot{\lambda}_{ref} = - K_v \dot{\lambda} - K_p \lambda \f + K_f \gamma $ and 
	//! \f$ \dot{\lambda}_{ref} = - K_v \lambda - K_p \int \lambda dt  + K_f \int \gamma dt \f$.
	//! where \f$ \lambda \f$ is the Lie algebra error and \f$ \gamma \f$ is the Lie algebra force error corresponding to \f$ \lambda \f$.
	//! They are computed by \f$ \exp(\gamma) = \tilde{T} \f$, \f$ \dot{\lambda} = {\rm dexp}_{-\lambda}^{-1} \tilde{V} \f$,
	//! and \f$ \gamma = {\rm dexp}_{-\lambda}^T \tilde{F} \f$ for the Lie group motion error \f$ \tilde{T} \f$, \f$ \tilde{V} \f$, and 
	//! Lie group force error \f$ \tilde{F} \f$, where
	//! \f$ \exp(\theta) = \tilde{T} = T^{-1} T_{des} \f$, \f$ \tilde{V} = V_{des} - {\rm Adj}_{\tilde{T}}^{-1} V \f$, 
	//! and \f$ \tilde{F} = F_{des} - {\rm Adj}_{\tilde{T}}^{T}  F \f$. Note that \f$ V \f$ and \f$ F \f$ are physically consistent pair defining the valid power.
	//!
	//! \param T Current homogeneous transformation matrix or \f$ T \f$
	//! \param V Current body twist or \f$ V \f$
	//! \param Td Desired homogeneous transformation matrix or \f$ T_{des} \f$
	//! \param Vd Desired body twist or \f$ V_{des} \f$
	//! \param Vdotd Desired body acceleration or \f$ \dot{V}_{des} \f$
	//! \param F Current body wrench or \f$ F \f$
	//! \param Fd Desired body wrench or \f$ F_{des} \f$
	//! \param Fref homogeneous transformation reference force or \f$ F_{ref} \f$
	//! \param lambda Lie algebra position error or \f$ \lambda \f$
	//! \param lambdadot Lie algebra velocity error or \f$ \dot{\lambda} \f$
	//! \param gamma Lie algebra force error or \f$ \gamma \f$
	//  ----------------------------------------------------------
	void refComplianceForce(LieGroup::HTransform const & T, LieGroup::Twist const & V, 
		LieGroup::HTransform const & Td, LieGroup::Twist const & Vd, LieGroup::Twist const & Vdotd, 
		LieGroup::Wrench const & F, LieGroup::Wrench const & Fd, 
		TaskVec & Fref, 
		TaskVec & lambda, TaskVec & lambdadot,
		TaskVec & gamma);

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the homogeneous transform reference velocity as well as Lie algebra error
	//!
	//! \details
	//! This computes the homogeneous transformation reference velocity by
	//!	 \f$ V_{ref} = {\rm Adj}_{\tilde{T}} ( V_{des} - {\rm dexp}_{-\lambda} \dot{\lambda}_{ref} ) \f$.
	//! The Lie algebra reference velocity is defined by
	//! \f$ \dot{\lambda}_{ref} = - k_v \lambda - k_p \int \lambda dt \f$.
	//! 
	//! \param T Current homogeneous transformation matrix or \f$ T \f$
	//! \param Td Desired homogeneous transformation matrix or \f$ T_{des} \f$
	//! \param Vd Desired body twist or \f$ V_{des} \f$
	//! \param Vref Homogeneous transformation reference velocity or \f$ V_{ref} \f$
	//! \param lambda Lie algebra position error or \f$ \lambda \f$
	//  ----------------------------------------------------------
	inline void refVelocity(LieGroup::HTransform const & T, 
		LieGroup::HTransform const & Td, LieGroup::Twist const & Vd,
		TaskVec & Vref)
	{
		TaskVec lambda;

		refVelocity(T, Td, Vd, Vref, lambda);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the homogeneous transform reference velocity as well as Lie algebra error
	//!
	//! \details
	//! This computes the homogeneous transformation reference velocity by
	//!	 \f$ V_{ref} = {\rm Adj}_{\tilde{T}} ( V_{des} - {\rm dexp}_{-\lambda} \dot{\lambda}_{ref} ) \f$.
	//! The Lie algebra reference velocity is defined by
	//! \f$ \dot{\lambda}_{ref} = - k_v \lambda - k_p \int \lambda dt \f$.
	//! 
	//! \param T Current homogeneous transformation matrix or \f$ T \f$
	//! \param Td Desired homogeneous transformation matrix or \f$ T_{des} \f$
	//! \param Vd Desired body twist or \f$ V_{des} \f$
	//! \param Vref Homogeneous transformation reference velocity or \f$ V_{ref} \f$
	//! \param lambda Lie algebra position error or \f$ \lambda \f$
	//  ----------------------------------------------------------
	void refVelocity(LieGroup::HTransform const & T, 
		LieGroup::HTransform const & Td, LieGroup::Twist const & Vd,
		TaskVec & Vref, TaskVec & lambda);
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the Lie algebra error and its derivative
	//!
	//! \param T Current homogeneous transformation matrix or \f$ T \f$
	//! \param V Current body twist or \f$ V \f$
	//! \param Td Desired homogeneous transformation matrix or \f$ T_{des} \f$
	//! \param Vd Desired body twist or \f$ V_{des} \f$
	//! \param lambda Lie algebra position error or \f$ \lambda \f$
	//! \param lambdadot Lie algebra velocity error or \f$ \dot{\lambda} \f$
	//  ----------------------------------------------------------
	void error(LieGroup::HTransform const & T, LieGroup::Twist const & V, 
		LieGroup::HTransform const & Td, LieGroup::Twist const & Vd, 
		TaskVec & lambda, TaskVec & lambdadot);

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the Lie algebra error 
	//!
	//! \param T Current homogeneous transformation matrix or \f$ T \f$
	//! \param Td Desired homogeneous transformation matrix or \f$ T_{des} \f$
	//! \param lambda Lie algebra position error or \f$ \lambda \f$
	//  ----------------------------------------------------------
	void error(LieGroup::HTransform const & T, LieGroup::HTransform const & Td, TaskVec & lambda);

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! reset all integral errors
	//  ----------------------------------------------------------
	/// ADDED @20160511
	inline void resetIntError()
	{
		_int_error.setZero();
		_int_ef.setZero();
	}
};	// class HTransformController

} // namespace NRMKFoundation
