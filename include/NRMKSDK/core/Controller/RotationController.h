//  ---------------------- Doxygen info ----------------------
//! \file RotationController.h
//!
//! \brief
//! Header file for the class RotationController (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a rotation controller
//! to be used for the interface of NRMKFoundation library.
//!  
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
//! \note v1.9.4 (20160426): fixed refComplianceForce() for compliance control under Hinf optimal control
//! \note v1.9.4 (20160511): added resetIntError()
//!
//!
//! \note Copyright (C) 2013--2016 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#include "LieGroup/LieGroup.h"
#include "Controller.h"

namespace NRMKFoundation
{
namespace internal
{
}

//  ---------------------- Doxygen info ----------------------
//! \class RotationController
//!
//! \brief
//! This implements a rotation controller class.
//!
//! \details
//! Rotational controllers is for three-dimensional orientation task, which is described by 
//! the rotation matrix as the task position, the body angular velocity as the task velocity, and 
//! the body angular acceleration as the task acceleration. Rotation matrices are represented by LieGroup::Rotation, and 
//! angular velocities and accelerations are represented by LieGroup::Vector3D. 
//! In particular, the current rotation is denoted by \f$ R \f$, and the current angular velocity and accelerations 
//! by \f$ \omega \f$ and \f$ \dot{\omega} \f$. 
//! 
//! Since rotation matrices comprises the Lie group \f$ SO(3) \f$, 
//! the rotational error between a desired rotation matrix \f$ R_{des} \f$ and the current rotation \f$ R \f$
//! is defined in terms of the corresponding Lie algebra vector (of type LieGroup::Vector3D) \f$ \xi \f$
//! defined by \f$ \exp(\xi) = R^T R_d = \tilde{R} \f$. The velocity error \f$ \dot{\xi} \f$ is then defined by
//! \f$ \dot{\xi} = {\rm dexp}_{-\xi}^{-1} \tilde{\omega} \f$ for the angular velocity error
//! \f$ \tilde{\omega} = \omega_{des} - \tilde{R}^T \omega \f$.
//!
//! \sa Controller
//  ----------------------------------------------------------
class RotationController : public Controller<3, RotationController, LieGroup::Rotation, LieGroup::Vector3D>
{	
public:
	typedef Controller<3, RotationController, LieGroup::Rotation, LieGroup::Vector3D>::TaskVec TaskVec; //!< Typedef of task vector
	typedef Controller<3, RotationController, LieGroup::Rotation, LieGroup::Vector3D>::TaskMat TaskMat; //!< Typedef of task matrix

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
	//! computes the rotation reference velocity and acceleration
	//!
	//! \details
	//! This computes the rotation reference velocity and acceleration by
	//!	 \f$ \dot{\omega}_{ref} = \tilde{R} \left( \dot{\omega}_{des} - {\rm dexp}_{-\xi} \ddot{\xi}_{ref} - \frac{d}{dt} {\rm dexp}_{-\xi} (-\dot{\xi}) \dot{\xi} + \tilde{\omega} \times \omega_{des} \right) \f$ 
	//! and \f$ \omega_{ref} = \tilde{R} ( \omega_{des} - {\rm dexp}_{-\xi} \dot{\xi}_{ref} ) \f$.
	//! The Lie algebra reference velocity and acceleration are defined by
	//! \f$ \ddot{\xi}_{ref} = - k_v \dot{\xi} - k_p \xi \f$ and 
	//! \f$ \dot{\xi}_{ref} = - k_v \xi - k_p \int \xi dt \f$.
	//! 
	//! \param R Current rotation matrix or \f$ R \f$
	//! \param w Current body angular velocity vector or \f$ \omega \f$
	//! \param Rd Desired rotation matrix or \f$ R_{des} \f$
	//! \param wd Desired body angular velocity vector or \f$ \omega_{des} \f$
	//! \param wdotd Desired body angular acceleration vector or \f$ \dot{\omega}_{des} \f$
	//! \param wref Rotational reference velocity vector or \f$ \omega_{ref} \f$
	//! \param wdotref Rotational reference acceleration vector or \f$ \dot{\omega}_{ref} \f$
	//  ----------------------------------------------------------
	inline void refAcceleration(LieGroup::Rotation const & R, LieGroup::Vector3D const & w, 
						LieGroup::Rotation const & Rd, LieGroup::Vector3D const & wd, LieGroup::Vector3D const & wdotd, 
						TaskVec & wref, TaskVec & wdotref)
	{
		TaskVec xi;
		TaskVec xidot;
		
		refAcceleration(R, w, Rd, wd, wdotd, wref, wdotref, xi, xidot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the rotation reference velocity and acceleration as well as Lie algebra error and its derivative
	//!
	//! \details
	//! This computes the rotation reference velocity and acceleration by
	//!	 \f$ \dot{\omega}_{ref} = \tilde{R} \left( \dot{\omega}_{des} - {\rm dexp}_{-\xi} \ddot{\xi}_{ref} - \frac{d}{dt} {\rm dexp}_{-\xi} (-\dot{\xi}) \dot{\xi} + \tilde{\omega} \times \omega_{des} \right) \f$ 
	//! and \f$ \omega_{ref} = \tilde{R} ( \omega_{des} - {\rm dexp}_{-\xi} \dot{\xi}_{ref} ) \f$.
	//! The Lie algebra reference velocity and acceleration are defined by
	//! \f$ \ddot{\xi}_{ref} = - k_v \dot{\xi} - k_p \xi \f$ and 
	//! \f$ \dot{\xi}_{ref} = - k_v \xi - k_p \int \xi dt \f$.
	//! 
	//! \param R Current rotation matrix or \f$ R \f$
	//! \param w Current body angular velocity vector or \f$ \omega \f$
	//! \param Rd Desired rotation matrix or \f$ R_{des} \f$
	//! \param wd Desired body angular velocity vector or \f$ \omega_{des} \f$
	//! \param wdotd Desired body angular acceleration vector or \f$ \dot{\omega}_{des} \f$
	//! \param wref Rotational reference velocity vector or \f$ \omega_{ref} \f$
	//! \param wdotref Rotational reference acceleration vector or \f$ \dot{\omega}_{ref} \f$
	//! \param xi_ Lie algebra position error or \f$ \xi \f$
	//! \param xidot Lie algebra velocity error or \f$ \dot{\xi} \f$
	//  ----------------------------------------------------------
	void refAcceleration(LieGroup::Rotation const & R, LieGroup::Vector3D const & w, 
		LieGroup::Rotation const & Rd, LieGroup::Vector3D const & wd, LieGroup::Vector3D const & wdotd, 
		TaskVec & wref, TaskVec & wdotref, 
		TaskVec & xi, TaskVec & xidot); 

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the rotation reference velocity and acceleration by impedance
	//!
	//! \details
	//! This computes the rotation reference velocity and acceleration by
	//!	 \f$ \dot{\omega}_{ref} = \tilde{R} \left( \dot{\omega}_{des} - {\rm dexp}_{-\xi} \ddot{\xi}_{ref} - \frac{d}{dt} {\rm dexp}_{-\xi} (-\dot{\xi}) \dot{\xi} + \tilde{\omega} \times \omega_{des} \right) \f$ 
	//! and \f$ \omega_{ref} = \tilde{R} ( \omega_{des} - {\rm dexp}_{-\xi} \dot{\xi}_{ref} ) \f$.
	//! The Lie algebra reference velocity and acceleration are defined by
	//! \f$ \ddot{\xi}_{ref} = - K_v \dot{\xi} - K_p \xi + K_F \eta \f$ and 
	//! \f$ \dot{\xi}_{ref} = - K_v \xi - K_p \int \xi dt + K_F \int \eta dt\f$,
	//! where \f$ \xi \f$ is the Lie algebra error and \f$ \eta \f$ is the Lie algebra force error corresponding to \f$ \xi \f$.
	//! They are computed by \f$ \exp(\xi) = \tilde{R} \f$, \f$ \dot{xi} = {\rm dexp}_{-\xi}^{-1} \tilde{\omega} \f$,
	//! and \f$ \eta = {\rm dexp}_{-\xi}^T \tilde{n} \f$ for the Lie group motion error \f$ \tilde{R} \f$, \f$ \tilde{\omega} \f$, and 
	//! Lie group force error \f$ \tilde{n} \f$, where
	//! \f$ \exp(\theta) = \tilde{R} = R^T R_{des} \f$, \f$ \tilde{\omega} = \omega_{des} - \tilde{R}^T \omega \f$, 
	//! and \f$ \tilde{n} = n_{des} - \tilde{R}^T n \f$. Note that \f$ \omega \f$ and \f$ n \f$ are physically consistent pair defining the valid power.
	//!
	//! The impedance relationship is properly prescribed in terms of Lie group motion and force errors, that is 
	//! \f$ \tilde{n} = M_{\omega} \dot{\tilde{\omega}} + D_{\omega} \tilde{\omega} + K_{\omega} \theta \f$.
	//! The impedance parameters are converted to the control gains \f$ K_v \f$, \f$ K_p \f$, and \f$ K_f \f$ used in generating the reference acceleration.
	//! 
	//! \param R Current rotation matrix or \f$ R \f$
	//! \param w Current body angular velocity vector or \f$ \omega \f$
	//! \param Rd Desired rotation matrix or \f$ R_{des} \f$
	//! \param wd Desired body angular velocity vector or \f$ \omega_{des} \f$
	//! \param wdotd Desired body angular acceleration vector or \f$ \dot{\omega}_{des} \f$
	//! \param n Current body moment vector or \f$ n \f$
	//! \param nd Desired body moment vector or \f$ n_{des} \f$
	//! \param wref Rotational reference velocity vector or \f$ \omega_{ref} \f$
	//! \param wdotref Rotational reference acceleration vector or \f$ \dot{\omega}_{ref} \f$
	//  ----------------------------------------------------------
	inline void refImpedanceAcceleration(LieGroup::Rotation const & R, LieGroup::Vector3D const & w, 
		LieGroup::Rotation const & Rd, LieGroup::Vector3D const & wd, LieGroup::Vector3D const & wdotd, 
		LieGroup::Vector3D const & n, LieGroup::Vector3D const & nd, 
		TaskVec & wref, TaskVec & wdotref)
	{
		TaskVec xi;
		TaskVec xidot;
		TaskVec eta;

		refImpedanceAcceleration(R, w, Rd, wd, wdotd, n, nd, wref, wdotref, xi, xidot, eta);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the rotation reference velocity and acceleration as well as Lie algebra error and its derivative by impedance
	//!
	//! \details
	//! This computes the rotation reference velocity and acceleration by
	//!	 \f$ \dot{\omega}_{ref} = \tilde{R} \left( \dot{\omega}_{des} - {\rm dexp}_{-\xi} \ddot{\xi}_{ref} - \frac{d}{dt} {\rm dexp}_{-\xi} (-\dot{\xi}) \dot{\xi} + \tilde{\omega} \times \omega_{des} \right) \f$ 
	//! and \f$ \omega_{ref} = \tilde{R} ( \omega_{des} - {\rm dexp}_{-\xi} \dot{\xi}_{ref} ) \f$.
	//! The Lie algebra reference velocity and acceleration are defined by
	//! \f$ \ddot{\xi}_{ref} = - K_v \dot{\xi} - K_p \xi + K_F \eta \f$ and 
	//! \f$ \dot{\xi}_{ref} = - K_v \xi - K_p \int \xi dt + K_F \int \eta dt\f$,
	//! where \f$ \xi \f$ is the Lie algebra error and \f$ \eta \f$ is the Lie algebra force error corresponding to \f$ \xi \f$.
	//! They are computed by \f$ \exp(\xi) = \tilde{R} \f$, \f$ \dot{xi} = {\rm dexp}_{-\xi}^{-1} \tilde{\omega} \f$,
	//! and \f$ \eta = {\rm dexp}_{-\xi}^T \tilde{n} \f$ for the Lie group motion error \f$ \tilde{R} \f$, \f$ \tilde{\omega} \f$, and 
	//! Lie group force error \f$ \tilde{n} \f$, where
	//! \f$ \exp(\theta) = \tilde{R} = R^T R_{des} \f$, \f$ \tilde{\omega} = \omega_{des} - \tilde{R}^T \omega \f$, 
	//! and \f$ \tilde{n} = n_{des} - \tilde{R}^T n \f$. Note that \f$ \omega \f$ and \f$ n \f$ are physically consistent pair defining the valid power.
	//!
	//! The impedance relationship is properly prescribed in terms of Lie group motion and force errors, that is 
	//! \f$ \tilde{n} = M_{\omega} \dot{\tilde{\omega}} + D_{\omega} \tilde{\omega} + K_{\omega} \theta \f$.
	//! The impedance parameters are converted to the control gains \f$ K_v \f$, \f$ K_p \f$, and \f$ K_f \f$ used in generating the reference acceleration.
	//! 
	//! \param R Current rotation matrix or \f$ R \f$
	//! \param w Current body angular velocity vector or \f$ \omega \f$
	//! \param Rd Desired rotation matrix or \f$ R_{des} \f$
	//! \param wd Desired body angular velocity vector or \f$ \omega_{des} \f$
	//! \param wdotd Desired body angular acceleration vector or \f$ \dot{\omega}_{des} \f$
	//! \param n Current body moment vector or \f$ n \f$
	//! \param nd Desired body moment vector or \f$ n_{des} \f$
	//! \param wref Rotational reference velocity vector or \f$ \omega_{ref} \f$
	//! \param wdotref Rotational reference acceleration vector or \f$ \dot{\omega}_{ref} \f$
	//! \param xi Lie algebra position error or \f$ \xi \f$
	//! \param xidot Lie algebra velocity error or \f$ \dot{\xi} \f$
	//! \param eta Lie algebra force error or \f$ \eta \f$
	//  ----------------------------------------------------------
	void refImpedanceAcceleration(LieGroup::Rotation const & R, LieGroup::Vector3D const & w, 
		LieGroup::Rotation const & Rd, LieGroup::Vector3D const & wd, LieGroup::Vector3D const & wdotd, 
		LieGroup::Vector3D const & n, LieGroup::Vector3D const & nd, 
		TaskVec & wref, TaskVec & wdotref, 
		TaskVec & xi, TaskVec & xidot,
		TaskVec & eta);

	// FIXME @20151015
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the rotation reference force as well as Lie algebra error and its derivative by compliance
	//!
	//! \details
	//! This computes the rotation reference force by
	//!	 \f$ f_{ref} = \tilde{R} \left( \dot{\omega}_{des} - {\rm dexp}_{-\xi} \ddot{\xi}_{ref} - \frac{d}{dt} {\rm dexp}_{-\xi} (-\dot{\xi}) \dot{\xi} + \tilde{\omega} \times \omega_{des} \right) \f$.
	//! The Lie algebra reference velocity and acceleration are defined by
	//! \f$ \ddot{\xi}_{ref} = - K_v \dot{\xi} - K_p \xi + K_F \eta \f$ and 
	//! \f$ \dot{\xi}_{ref} = - K_v \xi - K_p \int \xi dt + K_F \int \eta dt\f$,
	//! where \f$ \xi \f$ is the Lie algebra error and \f$ \eta \f$ is the Lie algebra force error corresponding to \f$ \xi \f$.
	//! They are computed by \f$ \exp(\xi) = \tilde{R} \f$, \f$ \dot{xi} = {\rm dexp}_{-\xi}^{-1} \tilde{\omega} \f$,
	//! and \f$ \eta = {\rm dexp}_{-\xi}^T \tilde{n} \f$ for the Lie group motion error \f$ \tilde{R} \f$, \f$ \tilde{\omega} \f$, and 
	//! Lie group force error \f$ \tilde{n} \f$, where
	//! \f$ \exp(\theta) = \tilde{R} = R^T R_{des} \f$, \f$ \tilde{\omega} = \omega_{des} - \tilde{R}^T \omega \f$, 
	//! and \f$ \tilde{n} = n_{des} - \tilde{R}^T n \f$. Note that \f$ \omega \f$ and \f$ n \f$ are physically consistent pair defining the valid power.
	//!
	//! The impedance relationship is properly prescribed in terms of Lie group motion and force errors, that is 
	//! \f$ \tilde{n} = M_{\omega} \dot{\tilde{\omega}} + D_{\omega} \tilde{\omega} + K_{\omega} \theta \f$.
	//! The impedance parameters are converted to the control gains \f$ K_v \f$, \f$ K_p \f$, and \f$ K_f \f$ used in generating the reference acceleration.
	//! 
	//! \param R Current rotation matrix or \f$ R \f$
	//! \param w Current body angular velocity vector or \f$ \omega \f$
	//! \param Rd Desired rotation matrix or \f$ R_{des} \f$
	//! \param wd Desired body angular velocity vector or \f$ \omega_{des} \f$
	//! \param wdotd Desired body angular acceleration vector or \f$ \dot{\omega}_{des} \f$
	//! \param n Current body moment vector or \f$ n \f$
	//! \param nd Desired body moment vector or \f$ n_{des} \f$
	//! \param nref Rotational reference force vector or \f$ n_{ref} \f$
	//  ----------------------------------------------------------
	inline void refComplianceForce(LieGroup::Rotation const & R, LieGroup::Vector3D const & w, 
		LieGroup::Rotation const & Rd, LieGroup::Vector3D const & wd, LieGroup::Vector3D const & wdotd, 
		LieGroup::Vector3D const & n, LieGroup::Vector3D const & nd, 
		TaskVec & nref)
	{
		TaskVec xi;
		TaskVec xidot;
		TaskVec eta;

		refComplianceForce(R, w, Rd, wd, wdotd, n, nd, nref, xi, xidot, eta);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the rotation reference force as well as Lie algebra error and its derivative by compliance
	//!
	//! \details
	//! This computes the rotation reference force by
	//!	 \f$ f_{ref} = \tilde{R} \left( \dot{\omega}_{des} - {\rm dexp}_{-\xi} \ddot{\xi}_{ref} - \frac{d}{dt} {\rm dexp}_{-\xi} (-\dot{\xi}) \dot{\xi} + \tilde{\omega} \times \omega_{des} \right) \f$.
	//! The Lie algebra reference velocity and acceleration are defined by
	//! \f$ \ddot{\xi}_{ref} = - K_v \dot{\xi} - K_p \xi + K_F \eta \f$ and 
	//! \f$ \dot{\xi}_{ref} = - K_v \xi - K_p \int \xi dt + K_F \int \eta dt\f$,
	//! where \f$ \xi \f$ is the Lie algebra error and \f$ \eta \f$ is the Lie algebra force error corresponding to \f$ \xi \f$.
	//! They are computed by \f$ \exp(\xi) = \tilde{R} \f$, \f$ \dot{xi} = {\rm dexp}_{-\xi}^{-1} \tilde{\omega} \f$,
	//! and \f$ \eta = {\rm dexp}_{-\xi}^T \tilde{n} \f$ for the Lie group motion error \f$ \tilde{R} \f$, \f$ \tilde{\omega} \f$, and 
	//! Lie group force error \f$ \tilde{n} \f$, where
	//! \f$ \exp(\theta) = \tilde{R} = R^T R_{des} \f$, \f$ \tilde{\omega} = \omega_{des} - \tilde{R}^T \omega \f$, 
	//! and \f$ \tilde{n} = n_{des} - \tilde{R}^T n \f$. Note that \f$ \omega \f$ and \f$ n \f$ are physically consistent pair defining the valid power.
	//!
	//! The impedance relationship is properly prescribed in terms of Lie group motion and force errors, that is 
	//! \f$ \tilde{n} = M_{\omega} \dot{\tilde{\omega}} + D_{\omega} \tilde{\omega} + K_{\omega} \theta \f$.
	//! The impedance parameters are converted to the control gains \f$ K_v \f$, \f$ K_p \f$, and \f$ K_f \f$ used in generating the reference acceleration.
	//! 
	//! \param R Current rotation matrix or \f$ R \f$
	//! \param w Current body angular velocity vector or \f$ \omega \f$
	//! \param Rd Desired rotation matrix or \f$ R_{des} \f$
	//! \param wd Desired body angular velocity vector or \f$ \omega_{des} \f$
	//! \param wdotd Desired body angular acceleration vector or \f$ \dot{\omega}_{des} \f$
	//! \param n Current body moment vector or \f$ n \f$
	//! \param nd Desired body moment vector or \f$ n_{des} \f$
	//! \param nref Rotational reference force vector or \f$ n_{ref} \f$
	//! \param xi_ Lie algebra position error or \f$ \xi \f$
	//! \param xidot Lie algebra velocity error or \f$ \dot{\xi} \f$
	//! \param eta Lie algebra force error or \f$ \eta \f$
	//  ----------------------------------------------------------
	void refComplianceForce(LieGroup::Rotation const & R, LieGroup::Vector3D const & w, 
		LieGroup::Rotation const & Rd, LieGroup::Vector3D const & wd, LieGroup::Vector3D const & wdotd, 
		LieGroup::Vector3D const & n, LieGroup::Vector3D const & nd, 
		TaskVec & nref,
		TaskVec & xi, TaskVec & xidot,
		TaskVec & eta);
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the rotation reference velocity 
	//!
	//! \details
	//! This computes the rotation reference velocity by
	//!	\f$ \omega_{ref} = \tilde{R} ( \omega_{des} - {\rm dexp}_{-\xi} \dot{\xi}_{ref} ) \f$,
	//! where the Lie algebra reference velocity is defined by
	//! \f$ \dot{\xi}_{ref} = - k_v \xi - k_p \int \xi dt \f$.
	//! 
	//! \param R Current rotation matrix or \f$ R \f$
	//! \param Rd Desired rotation matrix or \f$ R_{des} \f$
	//! \param wd Desired body angular velocity vector or \f$ \omega_{des} \f$
	//! \param wref Rotational reference velocity vector or \f$ \omega_{ref} \f$
	//  ----------------------------------------------------------
	inline void refVelocity(LieGroup::Rotation const & R, 
		LieGroup::Rotation const & Rd, LieGroup::Vector3D const & wd,
		TaskVec & wref)
	{
		TaskVec xi;

		refVelocity(R, Rd, wd, wref, xi);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the rotation reference velocity as well as Lie algebra error vector
	//!
	//! \details
	//! This computes the rotation reference velocity by
	//!	\f$ \omega_{ref} = \tilde{R} ( \omega_{des} - {\rm dexp}_{-\xi} \dot{\xi}_{ref} ) \f$,
	//! where the Lie algebra reference velocity is defined by
	//! \f$ \dot{\xi}_{ref} = - k_v \xi - k_p \int \xi dt \f$.
	//! 
	//! \param R Current rotation matrix or \f$ R \f$
	//! \param Rd Desired rotation matrix or \f$ R_{des} \f$
	//! \param wd Desired body angular velocity vector or \f$ \omega_{des} \f$
	//! \param wref Rotational reference velocity vector or \f$ \omega_{ref} \f$
	//! \param xi Lie algebra position error or \f$ \xi \f$
	//  ----------------------------------------------------------
	void refVelocity(LieGroup::Rotation const & R, 
		LieGroup::Rotation const & Rd, LieGroup::Vector3D const & wd,
		TaskVec & wref, TaskVec & xi);

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the Lie algebra error vector and its derivative
	//! 
	//! \param R Current rotation matrix or \f$ R \f$
	//! \param w Current body angular velocity vector or \f$ \omega \f$
	//! \param Rd Desired rotation matrix or \f$ R_{des} \f$
	//! \param wd Desired body angular velocity vector or \f$ \omega_{des} \f$
	//! \param xi Lie algebra position error or \f$ \xi \f$
	//! \param xidot Lie algebra velocity error or \f$ \dot{\xi} \f$
	//  ----------------------------------------------------------
	void error(LieGroup::Rotation const & R, LieGroup::Vector3D const & w, 
		LieGroup::Rotation const & Rd, LieGroup::Vector3D const & wd, 
		TaskVec & xi, TaskVec & xidot);

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the Lie algebra error vector
	//! 
	//! \param R Current rotation matrix or \f$ R \f$
	//! \param Rd Desired rotation matrix or \f$ R_{des} \f$
	//! \param xi Lie algebra position error or \f$ \xi \f$
	//  ----------------------------------------------------------
	void error(LieGroup::Rotation const & R, LieGroup::Rotation const & Rd, TaskVec & xi);

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

};	// class RotationController

} // namespace NRMKFoundation
