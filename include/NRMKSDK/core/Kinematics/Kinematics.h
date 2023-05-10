//  ---------------------- Doxygen info ----------------------
//! \file Kinematics.h
//!
//! \brief
//! Header file for the class Kinematics (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a kinematics class
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

// This file is part of NRMKFoundation, a lightweight C++ template library
// for robot motion control.
//
// Copyright (C) 2013-2013 Neuromeka <coolcat@neuromeka.com>

#pragma once

#include <Eigen/Eigen>

namespace NRMKFoundation
{
namespace internal
{
}

//  ---------------------- Doxygen info ----------------------
//! \class Kinematics
//!
//! \brief
//! This implements a base task kinematics class.
//!
//! \details
//! Class Kinematics models any type of kinematics algorithm which computes the position-level task value and 
//! the velocity-level task value as well as the Jacobian matrix and its derivative relating the joint velocity vector 
//! and the velocity-level task vector for the given system. 
//! They are called the task position, task velocity, and task Jacobian and task Jacobian derivative. 
//! Since there may exists many types to represent task position, task velocity, and task accelerations, 
//! they are specified as the template arguments. 
//! As different system may have different kinematics algorithm, the type of the system is also a template parameter.
//!
//! In order to enforce the static polymorphism the derived class should inherit the class by the CRTP pattern. 
//! See http://en.wikipedia.org/wiki/Curiously_recurring_template_pattern.
//!  
//! \tparam SubsysType Type of the subsys
//! \tparam _DIM Dimension of the task variable
//! \tparam KinematicsType Derived class 
//! \tparam _PosType Type of the position-level task variable. 
//!		The default type is Eigen::Matrix<double, _DIM, 1>.
//! \tparam _VelType Type of the velocity-level task variable.
//!		The default type is Eigen::Matrix<double, _DIM, 1>.
//! \tparam _AccType Type of the acceleration-level task variable.
//!		The default type is same as _VelType.
////! \tparam _ForceType Type of the force-level task variable.
////!		The default type is same as _VelType.
//!
//! \sa FunctionalKinematics
//! \sa QuasiCoordKinematics
//! \sa DisplacementKinematics
//! \sa RotationKinematics
//! \sa HTransformKinematics
//  ----------------------------------------------------------
template<typename _SubsysType, int _DIM, typename KinematicsType, typename _PosType = Eigen::Matrix<double, _DIM, 1>, typename _VelType = Eigen::Matrix<double, _DIM, 1>, typename _AccType = _VelType, typename _ForceType = _VelType>
class Kinematics
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \enum
	//! 
	//! \brief Provides compile-time constants 
	//  ----------------------------------------------------------
	enum 
	{	
		// FIXME @20140221
		//JOINT_DOF = _SubsysType::JOINT_DOF, //!< Degrees-of-freedom of the total joints of the system (includeing an earthing joint) 
		JOINT_DOF = _SubsysType::SYSTEM_DOF, //!< Degrees-of-freedom of the total joints of the system (includeing an earthing joint) 
		DIM = _DIM, //!< Task dimension
	};

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides Typedefs 
	//  ----------------------------------------------------------
	typedef _SubsysType SubsysType;
	typedef _PosType PosType; //!< Typedef of the type for position variable
	typedef _VelType VelType; //!< Typedef of the type for velocity variable
	typedef _AccType AccType; //!< Typedef of the type for acceleration variable
	typedef _ForceType ForceType; //!< Typedef of the type for force variable
	typedef Eigen::Matrix<double, DIM, 1> TaskVec; //!< Typedef of task vector
	typedef Eigen::Matrix<double, DIM, JOINT_DOF> TaskJac; //!< Typedef of task Jacobian matrix

	//! \returns Reference to the derived object 
	inline KinematicsType& derived() { return *static_cast<KinematicsType*>(this); }
	//! \returns Constant reference to the derived object 
	inline const KinematicsType& derived() const { return *static_cast<const KinematicsType*>(this); }

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the task kinematics 
	//!
	//! \details
	//! It should update the task position and velocity values for the system.
	//! It is mandatory that the system (of type SubsysType) be updated before calling this function.
	//! 
	//! \param subsys System object
	//! \param pos Task position 
	//! \param vel Task velocity 
	//  ----------------------------------------------------------
	void kinematics(SubsysType const & subsys, PosType & pos, VelType & vel) const
	{
		derived().kinematics(subsys, pos, vel);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the task kinematics 
	//!
	//! \details
	//! It should update the task position values for the system.
	//! It is mandatory that the system (of type SubsysType) be updated before calling this function.
	//! 
	//! \param subsys System object
	//! \param pos Task position 
	//  ----------------------------------------------------------
	void kinematics(SubsysType const & subsys, PosType & pos) const
	{
		derived().kinematics(subsys, pos);
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the task kinematics with the task Jacobian and its derivative
	//!
	//! \details
	//! It should update the task position and velocity values
	//! as well as the task Jacobian and its derivative for the system.
	//! It is mandatory that the system (of type SubsysType) be updated before calling this function.
	//! 
	//! \param subsys System object
	//! \param pos Task position 
	//! \param vel Task velocity 
	//! \param J Task jacobian
	//! \param Jdot Task jacobian derivative 
	//  ----------------------------------------------------------
	void jacobian(SubsysType const & subsys, PosType & pos, VelType & vel, TaskJac & J, TaskJac & Jdot) const
	{
		derived().jacobian(subsys, pos, vel, J, Jdot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the task position and the task Jacobian 
	//!
	//! \details
	//! It should update the task position values as well as the task Jacobian for the system.
	//! It is mandatory that the system (of type SubsysType) be updated before calling this function.
	//! 
	//! \param subsys System object
	//! \param pos Task position 
	//! \param J Task jacobian
	//  ----------------------------------------------------------
	void jacobian(SubsysType const & subsys, PosType & pos, TaskJac & J) const
	{
		derived().jacobian(subsys, pos, J);
	}
};

} // namespace NRMKFoundation
