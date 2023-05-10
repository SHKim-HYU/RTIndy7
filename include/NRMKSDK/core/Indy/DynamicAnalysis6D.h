//! \file DynamicAnalysis.h
//! \brief Header file for the class DynamicAnalysis (API of IndySDK)
//!
//! \details
//! This file implements an articulated multibody system class
//! to be used for the interface of NRMKFoundation library
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
//! \note Copyright (C) 2013--2017 Neuromeka

#pragma once

#include "LieGroup/LieGroup.h"

#include "Indy6D.h"

namespace NRMKFoundation
{
namespace IndySDK
{
//! \class DynamicAnalysis
//! \brief This class provides a set of functions useful for dynamic analysis of the system.
class DynamicAnalysis6D
{
public:
	typedef IndyBase6D::JointMat JointMat;
	typedef IndyBase6D::JointVec JointVec;

public:
	//! \brief constructs the joint dynamics matrices 
	//! \param subsys Indy object
	//! \param M System inertia matrix (of type JointMat)
	//! \param C System bias matrix (of type JointMat)
	//! \param G System gravity vector (of type JointVec)
	//! \param gacc Gravitation acceleration vector (represented with respect to the global reference frame)
	//!
	//! \note 
	//! M and C are square matrices of dimension (JOINT_DOF = 6) and 
	//! G is a vector of the same dimension.  
	//! This function should be called after system's kinematic context has been updated by calling update().
	static void JointDynamics(IndyBase6D & subsys, JointMat & M, JointMat & C, JointVec & G, LieGroup::Vector3D const & gacc);
};

} // namespace IndySDK

} // namespace NRMKFoundation
