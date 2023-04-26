//! \file MomentumObserver.h
//! \brief Header file for the class MomentumObserver (API of IndySDK)
//!
//! \details
//! This file implements a momentum observer algorithm class
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

#include "NRMKCore.h"

#include "LieGroup/LieGroup.h"

#include "Indy/Indy.h"

namespace NRMKFoundation
{

namespace IndySDK
{
//! \class MomentumObserver
//!
//! \brief
//! This class provides a class useful for momentum observer of the system.

class NRMKCore_API MomentumObserver
{
public:	
	typedef IndyBase::JointVec JointVec;
	typedef IndyBase::JointMat JointMat;

public:
	MomentumObserver();

	//! \brief sets the control period
	//! \param h control period (in msec)
	void setPeriod(double h);

	//! \brief sets the observer gain
	//! \note setPeriod() should be called before this function
	//! \param L observer gain vector
	void setGain(JointVec const & L);

	//! \brief setimate the external torque by momentum observer for a system 
	//! \param subsys subsys containing 
	//! \param tau_ext estimated external torque
	//! \param gacc gravitational acceleration 
	void estimate(IndyBase & subsys, JointVec & tau_ext, LieGroup::Vector3D const & gacc);
		
	//! \brief setimate the external torque by momentum observer for a system without gravity
	//! \param subsys subsys containing 
	//! \param tau_ext estimated external torque
	void estimate(IndyBase & subsys, JointVec & tau_ext);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//! \brief residual vector
	JointVec _r;

	//! \brief observer gain
	JointVec _L;
	
	//! \brief initial momentum
	JointVec _p0;

	//! \brief control period
	double _h;
};

} // namespace IndySDK

} // namespace NRMKFoundation
