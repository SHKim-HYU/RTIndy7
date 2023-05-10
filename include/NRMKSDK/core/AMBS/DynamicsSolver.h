//  ---------------------- Doxygen info ----------------------
//! \file DynamicsSolver.h
//!
//! \brief
//! Header file for the class DynamicsSolver (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a dynamics solver algorithm class
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
//! \date August 2015
//! 
//! \version 1.9
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013-2014 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

// This file is part of NRMKFoundation, a lightweight C++ template library
// for robot motion control.
//
// Copyright (C) 2013-2014 Neuromeka <coolcat@neuromeka.com>

#pragma once

#include "LieGroup/LieGroup.h"

#include "Body.h"
#include "Joint.h"
#include "FlexibleJoint.h"

#include "CustomDynamics.h"

#include "Integrator.h"

namespace NRMKFoundation
{
namespace internal
{
}

enum ConstraintSolverState
{											
	CSS_RESOLVED = 0,
	CSS_NEED_SYSTEM_UPDATE,
};

//  ---------------------- Doxygen info ----------------------
//! \class DynamicsSolver
//!
//! \brief
//! This implements a dynamics solver class.
//!
//! \details
//! Class DynamicsSolver models any type of forward dynamics algorithm for the given system. 
//! Since the system may be either floating or fixed, 
//! one has to deal with both cases in the derived concrete integrator class. 
//! In addition, it is supposed that after the member function update() has been executed, 
//! all body external wrench should be cleared as well as the control body wrench and the joint torque vector. 
//!  
//! \tparam ForwardDynamicsType Type of the ForwardDynamics class
//! \tparam IntegratorType Type of the Integrator class
//!
//! \sa EulerCromerMethod
//  ----------------------------------------------------------
template <typename ForwardDynamicsType, typename IntegratorType = EulerCromerMethod<typename ForwardDynamicsType::SubsysType> >
class DynamicsSolver
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides Typedefs 
	//  ----------------------------------------------------------
	typedef typename ForwardDynamicsType::SubsysType SubsysType;	//!< Typedef of the subsys
	typedef typename SubsysType::JointVec JointVec; //!< Typedef of the vector having the dimension of the total joint dof

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides compile-time constants of the subsys
	//  ----------------------------------------------------------
	enum 
	{	
		NUM_BODIES = SubsysType::NUM_BODIES,  //!< Number of bodies
		JOINT_DOF = SubsysType::JOINT_DOF, //!< Joint degrees-of-freedom
		EARTH_DOF = 0, //!< Degrees-of-freedom of the earthing joint
	};

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the dynamics solver object
	//!
	//! \param fdyn forward dynamics object
	//  ----------------------------------------------------------
	inline DynamicsSolver(ForwardDynamicsType & fdyn)
		: _css(CSS_NEED_SYSTEM_UPDATE)
		, _fdyn(fdyn), _integrator()
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! updates the system kinematics context
	//  ----------------------------------------------------------
	inline void updateRobot(bool pos_only = false)
	{
		if (_css == CSS_NEED_SYSTEM_UPDATE)
			_fdyn.subsys().update(pos_only);
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! integrates free dynamics 
	//!
	//! \param t current time
	//! \param delT time step
	//  ----------------------------------------------------------
	inline void step(double t, double delT)
	{
		updateCustomDynamics(t);
				
		SubsysType & subsys = _fdyn.subsys();

		/// FIXME @20151012
		// for flexible joint
		for (volatile int j = 0; j < SubsysType::NUM_TOTAL_JOINTS; j++)
		{
			/// FIXME @20160312
			FlexibleJoint *fj = subsys.joint(j).flexibleJoint(); 	// subsys.joint(0) is base joint
			if (fj)
			{
				/// FIXME @20151022
				for (int i = 0; i < subsys.joint(j).dof(); i++)
				{
					int idx = subsys.joint(j).index() + i;
					subsys.tau()[idx] -= fj->frictionTorque(subsys.q()[idx], subsys.qdot()[idx], subsys.tau()[idx]);

					fj[i].computeAcc(subsys.q()[idx], subsys.qdot()[idx], subsys.qddot()[idx], subsys.tau()[idx]);
					fj[i].integrate();
				}
			}
		}

		_fdyn.solve();

		_integrator.integrate(_fdyn.subsys(), delT);

		_css = CSS_NEED_SYSTEM_UPDATE;

		// Clear all wrenches and torques
		subsys.clearAllBodyWrenches();

		// FIXME @20150916 Moved to main
		// subsys.F().setZero();
		// subsys.tau().setZero();
	}

	// FIXME @20130114: This should move to SubsysBase
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! updates the system custom dynamics
	//  ----------------------------------------------------------
	inline void updateCustomDynamics(double t)
	{
		internal::custom_system_dynamics_algorithm<internal::HasCustomSystemDynamics<SubsysType>::value>::updateCustomDynamics(_fdyn.subsys(), t); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the system integration period
	//  ----------------------------------------------------------
	inline void setPeriod(double dT)
	{
		_delT = dT;
		_fdyn.setPeriod(dT);
	}

// public:
// 	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Constraint solver state
	//  ----------------------------------------------------------
	ConstraintSolverState _css;

	//  ---------------------- Doxygen info ----------------------
	//! \brief System object
	//  ----------------------------------------------------------
	ForwardDynamicsType & _fdyn;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Default numerical integrator
	//  ----------------------------------------------------------
	IntegratorType _integrator;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Numerical integration period
	//  ----------------------------------------------------------
	double _delT;	
};

} // namespace NRMKFoundation
