//  ---------------------- Doxygen info ----------------------
//! \file CustomDynamics.h
//!
//! \brief
//! Header file for the class CustomDynamics (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a custom dynamics algorithm class
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
//! \date March 2014
//! 
//! \version 1.7
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013-2014 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------
#pragma once

//#include "LieGroup/LieGroup.h"

#include "Body.h"
#include "Joint.h"

namespace NRMKFoundation
{
namespace internal
{
	//  ---------------------- Doxygen info ----------------------
	//! \brief Trait class to determine whether a system has custom dynamics
	//! 
	//! \note 
	//! Check value to see if the system has custom dynamics or not.
	//  ----------------------------------------------------------
	template<typename SystemType>
	struct HasCustomSystemDynamics
	{
		static const bool value = false; //!< value is false for general joint types
	};

	//  ---------------------- Doxygen info ----------------------
	//! \brief Trait class to determine whether a body has custom dynamics
	//! 
	//! \note 
	//! Check value to see if the body has custom dynamics or not.
	//  ----------------------------------------------------------
	template<typename BodyType>
	struct HasCustomBodyDynamics
	{
		static const bool value = true; //!< value is false for general joint types
	};

	template<>
	struct HasCustomBodyDynamics<NRMKFoundation::Body>
	{
		static const bool value = false; //!< value is false for general joint types
	};

	//  ---------------------- Doxygen info ----------------------
	//! \brief Trait class to determine whether a joint has custom dynamics
	//! 
	//! \note 
	//! Check value to see if the joint has custom dynamics or not.
	//  ----------------------------------------------------------
	template<typename JointType>
	struct HasCustomJointDynamics
	{
		static const bool value = false; //!< value is false for general joint types
	};

	template <bool hasCustomDynamics>
	struct custom_system_dynamics_algorithm
	{
		template <typename SubsysType>
		inline static void updateCustomDynamics(SubsysType & subsys, double t)
		{
		}
	};

	template <>
	struct custom_system_dynamics_algorithm<true>
	{
		template <typename SubsysType>
		inline static void updateCustomDynamics(SubsysType & subsys, double t)
		{
			// Update body custom dynamics
			for (int k = 0; k < SubsysType::NUM_BODIES; k++)
			{
				updateCustomBodyDynamics(subsys.body(k), t); 

// 				Body & b = subsys.body(k);
// 				//Joint const & j = subsys.joint(k);			
// 
// 				b.updateCustomDynamics(t);
// 				b.updateExtWrench();
			}
			//}

// 			// Need to update body Jacobians
// 			// FIXME @20140116: Need to optimize computation
// 			//	since it is necessary to update only the body Jacobians of bodies which require effective dynamics 
// 			subsys.updateJacobian();
// 			subsys.updateConstraints();
// 
// 			Eigen::Matrix<double, 1, 6> Jc;
// 			Eigen::Matrix<double, 1, 6> Jdotc;
// 			subsys.constraintJacobian(Jc, Jdotc);
// 
// 			Eigen::Matrix<double, 1, 1> H;
// 			Eigen::Matrix<double, 1, 1> h;
// 
// 			Eigen::Matrix<double, 6, 2> sol;
// 			fd.effectiveDynamics(Jc, Jdotc, H, h, sol);
// 
// 			NRMKHelper::LCP<1> lcp(H, -h);

			// Pass 2:
			/*
			if (subsys.isFloating())
			{
				// initialize AB inertia and bias
				Body & base = subsys.body(0);

				//////////////////////////////////////////////////////////////////////////
				// ADDED @20140113: Temporarily
				if (base.needEffectiveDynamics())
				{
					Matrix6d A;
					Vector6d b;

					fd.effectiveDynamics(0, A, b);
					base.updateExtWrenchByEffectiveDynamics(A, b);
				}

				for (int k = 1; k < SubsysType::NUM_BODIES; k++)
				{
					Body & b = subsys.body(k);
					//Joint const & j = subsys.joint(k);			

					//////////////////////////////////////////////////////////////////////////
					// ADDED @20140113: Temporarily
					if (b.needEffectiveDynamics())
					{
						Matrix6d A;
						Vector6d p;

						fd.effectiveDynamics(k, A, p);
						b.updateExtWrenchByEffectiveDynamics(A, p);
					}
				}
			}
			else
			{
				// initialize AB inertia and bias
				for (int k = 0; k < SubsysType::NUM_BODIES; k++)
				{
					Body & b = subsys.body(k);
					//Joint const & j = subsys.joint(k);			

					//////////////////////////////////////////////////////////////////////////
					// ADDED @20140113: Temporarily
					if (b.needEffectiveDynamics())
					{
						Matrix6d A;
						Vector6d p;

						fd.effectiveDynamics(k, A, p);
						b.updateExtWrenchByEffectiveDynamics(A, p);
					}
				}
			}
			*/
		}

		template <typename BodyType>
		inline static void updateCustomBodyDynamics(BodyType & body, double t)
		{
			//internal::custom_body_dynamics_algorithm<internal::HasCustomBodyDynamics<BodyType>::value>::updateCustomDynamics(body, t); 
			body.updateCustomDynamics(t);
			body.updateExtWrench();
		}
	};

	template <bool hasCustomDynamics>
	struct custom_body_dynamics_algorithm
	{
		template <typename BodyType>
		inline static void updateCustomDynamics(BodyType & body, double t)
		{
		}
	};

	template <>
	struct custom_body_dynamics_algorithm<true>
	{
		template <typename BodyType>
		inline static void updateCustomDynamics(BodyType & body, double t)
		{
			body.updateCustomDynamics(t);
			body.updateExtWrench();
		}
	};

	template <bool hasCustomDynamics>
	struct custom_joint_dynamics_algorithm
	{
		template <typename JointType>
		inline static void updateCustomDynamics(JointType & joint, double t)
		{
		}
	};

	template <>
	struct custom_joint_dynamics_algorithm<true>
	{
		template <typename JointType>
		inline static void updateCustomDynamics(JointType & joint, double t)
		{
			joint.updateCustomDynamics(t);
			joint.updateTorque();
		}
	};

} // namespace internal
} // namespace NRMKFoundation
