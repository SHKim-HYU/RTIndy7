//  ---------------------- Doxygen info ----------------------
//! \file Integrator.h
//!
//! \brief
//! Header file for the class Integrator (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements an integrator class
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
//! \date April 2014
//! 
//! \version 1.6
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

namespace NRMKFoundation
{

//  ---------------------- Doxygen info ----------------------
//! \brief
//! integrates the arbitrary state 
//!
//! \param x state
//! \param xdot state derivative
//! \param dt Time increment
//  ----------------------------------------------------------
static inline void Integrate(double & x, double xdot, double dt)
{
	x += dt * xdot;
}

//  ---------------------- Doxygen info ----------------------
//! \brief
//! integrates the arbitrary position and velocity 
//!
//! \param x position 
//! \param xdot velocity 
//! \param xddot acceleration 
//! \param dt Time increment
//  ----------------------------------------------------------
static inline void Integrate(double & x, double & xdot, double xddot, double dt)
{
	xdot += dt * xddot;
	x += dt * xdot;
}

//  ---------------------- Doxygen info ----------------------
//! \brief
//! integrates the arbitrary state vector
//!
//! \param x state vector
//! \param xdot state derivative vector
//! \param dt Time increment
//  ----------------------------------------------------------
template<typename Derived1, typename Derived2>
static inline void Integrate(Eigen::MatrixBase<Derived1> & x, Eigen::MatrixBase<Derived2> const & xdot, double dt)
{
	x += dt * xdot;
}

//  ---------------------- Doxygen info ----------------------
//! \brief
//! integrates the arbitrary position and velocity vector
//!
//! \param x position vector
//! \param xdot velocity vector
//! \param xddot acceleration vector
//! \param dt Time increment
//  ----------------------------------------------------------
template<typename Derived1, typename Derived2, typename Derived3>
static inline void Integrate(Eigen::MatrixBase<Derived1> & x, Eigen::MatrixBase<Derived2> & xdot, Eigen::MatrixBase<Derived3> const & xddot, double dt)
{
	xdot += dt * xddot;
	x += dt * xdot;
}

namespace internal
{
	template <bool Floating>
	struct eulercromer_algorithm
	{
		template <typename SubsysType>
		inline static void integrate(SubsysType & subsys, double dt, bool const_vel = false)
		{
			if (!const_vel)
			{
				subsys.V() += dt * subsys.Vdot();				
				subsys.qdot() += dt * subsys.qddot();
				//qdot += dt * qddot;			
			}

			LieGroup::Twist Theta = dt * subsys.V();	// since Theta0 = 0
			subsys.T() *= Theta.exp(); 

			subsys.q() += dt * subsys.qdot();
		}
	};

	template <>
	struct eulercromer_algorithm<false>
	{
		template <typename SubsysType>
		inline static void integrate(SubsysType & subsys, double dt, bool const_vel = false)
		{
			//q += dt * qdot; // This is the first-order explicit euler method
			if (!const_vel)
				subsys.qdot() += dt * subsys.qddot();

			subsys.q() += dt * subsys.qdot();
		}
	};

	template <bool Floating>
	struct trapezoidal_algorithm
	{
		template <typename SubsysType>
		inline static void integrate(SubsysType & subsys, double dt, bool const_vel = false)
		{
			typedef typename SubsysType::JointVec JointVec;

			if (!const_vel)
			{
				LieGroup::Twist V_prev = subsys.V();
				subsys.V() += dt * subsys.Vdot();
				LieGroup::Twist Theta = 0.5 * dt * (V_prev + subsys.V());	// since Theta0 = 0
				subsys.T() *= Theta.exp(); 

				JointVec qdot_prev = subsys.qdot();
				subsys.qdot() += dt * subsys.qddot();
				subsys.q() += 0.5 * dt * (qdot_prev + subsys.qdot());
				//qdot += dt * qddot;			
			}
			else
			{
				LieGroup::Twist Theta = dt * subsys.V();	// since Theta0 = 0
				subsys.T() *= Theta.exp(); 

				subsys.q() += dt * subsys.qdot();
			}
		}
	};

	template <>
	struct trapezoidal_algorithm<false>
	{
		template <typename SubsysType>
		inline static void integrate(SubsysType & subsys, double dt, bool const_vel = false)
		{
			typedef typename SubsysType::JointVec JointVec;

			if (!const_vel)
			{
				//q += dt * qdot; // This is the first-order explicit euler method
				JointVec qdot_prev = subsys.qdot();
				subsys.qdot() += dt * subsys.qddot();
				subsys.q() += 0.5 * dt * (qdot_prev + subsys.qdot());
			}
			else
			{
				subsys.q() += dt * subsys.qdot();
			}
		}
	};
}

//  ---------------------- Doxygen info ----------------------
//! \class Integrator
//!
//! \brief
//! This implements a base (numerical) integrator class.
//!
//! \details
//! Class Integrator implements a numerical integrator algorithm for system. 
//! Since the system may be either floating or fixed, 
//! one has to deal with both cases in the derived concrete integrator class. 
//!
//! In order to enforce the static polymorphism the derived class should inherit the class by the CRTP pattern. 
//! See http://en.wikipedia.org/wiki/Curiously_recurring_template_pattern.
//!  
//! \tparam SubsysType Type of the subsys
//! \tparam IntegratorType Derived class 
//!
//! \sa EulerCromerMethod
//  ----------------------------------------------------------
template <typename SubsysType, typename IntegratorType>
class Integrator
{
public:
	//! \returns Reference to the derived object 
	IntegratorType& derived() { return *static_cast<IntegratorType*>(this); }
	//! \returns Constant reference to the derived object 
	const IntegratorType& derived() const { return *static_cast<const IntegratorType*>(this); }

	typedef typename SubsysType::JointVec JointVec; //!< Typedef of the vector having the dimension of the total joint dof

public:	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! integrates the system states 
	//!
	//! \param dt Time increment
	//  ----------------------------------------------------------
	inline void integrate(SubsysType & subsys, double dt, bool const_vel = false)
	{
		derived().integrate(subsys, dt, const_vel);
	}
};

//  ---------------------- Doxygen info ----------------------
//! \class EulerCromerMethod
//!
//! \brief
//! This implements the Euler-Cromer integrator method  (a.k.a. semi-implicit first-order method).
//!
//! \details
//! For floating system it implements the Munthe-Kaas Lie group integrator method. 
//!
//! \tparam SubsysType Type of the subsys
//!
//! \sa Integrator
//  ----------------------------------------------------------
template<typename SubsysType>
class EulerCromerMethod : public Integrator<SubsysType, EulerCromerMethod<SubsysType> >
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! integrates the system states 
	//!
	//! \param dt Time increment
	//  ----------------------------------------------------------
	inline void integrate(SubsysType & subsys, double dt, bool const_vel = false)
	{
		internal::eulercromer_algorithm<internal::IsFloatingJoint<NRMKFoundation::RigidJoint>::value>::integrate(subsys, dt, const_vel);
	}
};

//  ---------------------- Doxygen info ----------------------
//! \class TrapezoidalMethod
//!
//! \brief
//! This implements the Trapezoidal integrator method .
//!
//! \details
//! For floating system it implements the Munthe-Kaas Lie group integrator method. 
//!
//! \tparam SubsysType Type of the subsys
//!
//! \sa Integrator
//  ----------------------------------------------------------
template<typename SubsysType>
class TrapezoidalMethod : public Integrator<SubsysType, TrapezoidalMethod<SubsysType> >
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! integrates the system states 
	//!
	//! \param dt Time increment
	//  ----------------------------------------------------------
	inline void integrate(SubsysType & subsys, double dt, bool const_vel = false)
	{
		internal::trapezoidal_algorithm<internal::IsFloatingJoint<typename SubsysType::EarthJoint>::value>::integrate(subsys, dt, const_vel); 
	}
};

} // namespace NRMKFoundation
