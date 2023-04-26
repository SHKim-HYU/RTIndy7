//  ---------------------- Doxygen info ----------------------
//! \file DisplacementKinematics.h
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
//!	\author Jonghoon Park, <crossover69@gmail.com>
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
// Copyright (C) 2013-2013 Neuromeka <crossover69@gmail.com>

#pragma once

#include "DisplacementKinematics.h"
#include "FunctionalKinematics.h"
#include "QuasiCoordKinematics.h"
#include "../AMBS/Subsys.h"

namespace NRMKFoundation
{
namespace internal
{
}

//  ---------------------- Doxygen info ----------------------
//! \class COMKinematics
//!
//! \brief
//! This implements a center-of-mass task kinematics class
//!
//! \details 
//!	This is the task which defines a center-of-mass of the system. The center-of-mass is defined with respect to
//! a fixed body.
//! 
//! \note 
//! The system should be updated by update() before calling kinematics(). Furthermore, 
//! it should have updated body Jacobians by updateJacobian() before calling jacobian().
//! 
//! \tparam SubsysType Type of the subsys
//!
//! \sa DisplacementKinematics
//  ----------------------------------------------------------
template<typename SubsysType>
class COMKinematics : public DisplacementKinematics<SubsysType, COMKinematics<SubsysType> >
{
public:
	typedef typename DisplacementKinematics<SubsysType, COMKinematics<SubsysType> >::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename DisplacementKinematics<SubsysType, COMKinematics<SubsysType> >::TaskJac TaskJac; //!< Typedef of task Jacobian matrix

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the center-of-mass task kinematics
	//!
	//! \param subsys System object
	//! \param ref Body index of the reference body
	//  ----------------------------------------------------------
	inline COMKinematics(SubsysType const & subsys, int ref = -1)
		: DisplacementKinematics<SubsysType, COMKinematics<SubsysType>>()
		, _ref(ref), _M(0)
	{
		_M = TotalMass(subsys);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the data for center-of-mass task kinematics
	//!
	//! \param ref Body index of the reference body
	//  ----------------------------------------------------------
	inline void set(int ref)
	{
		_ref = ref;

		_M = TotalMass(subsys);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the center-of-mass task kinematics
	//!
	//! \param subsys System object of SubsysType
	//! \param r COM position with respect to the reference body 
	//! \param rdot COM velocity with respect to the reference body 
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::Displacement & r, LieGroup::Vector3D & rdot) const
	{
		r.setZero();
		rdot.setZero();
		
		if (_ref == -1)
		{
			for (int k = 0; k < SubsysType::NUM_BODIES; k++)
			{
				//Eigen::Map<const LieGroup::Displacement> r_k_com(subsys.body(k).com());
				LieGroup::Displacement r_k_com(subsys.body(k).com()[0], subsys.body(k).com()[1], subsys.body(k).com()[2]);
				// FIXME: Check whether Displacement::transform is called
				r_k_com.transform(subsys.body(k).T()); 
				r.noalias() += subsys.body(k).mass()*r_k_com;

				LieGroup::Vector3D rdot_F_k = subsys.body(k).T().R()*subsys.body(k).inertia().topRows<3>()*subsys.body(k).V();
				rdot.noalias() += subsys.body(k).mass()*rdot_F_k;
			}
		}
		else
		{
			Body const & F = subsys.body(_ref);

			for (int k = 0; k < SubsysType::NUM_BODIES; k++)
			{
				if (k == _ref)
					continue;

				LieGroup::HTransform T_F_k = F.T().icascade(subsys.body(k).T());
				LieGroup::Twist V_F_k = subsys.body(k).V() - T_F_k.itransform(F.V());
				
				//Eigen::Map<const LieGroup::Displacement> r_F_k_com(subsys.body(k).com());
				LieGroup::Displacement r_F_k_com(subsys.body(k).com()[0], subsys.body(k).com()[1], subsys.body(k).com()[2]);
				LieGroup::Displacement r_F_k = T_F_k.transform(r_F_k_com); 
				r.noalias() += subsys.body(k).mass()*T_F_k.transform(r_F_k_com);

				LieGroup::Vector3D rdot_F_k = T_F_k.R()*subsys.body(k).inertia().topRows<3>()*V_F_k; 
				rdot.noalias() += subsys.body(k).mass()*rdot_F_k;
			}
		}

		r /= _M;
		rdot /= _M;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the center-of-mass task kinematics
	//!
	//! \param subsys System object of SubsysType
	//! \param r COM position with respect to the reference body 
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::Displacement & r) const
	{
		r.setZero();

		if (_ref == -1)
		{
			for (int k = 0; k < SubsysType::NUM_BODIES; k++)
			{
				//Eigen::Map<const LieGroup::Displacement> r_k_com(subsys.body(k).com());
				LieGroup::Displacement r_k_com(subsys.body(k).com()[0], subsys.body(k).com()[1], subsys.body(k).com()[2]);
				// FIXME: Check whether Displacement::transform is called
				r_k_com.transform(subsys.body(k).T()); 
				r.noalias() += subsys.body(k).mass()*r_k_com;
			}
		}
		else
		{
			Body const & F = subsys.body(_ref);

			for (int k = 0; k < SubsysType::NUM_BODIES; k++)
			{
				if (k == _ref)
					continue;

				LieGroup::HTransform T_F_k = F.T().icascade(subsys.body(k).T());

				//Eigen::Map<const LieGroup::Displacement> r_F_k_com(subsys.body(k).com());
				LieGroup::Displacement r_F_k_com(subsys.body(k).com()[0], subsys.body(k).com()[1], subsys.body(k).com()[2]);
				LieGroup::Displacement r_F_k = T_F_k.transform(r_F_k_com); 
				r.noalias() += subsys.body(k).mass()*T_F_k.transform(r_F_k_com);
			}
		}

		r /= _M;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the center-of-mass task kinematics with the task Jacobian and its derivative
	//!
	//! \param subsys System object of SubsysType
	//! \param r COM position with respect to the reference body 
	//! \param rdot COM velocity with respect to the reference body 
	//! \param J Task Jacobian matrix or \f$ J \f$
	//! \param Jdot Task Jacobian derivative matrix or \f$ \dot{J} \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::Displacement & r, LieGroup::Vector3D & rdot, TaskJac & J, TaskJac & Jdot) const
	{
		r.setZero();
		rdot.setZero();

		J.setZero();
		Jdot.setZero();

		if (_ref == -1)
		{
			for (int k = 0; k < SubsysType::NUM_BODIES; k++)
			{
				//Eigen::Map<const LieGroup::Displacement> r_k_com(subsys.body(k).com());
				LieGroup::Displacement r_k_com(subsys.body(k).com()[0], subsys.body(k).com()[1], subsys.body(k).com()[2]);
				// FIXME: Check whether Displacement::transform is called
				r_k_com.transform(subsys.body(k).T()); 
				r.noalias() += subsys.body(k).mass()*r_k_com;

				// FIXME @ 20140109
				//LieGroup::Vector3D rdot_F_k = T_F_k.R()*(V_F_k.v() + V_F_k.w().cross(r_k_com));
				LieGroup::Vector3D rdot_F_k = subsys.body(k).T().R()*subsys.body(k).inertia().topRows<3>()*subsys.body(k).V(); 
				rdot.noalias() += subsys.body(k).mass()*rdot_F_k;

				// J_F_k & its derivative
				
				// ...
				TaskJac Jcom_F_k = subsys.body(k).inertia().topRows<3>()*subsys.J(k);

				TaskJac Jdotcom_F_k2 = subsys.body(k).inertia().topRows<3>()*subsys.Jdot(k);
				TaskJac Jdotcom_F_k1;
				LieGroup::internal::_cross(subsys.body(k).V().w()[0], subsys.body(k).V().w()[1], subsys.body(k).V().w()[2], Jcom_F_k, Jdotcom_F_k1);

				J.noalias() += subsys.body(k).mass()*subsys.body(k).T().R()*Jcom_F_k;
				Jdot.noalias() += subsys.body(k).mass()*subsys.body(k).T().R()*(Jdotcom_F_k1 + Jdotcom_F_k2);
			}
		}
		else
		{
			Body const & F = subsys.body(_ref);

			for (int k = 0; k < SubsysType::NUM_BODIES; k++)
			{
				if (k == _ref)
					continue;

				LieGroup::HTransform T_F_k = F.T().icascade(subsys.body(k).T());
				//LieGroup::Twist V_F_k = subsys.body(k).V() - T_F_k.itransform(subsys.body(F).V());
				LieGroup::Twist V_F_k = subsys.body(k).V() - T_F_k.itransform(F.V());

				//Eigen::Map<const LieGroup::Displacement> r_F_k_com(subsys.body(k).com());
				LieGroup::Displacement r_F_k_com(subsys.body(k).com()[0], subsys.body(k).com()[1], subsys.body(k).com()[2]);
				// FIXME: Check whether Displacement::transform is called
				r_F_k_com.transform(subsys.body(k).T()); 
				r.noalias() += subsys.body(k).mass()*r_F_k_com;

				//LieGroup::Vector3D rdot_F_k = T_F_k.R()*(V_F_k.v() + V_F_k.w().cross(r_k_com));
				LieGroup::Vector3D rdot_F_k = T_F_k.R()*subsys.body(k).inertia().topRows<3>()*V_F_k; 
				rdot.noalias() += subsys.body(k).mass()*rdot_F_k;

				// J_F_k & its derivative
				SubsysType::JointJac J_F_k;
				T_F_k.itransform(subsys.J(_ref), J_F_k);

				SubsysType::JointJac Jdot_F_k;
				T_F_k.itransform(subsys.Jdot(_ref), Jdot_F_k);

				SubsysType::JointJac Jdot_F_k2;
				V_F_k.adjoint(J_F_k, Jdot_F_k2);
			
				J_F_k = subsys.J(k) - J_F_k;
				Jdot_F_k = subsys.Jdot(k) + Jdot_F_k2 - Jdot_F_k;

				// ...
				TaskJac Jcom_F_k = subsys.body(k).inertia().topRows<3>()*J_F_k;

				TaskJac Jdotcom_F_k2 = subsys.body(k).inertia().topRows<3>()*Jdot_F_k;
				TaskJac Jdotcom_F_k1;
				LieGroup::internal::_cross(V_F_k.w()[0], V_F_k.w()[1], V_F_k.w()[2], Jcom_F_k, Jdotcom_F_k1);
				
				J.noalias() += subsys.body(k).mass()*T_F_k.R()*Jcom_F_k;
				Jdot.noalias() += subsys.body(k).mass()*T_F_k.R()*(Jdotcom_F_k1 + Jdotcom_F_k2);
			}
		}

		r /= _M;
		rdot /= _M;

		J /= _M;
		Jdot /= _M;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the center-of-mass task kinematics with the task Jacobian and its derivative
	//!
	//! \param subsys System object of SubsysType
	//! \param r COM position with respect to the reference body 
	//! \param J Task Jacobian matrix or \f$ J \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::Displacement & r, TaskJac & J) const
	{
		r.setZero();
		J.setZero();

		if (_ref == -1)
		{
			for (int k = 0; k < SubsysType::NUM_BODIES; k++)
			{
				//Eigen::Map<const LieGroup::Displacement> r_k_com(subsys.body(k).com());
				LieGroup::Displacement r_k_com(subsys.body(k).com()[0], subsys.body(k).com()[1], subsys.body(k).com()[2]);
				r_k_com.transform(subsys.body(k).T()); 				
				r.noalias() += subsys.body(k).mass()*r_k_com;

				// J_F_k & its derivative

				// ...
				TaskJac Jcom_F_k = subsys.body(k).inertia().topRows<3>()*subsys.J(k);
				J.noalias() += subsys.body(k).mass()*subsys.body(k).T().R()*Jcom_F_k;
			}
		}
		else
		{
			Body const & F = subsys.body(_ref);

			for (int k = 0; k < SubsysType::NUM_BODIES; k++)
			{
				if (k == _ref)
					continue;

				LieGroup::HTransform T_F_k = F.T().icascade(subsys.body(k).T());
				LieGroup::Twist V_F_k = subsys.body(k).V() - T_F_k.itransform(F.V());

				//Eigen::Map<const LieGroup::Displacement> r_F_k_com(subsys.body(k).com());
				LieGroup::Displacement r_F_k_com(subsys.body(k).com()[0], subsys.body(k).com()[1], subsys.body(k).com()[2]);
				r_F_k_com.transform(T_F_k); 				
				r.noalias() += subsys.body(k).mass()*r_F_k_com;

				// J_F_k & its derivative
				SubsysType::JointJac J_F_k;
				T_F_k.itransform(subsys.J(_ref), J_F_k);
				J_F_k = subsys.J(k) - J_F_k;

				// ...
				TaskJac Jcom_F_k = subsys.body(k).inertia().topRows<3>()*J_F_k;
				J.noalias() += subsys.body(k).mass()*T_F_k.R()*Jcom_F_k;
			}
		}

		r /= _M;
		J /= _M;
	}

private:
	inline double TotalMass(SubsysType const & subsys)
	{
		double M = 0;

		for (int k = 0; k < SubsysType::NUM_BODIES; k++)
		{	
			if (k == _ref) // do not take into account the body _ref
				continue;

			M += subsys.body(k).mass();
		}

		return M;
	}

private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Body index of the reference body
	//  ----------------------------------------------------------
	int _ref; 

	//  ---------------------- Doxygen info ----------------------
	//! \brief total mass of the system
	//  ----------------------------------------------------------
	double _M;	
};

//  ---------------------- Doxygen info ----------------------
//! \class ProjectedCOMKinematics
//!
//! \brief
//! This implements a projected-center-of-mass task kinematics class
//!
//! \details 
//!	This is the task which defines a center-of-mass of the system. The center-of-mass is defined with respect to
//! a fixed body.
//! 
//! \note 
//! The system should be updated by update() before calling kinematics(). Furthermore, 
//! it should have updated body Jacobians by updateJacobian() before calling jacobian().
//! 
//! \tparam SubsysType Type of the subsys
//! \tparam Axis1 the first axis to define the plane. It is 0 (meaning x-axis)  by default.
//! \tparam Axis2 the second axis to define the plane. It is 1 (meaning y-axis)  by default.
//!
//! \sa FunctionalKinematics
//  ----------------------------------------------------------
template<typename SubsysType, int Axis1 = 0, int Axis2 = 1>
class ProjectedCOMKinematics : public FunctionalKinematics<SubsysType, 2, ProjectedCOMKinematics<SubsysType, Axis1, Axis2> >
{
public:
	typedef typename  FunctionalKinematics<SubsysType, 2, ProjectedCOMKinematics<SubsysType, Axis1, Axis2> >::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename  FunctionalKinematics<SubsysType, 2, ProjectedCOMKinematics<SubsysType, Axis1, Axis2> >::TaskJac TaskJac; //!< Typedef of task Jacobian matrix

	typedef typename COMKinematics<SubsysType>::TaskJac TaskJacCOM;

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the center-of-mass task kinematics
	//!
	//! \param subsys System object
	//! \param ref Body index of the reference body
	//  ----------------------------------------------------------
	inline ProjectedCOMKinematics(SubsysType const & subsys, int ref = -1)
		: FunctionalKinematics<SubsysType, 2, ProjectedCOMKinematics<SubsysType,  Axis1, Axis2> >()
		, _com(subsys, ref)
	{		
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the center-of-mass task kinematics
	//!
	//! \param subsys System object of SubsysType
	//! \param r COM position with respect to the reference body 
	//! \param rdot COM velocity with respect to the reference body 
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, TaskVec & p, TaskVec & pdot) const
	{
		LieGroup::Displacement r;
		LieGroup::Vector3D rdot;

		_com.kinematics(subsys, r, rdot);

		p << r[Axis1], r[Axis2];
		pdot << rdot[Axis1], rdot[Axis2];		
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the center-of-mass task kinematics
	//!
	//! \param subsys System object of SubsysType
	//! \param r COM position with respect to the reference body 
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, TaskVec & p) const
	{
		LieGroup::Displacement r;

		_com.kinematics(subsys, r);

		p << r[Axis1], r[Axis2];
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the center-of-mass task kinematics with the task Jacobian and its derivative
	//!
	//! \param subsys System object of SubsysType
	//! \param r COM position with respect to the reference body 
	//! \param rdot COM velocity with respect to the reference body 
	//! \param J Task Jacobian matrix or \f$ J \f$
	//! \param Jdot Task Jacobian derivative matrix or \f$ \dot{J} \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, TaskVec & p, TaskVec & pdot, TaskJac & J, TaskJac & Jdot) const
	{
		LieGroup::Displacement r;
		LieGroup::Vector3D rdot;
		TaskJacCOM J_com;
		TaskJacCOM Jdot_com;

		_com.jacobian(subsys, r, rdot, J_com, Jdot_com);

		p << r[Axis1], r[Axis2];
		pdot << rdot[Axis1], rdot[Axis2];		

		J << J_com.row(Axis1), J_com.row(Axis2);
		Jdot << Jdot_com.row(Axis1), Jdot_com.row(Axis2);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the center-of-mass task kinematics with the task Jacobian and its derivative
	//!
	//! \param subsys System object of SubsysType
	//! \param r COM position with respect to the reference body 
	//! \param J Task Jacobian matrix or \f$ J \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, TaskVec & p, TaskJac & J) const
	{
		LieGroup::Displacement r;
		TaskJacCOM J_com;

		_com.jacobian(subsys, r, J_com);

		p << r[Axis1], r[Axis2];

		J << J_com.row(Axis1), J_com.row(Axis2);
	}

private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Body index of the reference body
	//  ----------------------------------------------------------
	COMKinematics<SubsysType> _com; 
};

//  ---------------------- Doxygen info ----------------------
//! \class AngularMomentumKinematics
//!
//! \brief
//! This implements a angular momentum kinematics, a concrete quasi-coordinate task class.
//! 
//! \details 
//! This defines the angular momentum of a system with respect to a fixed reference body. 
//!
//! \tparam SubsysType Type of the subsys
//! 
//! \sa QuasiCoordKinematics
//  ----------------------------------------------------------
template <typename SubsysType>
class AngularMomentumKinematics : public QuasiCoordKinematics<SubsysType, 3, AngularMomentumKinematics<SubsysType> >
{
public:
	typedef typename QuasiCoordKinematics<SubsysType, 3, AngularMomentumKinematics<SubsysType> >::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename QuasiCoordKinematics<SubsysType, 3, AngularMomentumKinematics<SubsysType> >::TaskJac TaskJac; //!< Typedef of task Jacobian matrix
	

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the default null velocity kinematics class
	//!
	//! \param subsys System object
	//! \param ref Body index of the reference body
	//  ----------------------------------------------------------
	inline AngularMomentumKinematics(SubsysType const & subsys, int ref = -1)
		: QuasiCoordKinematics<SubsysType, 3, AngularMomentumKinematics>()
		, _ref(ref)
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the angular momentum kinematics 
	//!
	//! \param subsys System object of SubsysType
	//! \param L angular momentum with respect to the reference body 
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::Vector3D &, LieGroup::Vector3D & L) const
	{
		L.setZero();

		if (_ref == -1)
		{
			for (int k = 0; k < SubsysType::NUM_BODIES; k++)
			{
				LieGroup::HTransform const & T_k = subsys.body(k).T();
				LieGroup::Twist const & V_k = subsys.body(k).V();

				Vector6d L_k = subsys.body(k).inertia()*V_k; 
				L.noalias() += T_k.r().cross(T_k.R()*L_k.head<3>()) +T_k.R()*L_k.tail<3>();
			}
		}
		else
		{
			Body const & F = subsys.body(_ref);

			for (int k = 0; k < SubsysType::NUM_BODIES; k++)
			{
				if (k == _ref)
					continue;

				LieGroup::HTransform T_F_k = F.T().icascade(subsys.body(k).T());
				LieGroup::Twist V_F_k = subsys.body(k).V() - T_F_k.itransform(F.V());

				Vector6d L_F_k = subsys.body(k).inertia()*V_F_k; 
				L.noalias() += T_F_k.r().cross(T_F_k.R()*L_F_k.head<3>()) + T_F_k.R()*L_F_k.tail<3>();
			}
		}	
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! do nothing.
	//!
	//! \param subsys System object of SubsysType
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::Vector3D &) const
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the angular momentum kinematics 
	//!
	//! \param subsys System object of SubsysType
	//! \param L angular momentum with respect to the reference body 
	//! \param J Task Jacobian matrix or \f$ J \f$
	//! \param Jdot Task Jacobian derivative matrix or \f$ \dot{J} \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::Vector3D &, LieGroup::Vector3D & L, TaskJac & J, TaskJac & Jdot) const
	{
		L.setZero();
		J.setZero();
		Jdot.setZero();

		if (_ref == -1)
		{
			for (int k = 0; k < SubsysType::NUM_BODIES; k++)
			{
				LieGroup::HTransform const & T_k = subsys.body(k).T();
				LieGroup::Twist const & V_k = subsys.body(k).V();

				Vector6d L_k = subsys.body(k).inertia()*V_k; 
				L.noalias() += T_k.r().cross(T_k.R()*L_k.head<3>()) +T_k.R()*L_k.tail<3>();

				// J_F_k & its derivative
				SubsysType::JointJac J_F_k = subsys.body(k).inertia()*subsys.J(k);
				SubsysType::JointJac Jdot_F_k = subsys.body(k).inertia()*subsys.Jdot(k);

				// ...
				TaskJac J_k;
				LieGroup::internal::_cross(T_k.r()[0], T_k.r()[1], T_k.r()[2], T_k.R()*J_F_k.topRows<3>(), J_k);

				J_k.noalias() += T_k.R()*J_F_k.bottomRows<3>();

				// ...
				// reuse Jdot_F_k2
				V_k.coadjoint(J_F_k, Jdot_F_k2);
				Jdot_F_k2 += Jdot_F_k;

				TaskJac Jdot_k;
				LieGroup::internal::_cross(T_k.r()[0], T_k.r()[1], T_k.r()[2], T_k.R()*Jdot_F_k2.topRows<3>(), Jdot_k);

				Jdot_k.noalias() += T_k.R()*Jdot_F_k2.bottomRows<3>();

				J.noalias() += J_k;
				Jdot.noalias() += Jdot_k;
			}
		}
		else
		{
			Body const & F = subsys.body(_ref);

			for (int k = 0; k < SubsysType::NUM_BODIES; k++)
			{
				if (k == _ref)
					continue;

				LieGroup::HTransform T_F_k = F.T().icascade(subsys.body(k).T());
				LieGroup::Twist V_F_k = subsys.body(k).V() - T_F_k.itransform(F.V());

				Vector6d L_F_k = subsys.body(k).inertia()*V_F_k; 
				L.noalias() += T_F_k.r().cross(T_F_k.R()*L_F_k.head<3>()) + T_F_k.R()*L_F_k.tail<3>();

				// J_F_k & its derivative
				SubsysType::JointJac J_F_k;
				T_F_k.itransform(subsys.J(_ref), J_F_k);

				SubsysType::JointJac Jdot_F_k;
				T_F_k.itransform(subsys.Jdot(_ref), Jdot_F_k);

				SubsysType::JointJac Jdot_F_k2;
				V_F_k.adjoint(J_F_k, Jdot_F_k2);

				J_F_k = subsys.J(k) - J_F_k;
				Jdot_F_k = subsys.Jdot(k) + Jdot_F_k2 - Jdot_F_k;

				J_F_k = subsys.body(k).inertia()*J_F_k;
				Jdot_F_k = subsys.body(k).inertia()*Jdot_F_k;

				// ...
				TaskJac J_k;
				LieGroup::internal::_cross(T_F_k.r()[0], T_F_k.r()[1], T_F_k.r()[2], T_F_k.R()*J_F_k.topRows<3>(), J_k);

				J_k.noalias() += T_F_k.R()*J_F_k.bottomRows<3>();

				// ...
				// reuse Jdot_F_k2
				V_F_k.coadjoint(J_F_k, Jdot_F_k2);
				Jdot_F_k2 += Jdot_F_k;

				TaskJac Jdot_k;
				LieGroup::internal::_cross(T_F_k.r()[0], T_F_k.r()[1], T_F_k.r()[2], T_F_k.R()*Jdot_F_k2.topRows<3>(), Jdot_k);

				Jdot_k.noalias() += T_F_k.R()*Jdot_F_k2.bottomRows<3>();

				J.noalias() += J_k;
				Jdot.noalias() += Jdot_k;
			}
		}		
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the angular momentum kinematics 
	//!
	//! \param subsys System object of SubsysType
	//! \param J Task Jacobian matrix or \f$ J \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::Vector3D &, LieGroup::Vector3D & J) const
	{
		J.setZero()

		if (_ref == -1)
		{
			for (int k = 0; k < SubsysType::NUM_BODIES; k++)
			{
				LieGroup::HTransform const & T_k = subsys.body(k).T();

				// J_F_k & its derivative
				SubsysType::JointJac J_F_k = subsys.body(k).inertia()*subsys.J(k);

				// ...
				TaskJac J_k;
				LieGroup::internal::_cross(T_k.r()[0], T_k.r()[1], T_k.r()[2], T_k.R()*J_F_k.topRows<3>(), J_k);

				J.noalias() += J_k + T_k.R()*J_F_k.bottomRows<3>();
			}
		}
		else
		{
			Body const & F = subsys.body(_ref);

			for (int k = 0; k < SubsysType::NUM_BODIES; k++)
			{
				if (k == _ref)
					continue;

				LieGroup::HTransform T_F_k = F.T().icascade(subsys.body(k).T());
				
				// J_F_k & its derivative
				SubsysType::JointJac J_F_k;
				T_F_k.itransform(subsys.J(_ref), J_F_k);

				J_F_k = subsys.J(k) - J_F_k;
				J_F_k = subsys.body(k).inertia()*J_F_k;

				// ...
				TaskJac J_k;
				LieGroup::internal::_cross(T_F_k.r()[0], T_F_k.r()[1], T_F_k.r()[2], T_F_k.R()*J_F_k.topRows<3>(), J_k);

				J.noalias() += J_k + T_F_k.R()*J_F_k.bottomRows<3>();
			}
		}	
	}
		
private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Body index of the reference body
	//  ----------------------------------------------------------
	int _ref;
};

} // namespace NRMKFoundation
