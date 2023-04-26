//  ---------------------- Doxygen info ----------------------
//! \file HTransformKinematics.h
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

#include "Kinematics.h"
#include "../AMBS/Subsys.h"

namespace NRMKFoundation
{
namespace internal
{
}

//  ---------------------- Doxygen info ----------------------
//! \class HTransformKinematics
//!
//! \brief
//! This implements a homogeneous transform kinematics class
//!
//! \details 
//!	Homogeneous transform tasks are described in terms of a homogeneous transformation matrix, \f$ T \f$,
//! and its twist \f$ V \f$ such that \f$ \left \lceil V \right \rceil = T^{-1} \dot{T} \f$. 
//! They are called the task transformation and task twist, respectively.
//! The kinematic relationship with joint position and velocity vectors \f$ q \f$ and \f$ \dot{q} \f$ 
//! are defined by \f$ T = T(q) \f$ and \f$ V = J(q) \dot{q} \f$.
//! User should provide the algorithm to compute \f$ T \f$, \f$ V \f$, \f$ J \f$, and \f$ \dot{J} \f$ 
//! to implement a concrete homogeneous transform task.
//! 
//! \note
//! Users should inherit this class to define a customized kinematics.
//! See HTransformAbsTaskKinematics and HTransformRelTaskKinematics.
//! 
//! \tparam SubsysType Type of the subsys
//! \tparam HTransformKinematicsType Derived homogeneous transform kinematics class 
//!
//! \sa HTransformTaskKinematics
//! \sa HTransformAbsTaskKinematics
//! \sa HTransformRelTaskKinematics
//  ----------------------------------------------------------
template <typename SubsysType, typename HTransformKinematicsType>
class HTransformKinematics : public Kinematics<SubsysType, 6, HTransformKinematics<SubsysType, HTransformKinematicsType>, LieGroup::HTransform, LieGroup::Twist, LieGroup::Twist, LieGroup::Wrench>
{
public:
	typedef typename Kinematics<SubsysType, 6, HTransformKinematics<SubsysType, HTransformKinematicsType>, LieGroup::HTransform, LieGroup::Twist>::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename Kinematics<SubsysType, 6, HTransformKinematics<SubsysType, HTransformKinematicsType>, LieGroup::HTransform, LieGroup::Twist>::TaskJac TaskJac; //!< Typedef of task Jacobian

	//! \returns Reference to the derived object 
	inline HTransformKinematicsType& derived() { return *static_cast<HTransformKinematicsType*>(this); }
	//! \returns Constant reference to the derived object 
	inline const HTransformKinematicsType& derived() const { return *static_cast<const HTransformKinematicsType*>(this); }

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the homogeneous transform task kinematics
	//!
	//! \details
	//! The task transformation \f$ T \f$ and the task twist \f$ V \f$ should be computed using the system's state. 
	//! It should be passed to the arguments T and V, respectively.
	//! 
	//! \param subsys System object of SubsysType
	//! \param T Task transformation or \f$ T \f$
	//! \param V Task twist or \f$ V \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::HTransform & T, LieGroup::Twist & V) const
	{
		derived().kinematics(subsys, T, V);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the homogeneous transform task kinematics
	//!
	//! \details
	//! The task transformation \f$ T \f$ should be computed using the system's state. 
	//! It should be passed to the arguments T.
	//! 
	//! \param subsys System object of SubsysType
	//! \param T Task transformation or \f$ T \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::HTransform & T) const
	{
		derived().kinematics(subsys, T);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the homogeneous transform task kinematics with the task Jacobian and its derivative
	//!
	//! \details
	//! The task transformation \f$ T \f$ and the task twist \f$ V \f$ should be computed using the system's state. 
	//! It should be passed to the arguments T and V, respectively. 
	//! Also, the task Jacobian \f$ J \f$ and its derivative \f$ \dot{J} \f$ should be computed and passed to the arguments J and Jdot, respectively.
	//! 
	//! \param subsys System object of SubsysType
	//! \param T Task transformation or \f$ T \f$
	//! \param V Task twist or \f$ V \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//! \param Jdot Task Jacobian derivative matrix or \f$ \dot{J} \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::HTransform & T, LieGroup::Twist & V, TaskJac & J, TaskJac & Jdot) const
	{
		derived().jacobian(subsys, T, V, J, Jdot);
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the task transformation with the task Jacobian
	//!
	//! \details
	//! The task transformation \f$ T \f$ should be computed using the system's state and passed to the arguments T. 
	//! Also, the task Jacobian \f$ J \f$ should be computed and passed to the arguments J.
	//! 
	//! \param subsys System object of SubsysType
	//! \param T Task transformation or \f$ T \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::HTransform & T, TaskJac & J) const
	{
		derived().jacobian(subsys, T, J);
	}
};

//  ---------------------- Doxygen info ----------------------
//! \class HTransformAbsTaskKinematics
//!
//! \brief
//! This implements an absolute homogeneous transform task kinematics class
//!
//! \details 
//!	Absolute homogeneous transform tasks are described in terms of the task transformation, \f$ {^{ref}T_{tar}} \f$,
//! and the task twist \f$ {^{ref}V_{tar}} \f$, of the target body of index \f$ \beta \f$ (belonging to the subsys), 
//! with respect to the (fixed) ground. In particular, the target coordinate frame \f$ \{ tar \} \f$ can have
//! an offset from the targte body frame \f$ \{ \beta \} \f$ or \f$ {^{\beta}T_{tar}} \f$, and the reference coordinate 
//! frame  \f$ \{ ref \} \f$ has an offset from the fixed reference frame \f$ \{ -1 \} \f$ or \f$ {^{-1}T_{ref}} \f$. 
//! 
//! The task Jacobian and its derivative is computed in terms of the body Jacobians of the system. 
//! 
//! \note 
//! The system should be updated by update() before calling kinematics(). Furthermore, 
//! it should have updated body Jacobians by updateJacobian() before calling jacobian().
//! 
//! \tparam SubsysType Type of the subsys
//! 
//! \sa HTransformKinematics
//! \sa HTransformRelTaskKinematics
//  ----------------------------------------------------------
template<typename SubsysType>
class HTransformAbsTaskKinematics
: public HTransformKinematics<SubsysType, HTransformAbsTaskKinematics<SubsysType> >
{
public:
	typedef typename HTransformKinematics<SubsysType, HTransformAbsTaskKinematics<SubsysType> >::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename HTransformKinematics<SubsysType, HTransformAbsTaskKinematics<SubsysType> >::TaskJac TaskJac; //!< Typedef of task Jacobian matrix

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the absolute homogeneous transform task kinematics
	//!
	//! \param subsys System object
	//! \param target Body index of the target body or \f$ \beta \f$
	//! \param Ttarget Offset homogeneous transformation matrix of the target frame relative to the target body frame, i.e. \f$ {^{\beta}T_{tar}} \f$
	//! \param Tref Offset homogeneous transformation matrix of the reference frame relative to the fixed ground frame, i.e. \f$ {^{-1}T_{ref}} \f$
	//  ----------------------------------------------------------
	inline HTransformAbsTaskKinematics(SubsysType const & subsys, 
		int target, LieGroup::HTransform const & Ttarget, LieGroup::HTransform const & Tref)
		: HTransformKinematics<SubsysType, HTransformAbsTaskKinematics<SubsysType> >()
		, _target(target), _Ttarget(Ttarget), _Tref(Tref)
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the absolute homogeneous transform task kinematics
	//!
	//! \note 
	//!	set() should be called after construction
	//!
	//! \param subsys System object
	//  ----------------------------------------------------------
	inline HTransformAbsTaskKinematics(SubsysType const & subsys)
		: HTransformKinematics<SubsysType, HTransformAbsTaskKinematics<SubsysType> >()
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the data for absolute homogeneous transform task kinematics
	//!
	//! \param target Body index of the target body or \f$ \beta \f$
	//! \param Ttarget Offset homogeneous transformation matrix of the target frame relative to the target body frame, i.e. \f$ {^{\beta}T_{tar}} \f$
	//! \param Tref Offset homogeneous transformation matrix of the reference frame relative to the fixed ground frame, i.e. \f$ {^{-1}T_{ref}} \f$
	//  ----------------------------------------------------------
	inline void set(int target, LieGroup::HTransform const & Ttarget, LieGroup::HTransform const & Tref)
	{
		_target = target;
		_Ttarget = Ttarget;
		_Tref = Tref;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the absolute homogeneous transform task kinematics
	//!
	//! \param subsys System object of SubsysType
	//! \param T Task transformation or \f$ {^{ref}T_{tar}} \f$
	//! \param V Task twist or \f$ {^{ref}V_{tar}} \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::HTransform & T, LieGroup::Twist & V) const
	{
		Body const & beta = subsys.body(_target);

		T = _Tref.icascade(beta.T().cascade(_Ttarget));
		V = _Ttarget.itransform(beta.V()); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the absolute homogeneous transform task kinematics
	//!
	//! \param subsys System object of SubsysType
	//! \param T Task transformation or \f$ {^{ref}T_{tar}} \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::HTransform & T) const
	{
		Body const & beta = subsys.body(_target);

		T = _Tref.icascade(beta.T().cascade(_Ttarget));
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the absolute homogeneous transform task kinematics with the task Jacobian and its derivative
	//!
	//! \param subsys System object of SubsysType
	//! \param T Task transformation or \f$ {^{ref}T_{tar}} \f$
	//! \param V Task twist or \f$ {^{ref}V_{tar}} \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//! \param Jdot Task Jacobian derivative matrix or \f$ \dot{J} \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::HTransform & T, LieGroup::Twist & V, TaskJac & J, TaskJac & Jdot) const
	{
		Body const & beta = subsys.body(_target);

		T = _Tref.icascade(beta.T().cascade(_Ttarget));
		V = _Ttarget.itransform(beta.V()); 

		_Ttarget.itransform(subsys.J(_target), J); 
		_Ttarget.itransform(subsys.Jdot(_target), Jdot); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the absolute task transformation with the task Jacobian
	//!
	//! \param subsys System object of SubsysType
	//! \param T Task transformation or \f$ {^{ref}T_{tar}} \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::HTransform & T, TaskJac & J) const
	{
		Body const & beta = subsys.body(_target);

		T = _Tref.icascade(beta.T().cascade(_Ttarget));
		
		_Ttarget.itransform(subsys.J(_target), J); //_J[target];
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Body index of the target body or \f$ \beta \f$
	//  ----------------------------------------------------------
	int _target; 

	//  ---------------------- Doxygen info ----------------------
	//! \brief Offset homogeneous transformation matrix of the target frame relative to the target body frame, i.e. \f$ {^{\beta}T_{tar}} \f$
	//  ----------------------------------------------------------
	LieGroup::HTransform _Ttarget;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Offset homogeneous transformation matrix of the reference frame relative to the fixed ground frame, i.e. \f$ {^{-1}T_{ref}} \f$
	//  ----------------------------------------------------------
	LieGroup::HTransform _Tref;
};

//  ---------------------- Doxygen info ----------------------
//! \class HTransformRelTaskKinematics
//!
//! \brief
//! This implements an relative homogeneous transform task kinematics class
//!
//! \details 
//!	Relative homogeneous transform tasks are described in terms of the task transformation, \f$ {^{ref}T_{tar}} \f$,
//! and the task twist \f$ {^{ref}V_{tar}} \f$, of the target body of index \f$ \beta \f$ (belonging to the subsys), 
//! with respect to the reference body of index \f$ \alpha \f$ (belonging to the subsys). 
//! In particular, the target coordinate frame \f$ \{ tar \} \f$ can have
//! an offset from the targte body frame \f$ \{ \beta \} \f$ or \f$ {^{\beta}T_{tar}} \f$, and the reference coordinate 
//! frame  \f$ \{ ref \} \f$ has an offset from the reference body frame \f$ \{ \alpha \} \f$ or \f$ {^{\alpha}T_{ref}} \f$. 
//! 
//! The task Jacobian and its derivative is computed in terms of the body Jacobians of the system. 
//! 
//! \note 
//! The system should be updated by update() before calling kinematics(). Furthermore, 
//! it should have updated body Jacobians by updateJacobian() before calling jacobian().
//! 
//! \tparam SubsysType Type of the subsys
//! 
//! \sa HTransformKinematics
//! \sa HTransformAbsTaskKinematics
//  ----------------------------------------------------------
template<typename SubsysType>
class HTransformRelTaskKinematics
: public HTransformKinematics<SubsysType, HTransformRelTaskKinematics<SubsysType> >
{
public:
	typedef typename HTransformKinematics<SubsysType, HTransformRelTaskKinematics<SubsysType> >::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename HTransformKinematics<SubsysType, HTransformRelTaskKinematics<SubsysType> >::TaskJac TaskJac; //!< Typedef of task Jacobian matrix

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the relative homogeneous transform task kinematics
	//!
	//! \param subsys System object
	//! \param target Body index of the target body or \f$ \beta \f$
	//! \param Ttarget Offset homogeneous transformation matrix of the target frame relative to the target body frame, i.e. \f$ {^{\beta}T_{tar}} \f$
	//! \param ref Body index of the reference body or \f$ \alpha \f$
	//! \param Tref Offset homogeneous transformation matrix of the reference frame relative to the reference body frame, i.e. \f$ {^{\alpha}T_{ref}} \f$
	//  ----------------------------------------------------------
	inline HTransformRelTaskKinematics(SubsysType const & subsys, 
		int target, LieGroup::HTransform const & Ttarget, int ref, LieGroup::HTransform const & Tref)
		: HTransformKinematics<SubsysType, HTransformRelTaskKinematics<SubsysType> >()
		, _target(target), _Ttarget(Ttarget), _ref(ref), _Tref(Tref)
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the relative homogeneous transform task kinematics
	//!
	//! \note 
	//!	set() should be called after construction
	//!
	//! \param subsys System object
	//  ----------------------------------------------------------
	inline HTransformRelTaskKinematics(SubsysType const & subsys)
		: HTransformKinematics<SubsysType, HTransformRelTaskKinematics<SubsysType> >()
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the data for absolute homogeneous transform task kinematics
	//!
	//! \param target Body index of the target body or \f$ \beta \f$
	//! \param Ttarget Offset homogeneous transformation matrix of the target frame relative to the target body frame, i.e. \f$ {^{\beta}T_{tar}} \f$
	//! \param Tref Offset homogeneous transformation matrix of the reference frame relative to the fixed ground frame, i.e. \f$ {^{-1}T_{ref}} \f$
	//  ----------------------------------------------------------
	inline void set(int target, LieGroup::HTransform const & Ttarget, int ref, LieGroup::HTransform const & Tref)
	{
		_target = target;
		_Ttarget = Ttarget;
		_ref = ref;
		_Tref = Tref;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the relative homogeneous transform task kinematics
	//!
	//! \param subsys System object of SubsysType
	//! \param T Task transformation or \f$ {^{ref}T_{tar}} \f$
	//! \param V Task twist or \f$ {^{ref}V_{tar}} \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::HTransform & T, LieGroup::Twist & V) const
	{
		Body const & alpha = subsys.body(_ref);
		Body const & beta = subsys.body(_target);

		LieGroup::HTransform T_alpha_beta = alpha.T().icascade(beta.T());
		T = _Tref.icascade(T_alpha_beta.cascade(_Ttarget));

		LieGroup::Twist V_alpha_beta = beta.V() - T_alpha_beta.itransform(alpha.V()); 
		V = _Ttarget.itransform(V_alpha_beta); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the relative homogeneous transform task kinematics
	//!
	//! \param subsys System object of SubsysType
	//! \param T Task transformation or \f$ {^{ref}T_{tar}} \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::HTransform & T) const
	{
		Body const & alpha = subsys.body(_ref);
		Body const & beta = subsys.body(_target);

		LieGroup::HTransform T_alpha_beta = alpha.T().icascade(beta.T());
		T = _Tref.icascade(T_alpha_beta.cascade(_Ttarget));
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the relative homogeneous transform task kinematics with the task Jacobian and its derivative
	//!
	//! \param subsys System object of SubsysType
	//! \param T Task transformation or \f$ {^{ref}T_{tar}} \f$
	//! \param V Task twist or \f$ {^{ref}V_{tar}} \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//! \param Jdot Task Jacobian derivative matrix or \f$ \dot{J} \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::HTransform & T, LieGroup::Twist & V, TaskJac & J, TaskJac & Jdot) const
	{
		Body const & alpha = subsys.body(_ref);
		Body const & beta = subsys.body(_target);

		LieGroup::HTransform T_alpha_beta = alpha.T().icascade(beta.T());
		T = _Tref.icascade(T_alpha_beta.cascade(_Ttarget));

		LieGroup::Twist V_alpha_beta = beta.V() - T_alpha_beta.itransform(alpha.V()); 
		V = _Ttarget.itransform(V_alpha_beta); 

		TaskJac Temp; 
		TaskJac dTemp;

		T_alpha_beta.itransform(subsys.J(_ref), Temp); 
		T_alpha_beta.itransform(subsys.Jdot(_ref), dTemp);
		TaskJac dTemp2;
		V_alpha_beta.adjoint(Temp, dTemp2);
		dTemp -= dTemp2;

		_Ttarget.itransform(subsys.J(_target) - Temp, J);
		_Ttarget.itransform(subsys.Jdot(_target) - dTemp, Jdot);

		// FIXME Do we need these explicitly?
		//J.leftCols<EARTH_DOF>().setZero(); 
		//Jdot.leftCols<EARTH_DOF>().setZero(); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the relative task transformation with the task Jacobian
	//!
	//! \param subsys System object of SubsysType
	//! \param T Task transformation or \f$ {^{ref}T_{tar}} \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::HTransform & T, TaskJac & J) const
	{
		Body const & alpha = subsys.body(_ref);
		Body const & beta = subsys.body(_target);

		LieGroup::HTransform T_alpha_beta = alpha.T().icascade(beta.T());
		T = _Tref.icascade(T_alpha_beta.cascade(_Ttarget));

		TaskJac Temp;

		T_alpha_beta.itransform(subsys.J(_ref), Temp); //_J[ref];
		_Ttarget.itransform(subsys.J(_target) - Temp, J);

		// FIXME Do we need these explicitly?
		//J.leftCols<EARTH_DOF>().setZero(); 
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Body index of the target body or \f$ \beta \f$
	//  ----------------------------------------------------------
	int _target; 

	//  ---------------------- Doxygen info ----------------------
	//! \brief Offset homogeneous transformation matrix of the target frame relative to the target body frame, i.e. \f$ {^{\beta}T_{tar}} \f$
	//  ----------------------------------------------------------
	LieGroup::HTransform _Ttarget;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Body index of the reference body or \f$ \alpha \f$
	//  ----------------------------------------------------------
	int _ref;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Offset homogeneous transformation matrix of the reference frame relative to the reference body frame, i.e. \f$ {^{\alpha}T_{ref}} \f$
	//  ----------------------------------------------------------
	LieGroup::HTransform _Tref;
};

} // namespace NRMKFoundation
