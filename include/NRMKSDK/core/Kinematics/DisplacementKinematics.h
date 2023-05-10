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

#include "Kinematics.h"
#include "../AMBS/Subsys.h"

namespace NRMKFoundation
{
namespace internal
{
}

//  ---------------------- Doxygen info ----------------------
//! \class DisplacementKinematics
//!
//! \brief
//! This implements a displacement kinematics class
//!
//! \details 
//!	Displacement tasks are described in terms of a displacement vector, \f$ r \f$,
//! and its derivative vector \f$ \dot{r} \f$. 
//! They are called the task displacement and task velocity, respectively.
//! The kinematic relationship with joint position and velocity vectors \f$ q \f$ and \f$ \dot{q} \f$ 
//! are defined by \f$ r = r(q) \f$ and \f$ \dot{r} = J(q) \dot{q} \f$.
//! User should provide the algorithm to compute \f$ r \f$, \f$ \dot{r} \f$, \f$ J \f$, and \f$ \dot{J} \f$ 
//! to implement a concrete rotation task.
//! 
//! \note
//! Users should inherit this class to define a customized kinematics.
//! See DisplacementAbsTaskKinematics and DisplacementRelTaskKinematics.
//! 
//! \tparam SubsysType Type of the subsys
//! \tparam DisplacementKinematicsType Derived displacement kinematics class 
//! 
//! \sa DisplacementAbsTaskKinematics
//! \sa DisplacementRelTaskKinematics
//  ----------------------------------------------------------
template <typename SubsysType, typename DisplacementKinematicsType>
class DisplacementKinematics : public Kinematics<SubsysType, 3, DisplacementKinematics<SubsysType, DisplacementKinematicsType>, LieGroup::Displacement, LieGroup::Vector3D>
{
public:
	typedef typename Kinematics<SubsysType, 3, DisplacementKinematics<SubsysType, DisplacementKinematicsType>, LieGroup::Displacement, LieGroup::Vector3D>::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename Kinematics<SubsysType, 3, DisplacementKinematics<SubsysType, DisplacementKinematicsType>, LieGroup::Displacement, LieGroup::Vector3D>::TaskJac TaskJac; //!< Typedef of task Jacobian

	//! \returns Reference to the derived object 
	inline DisplacementKinematicsType& derived() { return *static_cast<DisplacementKinematicsType*>(this); }
	//! \returns Constant reference to the derived object 
	inline const DisplacementKinematicsType& derived() const { return *static_cast<const DisplacementKinematicsType*>(this); }

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the displacement task kinematics
	//!
	//! \details
	//! The task displacement \f$ r \f$ and the task velocity \f$ \dot{r} \f$ should be computed using the system's state. 
	//! It should be passed to the arguments r and rdot, respectively.
	//! 
	//! \param subsys System object of SubsysType
	//! \param r Task displacement or \f$ r \f$
	//! \param rdot Task velocity or \f$ \dot{r} \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::Displacement & r, LieGroup::Vector3D & rdot) const
	{
		derived().kinematics(subsys, r, rdot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the displacement task kinematics
	//!
	//! \details
	//! The task displacement \f$ r \f$ should be computed using the system's state. 
	//! It should be passed to the arguments r.
	//! 
	//! \param subsys System object of SubsysType
	//! \param r Task displacement or \f$ r \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::Displacement & r) const
	{
		derived().kinematics(subsys, r);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the displacement task kinematics with the task Jacobian and its derivative
	//!
	//! \details
	//! The task displacement \f$ r \f$ and the task velocity \f$ \dot{r} \f$ should be computed using the system's state. 
	//! It should be passed to the arguments r and rdot, respectively.
	//! Also, the task Jacobian \f$ J \f$ and its derivative \f$ \dot{J} \f$ should be computed and passed to the arguments J and Jdot, respectively.
	//! 
	//! \param subsys System object of SubsysType
	//! \param r Task displacement or \f$ r \f$
	//! \param rdot Task velocity or \f$ \dot{r} \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//! \param Jdot Task Jacobian derivative matrix or \f$ \dot{J} \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::Displacement & r, LieGroup::Vector3D & rdot, TaskJac & J, TaskJac & Jdot) const
	{
		derived().jacobian(subsys, r, rdot, J, Jdot);
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the task displacement with the task Jacobian
	//!
	//! \details
	//! The task displacement \f$ r \f$ should be computed using the system's state and passed to the arguments r. 
	//! Also, the task Jacobian \f$ J \f$ should be computed and passed to the arguments J.
	//! 
	//! \param subsys System object of SubsysType
	//! \param r Task displacement or \f$ r \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::Displacement & r, TaskJac & J) const
	{
		derived().jacobian(subsys, r, J);
	}
};

//  ---------------------- Doxygen info ----------------------
//! \class DisplacementAbsTaskKinematics
//!
//! \brief
//! This implements an absolute displacement task kinematics class
//!
//! \details 
//!	Absolute displacement tasks are described in terms of the task displacement, \f$ {^{ref}r_{tar}} \f$,
//! and the task velocity vector \f$ {^{ref} \dot{r}_{tar}} \f$, of the target body of index \f$ \beta \f$ (belonging to the subsys), 
//! with respect to the (fixed) ground. In particular, the target point can have
//! an offset displacement from the targte body frame \f$ \{ \beta \} \f$ or \f$ {^{\beta}r_{tar}} \f$, and the reference point 
//! \f$ \{ ref \} \f$ has an offset displacement from the fixed reference frame \f$ \{ -1 \} \f$ or \f$ {^{-1}r_{ref}} \f$. 
//! 
//! The task Jacobian and its derivative is computed in terms of the body Jacobians of the system. 
//! 
//! \note 
//! The system should be updated by update() before calling kinematics(). Furthermore, 
//! it should have updated body Jacobians by updateJacobian() before calling jacobian().
//! 
//! \tparam SubsysType Type of the subsys
//!
//! \sa DisplacementKinematics
//! \sa DisplacementRelTaskKinematics
//  ----------------------------------------------------------
template<typename SubsysType>
class DisplacementAbsTaskKinematics : public DisplacementKinematics<SubsysType, DisplacementAbsTaskKinematics<SubsysType> >
{
public:
	typedef typename DisplacementKinematics<SubsysType, DisplacementAbsTaskKinematics<SubsysType> >::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename DisplacementKinematics<SubsysType, DisplacementAbsTaskKinematics<SubsysType> >::TaskJac TaskJac; //!< Typedef of task Jacobian matrix

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the absolute displacement task kinematics
	//!
	//! \param subsys System object
	//! \param target Body index of the target body or \f$ \beta \f$
	//! \param rtarget Offset displacement of the target point relative to the target body frame, i.e. \f$ {^{\beta}r_{tar}} \f$
	//! \param rref Offset displacement of the reference point relative to the fixed ground frame, i.e. \f$ {^{-1}r_{ref}} \f$
	//  ----------------------------------------------------------
	inline DisplacementAbsTaskKinematics(SubsysType const & subsys, 
		int target, LieGroup::Displacement const & rtarget, LieGroup::Displacement const & rref)
		: DisplacementKinematics<SubsysType, DisplacementAbsTaskKinematics<SubsysType> >()
		, _target(target), _rtarget(rtarget)//, _rref(rref)
	{
		_Tref.r() = rref;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the absolute displacement task kinematics
	//!
	//! \param subsys System object
	//! \param target Body index of the target body or \f$ \beta \f$
	//! \param rtarget Offset displacement of the target point relative to the target body frame, i.e. \f$ {^{\beta}r_{tar}} \f$
	//! \param Tref Offset transformation of the reference frame relative to the fixed ground frame, i.e. \f$ {^{-1}T_{ref}} \f$
	//  ----------------------------------------------------------
	inline DisplacementAbsTaskKinematics(SubsysType const & subsys, 
		int target, LieGroup::Displacement const & rtarget, LieGroup::HTransform const & Tref)
		: DisplacementKinematics<SubsysType, DisplacementAbsTaskKinematics<SubsysType> >()
		, _target(target), _rtarget(rtarget), _Tref(Tref)
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the absolute displacement task kinematics
	//!
	//! \note 
	//!	set() should be called after construction
	//!
	//! \param subsys System object
	//  ----------------------------------------------------------
	inline DisplacementAbsTaskKinematics(SubsysType const & subsys)
		: DisplacementKinematics<SubsysType, DisplacementAbsTaskKinematics<SubsysType> >()
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the data for absolute displacement task kinematics
	//!
	//! \param target Body index of the target body or \f$ \beta \f$
	//! \param rtarget Offset displacement of the target point relative to the target body frame, i.e. \f$ {^{\beta}r_{tar}} \f$
	//! \param rref Offset displacement of the reference point relative to the fixed ground frame, i.e. \f$ {^{-1}r_{ref}} \f$
	//  ----------------------------------------------------------
	inline void set(int target, LieGroup::Displacement const & rtarget, LieGroup::Displacement const & rref)
	{
		_target = target;
		_rtarget = rtarget;
		_Tref.r() = rref;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the data for absolute displacement task kinematics
	//!
	//! \param target Body index of the target body or \f$ \beta \f$
	//! \param rtarget Offset displacement of the target point relative to the target body frame, i.e. \f$ {^{\beta}r_{tar}} \f$
	//! \param Tref Offset transformation of the reference frame relative to the fixed ground frame, i.e. \f$ {^{-1}T_{ref}} \f$
	//  ----------------------------------------------------------
	inline void set(int target, LieGroup::Displacement const & rtarget, LieGroup::HTransform const & Tref)
	{
		_target = target;
		_rtarget = rtarget;
		_Tref = Tref;
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the absolute displacement task kinematics
	//!
	//! \param subsys System object of SubsysType
	//! \param r Task displacement or \f$ {^{ref}r_{tar}} \f$
	//! \param rdot Task velocity or \f$ {^{ref}\dot{r}_{tar}} \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::Displacement & r, LieGroup::Vector3D & rdot) const
	{
		Body const & beta = subsys.body(_target);

		//r = _Tref.R().transpose()*(beta.T().transform(_rtarget) - _rref); 
		r = _Tref.R().transpose()*(beta.T().transform(_rtarget) - _Tref.r()); 
		rdot = _Tref.R().transpose()*beta.T().R()*(beta.V().v() - _rtarget.cross(beta.V().w()));
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the absolute displacement task kinematics
	//!
	//! \param subsys System object of SubsysType
	//! \param r Task displacement or \f$ {^{ref}r_{tar}} \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::Displacement & r) const
	{
		Body const & beta = subsys.body(_target);

		//r = _Tref.R().transpose()*(beta.T().transform(_rtarget) - _rref); 
		r = _Tref.R().transpose()*(beta.T().transform(_rtarget) - _Tref.r()); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the absolute displacement task kinematics with the task Jacobian and its derivative
	//!
	//! \param subsys System object of SubsysType
	//! \param r Task displacement or \f$ {^{ref}r_{tar}} \f$
	//! \param rdot Task velocity or \f$ {^{ref}\dot{r}_{tar}} \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//! \param Jdot Task Jacobian derivative matrix or \f$ \dot{J} \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::Displacement & r, LieGroup::Vector3D & rdot, TaskJac & J, TaskJac & Jdot) const
	{
		Body const & beta = subsys.body(_target);

		//r = _Tref.R().transpose()*(beta.T().transform(_rtarget) - _rref); 
		r = _Tref.R().transpose()*(beta.T().transform(_rtarget) - _Tref.r()); 

		LieGroup::Rotation R_ref_beta = _Tref.R().transpose()*beta.T().R();
		rdot = R_ref_beta*(beta.V().v() - _rtarget.cross(beta.V().w()));

		TaskJac Temp;
		TaskJac dTemp;

#ifdef __GNUC__
		LieGroup::internal::_cross(_rtarget[0], _rtarget[1], _rtarget[2], subsys.J(_target).template bottomRows<3>(), Temp);
		Temp = subsys.J(_target).template topRows<3>() - Temp;
#else
		LieGroup::internal::_cross(_rtarget[0], _rtarget[1], _rtarget[2], subsys.J(_target).bottomRows<3>(), Temp);
		Temp = subsys.J(_target).topRows<3>() - Temp;
#endif

		// Temp is used to compute J ...
		J.noalias() = R_ref_beta*Temp;

		LieGroup::internal::_cross(beta.V().w()[0], beta.V().w()[1], beta.V().w()[2], Temp, dTemp);

		// ... hence now one can reuse Temp
#ifdef __GNUC__
		LieGroup::internal::_cross(_rtarget[0], _rtarget[1], _rtarget[2], subsys.Jdot(_target).template bottomRows<3>(), Temp);
		Temp = subsys.Jdot(_target).template topRows<3>() - Temp;
#else
		LieGroup::internal::_cross(_rtarget[0], _rtarget[1], _rtarget[2], subsys.Jdot(_target).bottomRows<3>(), Temp);
		Temp = subsys.Jdot(_target).topRows<3>() - Temp;
#endif
		Jdot.noalias() = R_ref_beta*(Temp + dTemp); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the absolute task displacement with the task Jacobian
	//!
	//! \param subsys System object of SubsysType
	//! \param r Task displacement or \f$ {^{ref}r_{tar}} \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::Displacement & r, TaskJac & J) const
	{
		Body const & beta = subsys.body(_target);

		//r = beta.T().transform(_rtarget) - _rref; 
		r = beta.T().transform(_rtarget) - _Tref.r(); 
		
		TaskJac Temp;		
#ifdef __GNUC__
		LieGroup::internal::_cross(_rtarget[0], _rtarget[1], _rtarget[2], subsys.J(_target).template bottomRows<3>(), Temp);		
		J.noalias() = _Tref.R().transpose()*beta.T().R()*(subsys.J(_target).template topRows<3>() - Temp); //_J[target];
#else
		LieGroup::internal::_cross(_rtarget[0], _rtarget[1], _rtarget[2], subsys.J(_target).bottomRows<3>(), Temp);		
		J.noalias() = _Tref.R().transpose()*beta.T().R()*(subsys.J(_target).topRows<3>() - Temp); //_J[target];
#endif
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Body index of the target body or \f$ \beta \f$
	//  ----------------------------------------------------------
	int _target; 

	//  ---------------------- Doxygen info ----------------------
	//! \brief Offset displacement of the target point relative to the target body frame, i.e. \f$ {^{\beta}r_{tar}} \f$
	//  ----------------------------------------------------------
	LieGroup::Displacement _rtarget;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Offset transformation of the reference frame relative to the fixed ground frame, i.e. \f$ {^{-1}T_{ref}} \f$
	//  ----------------------------------------------------------
	LieGroup::HTransform _Tref;
};

//  ---------------------- Doxygen info ----------------------
//! \class DisplacementRelTaskKinematics
//!
//! \brief
//! This implements an relative displacement task kinematics class
//!
//! \details 
//!	Relative displacement tasks are described in terms of the task displacement, \f$ {^{ref}r_{tar}} \f$,
//! and the task velocity vector \f$ {^{ref} \dot{r}_{tar}} \f$, of the target body of index \f$ \beta \f$ (belonging to the subsys), 
//! with respect to the reference body of index \f$ \alpha \f$ (belonging to the subsys). 
//! In particular, the target point can have
//! an offset displacement from the targte body frame \f$ \{ \beta \} \f$ or \f$ {^{\beta}r_{tar}} \f$, and the reference point 
//! \f$ \{ ref \} \f$ has an offset displacement from the reference body frame \f$ \{ \alpha \} \f$ or \f$ {^{\alpha}r_{ref}} \f$. 
//! 
//! The task Jacobian and its derivative is computed in terms of the body Jacobians of the system. 
//! 
//! \note 
//! The system should be updated by update() before calling kinematics(). Furthermore, 
//! it should have updated body Jacobians by updateJacobian() before calling jacobian().
//! 
//! \tparam SubsysType Type of the subsys
//! 
//! \sa DisplacementKinematics
//! \sa DisplacementAbsTaskKinematics
//  ----------------------------------------------------------
template<typename SubsysType>
class DisplacementRelTaskKinematics : public DisplacementKinematics<SubsysType, DisplacementRelTaskKinematics<SubsysType> >
{
public:
	typedef typename DisplacementKinematics<SubsysType, DisplacementRelTaskKinematics<SubsysType> >::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename DisplacementKinematics<SubsysType, DisplacementRelTaskKinematics<SubsysType> >::TaskJac TaskJac; //!< Typedef of task Jacobian matrix

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the relative displacement task kinematics
	//!
	//! \param subsys System object
	//! \param target Body index of the target body or \f$ \beta \f$
	//! \param rtarget Offset displacement of the target point relative to the target body frame, i.e. \f$ {^{\beta}r_{tar}} \f$
	//! \param ref Body index of the reference body or \f$ \alpha \f$
	//! \param rref Offset displacement of the reference point relative to the fixed ground frame, i.e. \f$ {^{-1}r_{ref}} \f$
	//  ----------------------------------------------------------
	inline DisplacementRelTaskKinematics(SubsysType const & subsys, 
		int target, LieGroup::Displacement const & rtarget, int ref, LieGroup::Displacement const & rref)
		: DisplacementKinematics<SubsysType, DisplacementRelTaskKinematics<SubsysType> >()
		, _target(target), _rtarget(rtarget), _ref(ref)/*, _rref(rref)*/
	{
		_Tref.r() = rref;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the relative displacement task kinematics
	//!
	//! \param subsys System object
	//! \param target Body index of the target body or \f$ \beta \f$
	//! \param rtarget Offset displacement of the target point relative to the target body frame, i.e. \f$ {^{\beta}r_{tar}} \f$
	//! \param ref Body index of the reference body or \f$ \alpha \f$
	//! \param Tref Offset transformation of the reference frame relative to the fixed ground frame, i.e. \f$ {^{-1}T_{ref}} \f$
	//  ----------------------------------------------------------
	inline DisplacementRelTaskKinematics(SubsysType const & subsys, 
		int target, LieGroup::Displacement const & rtarget, int ref, LieGroup::HTransform const & Tref)
		: DisplacementKinematics<SubsysType, DisplacementRelTaskKinematics<SubsysType> >()
		, _target(target), _rtarget(rtarget), _ref(ref), _Tref(Tref)
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the relative displacement task kinematics
	//!
	//! \note 
	//!	set() should be called after construction
	//!
	//! \param subsys System object
	//  ----------------------------------------------------------
	inline DisplacementRelTaskKinematics(SubsysType const & subsys)
		: DisplacementKinematics<SubsysType, DisplacementRelTaskKinematics<SubsysType> >()
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the data for relative displacement task kinematics
	//!
	//! \param target Body index of the target body or \f$ \beta \f$
	//! \param rtarget Offset displacement of the target point relative to the target body frame, i.e. \f$ {^{\beta}r_{tar}} \f$
	//! \param ref Body index of the reference body or \f$ \alpha \f$
	//! \param rref Offset displacement of the reference point relative to the fixed ground frame, i.e. \f$ {^{-1}r_{ref}} \f$
	//  ----------------------------------------------------------
	inline void set(int target, LieGroup::Displacement const & rtarget, int ref, LieGroup::Displacement const & rref)
	{
		_target = target;
		_rtarget = rtarget;
		_ref = ref;
		_Tref.r() = rref;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the data for relative displacement task kinematics
	//!
	//! \param target Body index of the target body or \f$ \beta \f$
	//! \param rtarget Offset displacement of the target point relative to the target body frame, i.e. \f$ {^{\beta}r_{tar}} \f$
	//! \param ref Body index of the reference body or \f$ \alpha \f$
	//! \param Tref Offset transformation of the reference frame relative to the fixed ground frame, i.e. \f$ {^{-1}T_{ref}} \f$
	//  ----------------------------------------------------------
	inline void set(int target, LieGroup::Displacement const & rtarget, int ref, LieGroup::HTransform const & Tref)
	{
		_target = target;
		_rtarget = rtarget;
		_ref = ref;
		_Tref = Tref;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the relative displacement task kinematics
	//!
	//! \param subsys System object of SubsysType
	//! \param r Task displacement or \f$ {^{ref}r_{tar}} \f$
	//! \param rdot Task velocity or \f$ {^{ref}\dot{r}_{tar}} \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::Displacement & r, LieGroup::Vector3D & rdot) const
	{
		Body const & alpha = subsys.body(_ref);
		Body const & beta = subsys.body(_target);

		//r = _Tref.R().transpose()*alpha.T().R().transpose()*(beta.T().transform(_rtarget) - alpha.T().transform(_rref));
		r = _Tref.R().transpose()*alpha.T().R().transpose()*(beta.T().transform(_rtarget) - alpha.T().transform(LieGroup::Displacement(_Tref.r())));

		Eigen::Matrix3d R_alpha_beta = alpha.T().R().transpose()*beta.T().R();
		//Eigen::Vector3d r_alpha_target = _rref + r;
		Eigen::Vector3d r_alpha_target = _Tref.r() + r;

		rdot = _Tref.R().transpose()*(R_alpha_beta*(beta.V().v() - _rtarget.cross(beta.V().w()))
					- (alpha.V().v() - r_alpha_target.cross(alpha.V().w())));
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the relative displacement task kinematics
	//!
	//! \param subsys System object of SubsysType
	//! \param r Task displacement or \f$ {^{ref}r_{tar}} \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::Displacement & r) const
	{
		Body const & alpha = subsys.body(_ref);
		Body const & beta = subsys.body(_target);

		//r = _Tref.R().transpose()*alpha.T().R().transpose()*(beta.T().transform(_rtarget) - alpha.T().transform(_rref));
		r = _Tref.R().transpose()*alpha.T().R().transpose()*(beta.T().transform(_rtarget) - alpha.T().transform(LieGroup::Displacement(_Tref.r())));
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the relative displacement task kinematics with the task Jacobian and its derivative
	//!
	//! \param subsys System object of SubsysType
	//! \param r Task displacement or \f$ {^{ref}r_{tar}} \f$
	//! \param rdot Task velocity or \f$ {^{ref}\dot{r}_{tar}} \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//! \param Jdot Task Jacobian derivative matrix or \f$ \dot{J} \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::Displacement & r, LieGroup::Vector3D & rdot, TaskJac & J, TaskJac & Jdot) const
	{
		Body const & alpha = subsys.body(_ref);
		Body const & beta = subsys.body(_target);

		//r = _Tref.R().transpose()*alpha.T().R().transpose()*(beta.T().transform(_rtarget) - alpha.T().transform(_rref));
		r = _Tref.R().transpose()*alpha.T().R().transpose()*(beta.T().transform(_rtarget) - alpha.T().transform(LieGroup::Displacement(_Tref.r())));

		Eigen::Matrix3d R_alpha_beta = alpha.T().R().transpose()*beta.T().R();
		//Eigen::Vector3d r_alpha_target = _rref + r;
		Eigen::Vector3d r_alpha_target = _Tref.r() + r;

		rdot = _Tref.R().transpose()*(R_alpha_beta*(beta.V().v() - _rtarget.cross(beta.V().w()))
					- (alpha.V().v() - r_alpha_target.cross(alpha.V().w())));

		TaskJac Temp1;

#ifdef __GNUC__
		LieGroup::internal::_cross(_rtarget[0], _rtarget[1], _rtarget[2], subsys.J(_target).template bottomRows<3>(), Temp1);
		Temp1 = subsys.J(_target).template topRows<3>()  - Temp1;
#else
		LieGroup::internal::_cross(_rtarget[0], _rtarget[1], _rtarget[2], subsys.J(_target).bottomRows<3>(), Temp1);
		Temp1 = subsys.J(_target).topRows<3>()  - Temp1;
#endif

		TaskJac Temp2;
		// FIXME@20130531 Check algorithm
#ifdef __GNUC__
		LieGroup::internal::_cross(r_alpha_target[0], r_alpha_target[1], r_alpha_target[2], subsys.J(_ref).template bottomRows<3>(), Temp2);
		Temp2 = subsys.J(_ref).template topRows<3>()  - Temp2;
#else
		LieGroup::internal::_cross(r_alpha_target[0], r_alpha_target[1], r_alpha_target[2], subsys.J(_ref).bottomRows<3>(), Temp2);
		Temp2 = subsys.J(_ref).topRows<3>()  - Temp2;
#endif
		J.noalias() = _Tref.R().transpose()*(R_alpha_beta*Temp1 - Temp2);

		TaskJac dTemp1;
#ifdef __GNUC__
		LieGroup::internal::_cross(_rtarget[0], _rtarget[1], _rtarget[2], subsys.Jdot(_target).template bottomRows<3>(), dTemp1);
		dTemp1 = subsys.Jdot(_target).template topRows<3>()  - dTemp1;
#else
		LieGroup::internal::_cross(_rtarget[0], _rtarget[1], _rtarget[2], subsys.Jdot(_target).bottomRows<3>(), dTemp1);
		dTemp1 = subsys.Jdot(_target).topRows<3>()  - dTemp1;
#endif

		TaskJac dTemp2;
		// FIXME@20130531 Check algorithm
#ifdef __GNUC__
		LieGroup::internal::_cross(r_alpha_target[0], r_alpha_target[1], r_alpha_target[2], subsys.Jdot(_ref).template bottomRows<3>(), dTemp2);
		dTemp2 = subsys.Jdot(_ref).template topRows<3>()  - dTemp2;
#else
		LieGroup::internal::_cross(r_alpha_target[0], r_alpha_target[1], r_alpha_target[2], subsys.Jdot(_ref).bottomRows<3>(), dTemp2);
		dTemp2 = subsys.Jdot(_ref).topRows<3>()  - dTemp2;
#endif
		Jdot.noalias() = R_alpha_beta*dTemp1 - dTemp2;

		Eigen::Vector3d w_alpha_beta = beta.V().w() - R_alpha_beta.transpose()*alpha.V().w(); 

		TaskJac Temp3;
		LieGroup::internal::_cross(w_alpha_beta[0], w_alpha_beta[1], w_alpha_beta[2], Temp1, Temp3);

		TaskJac Temp4;
#ifdef __GNUC__
		LieGroup::internal::_cross(rdot[0], rdot[1], rdot[2], subsys.J(_ref).template bottomRows<3>(), Temp4);
#else
		LieGroup::internal::_cross(rdot[0], rdot[1], rdot[2], subsys.J(_ref).bottomRows<3>(), Temp4);
#endif
		Jdot.noalias() = _Tref.R().transpose()*(R_alpha_beta*(dTemp1 - Temp3) - (dTemp2 - Temp4));

		// FIXME Do we need these explicitly?
// 		J.leftCols<EARTH_DOF>().setZero(); 
// 		Jdot.leftCols<EARTH_DOF>().setZero(); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the absolute task displacement with the task Jacobian
	//!
	//! \param subsys System object of SubsysType
	//! \param r Task displacement or \f$ {^{ref}r_{tar}} \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::Displacement & r, TaskJac & J) const
	{
		Body const & alpha = subsys.body(_ref);
		Body const & beta = subsys.body(_target);

		//r = _Tref.R().transpose()*alpha.T().R().transpose()*(beta.T().transform(_rtarget) - alpha.T().transform(_rref));
		r = _Tref.R().transpose()*alpha.T().R().transpose()*(beta.T().transform(_rtarget) - alpha.T().transform(LieGroup::Displacement(_Tref.r())));

		Eigen::Matrix3d R_alpha_beta = alpha.T().R().transpose()*beta.T().R();
		//Eigen::Vector3d r_alpha_target = _rref + r;
		Eigen::Vector3d r_alpha_target = _Tref.r() + r;

		TaskJac Temp1;
#ifdef __GNUC__
		LieGroup::internal::_cross(_rtarget[0], _rtarget[1], _rtarget[2], subsys.J(_target).template bottomRows<3>(), Temp1);
		Temp1 = subsys.J(_target).template topRows<3>()  - Temp1;
#else
		LieGroup::internal::_cross(_rtarget[0], _rtarget[1], _rtarget[2], subsys.J(_target).bottomRows<3>(), Temp1);
		Temp1 = subsys.J(_target).topRows<3>()  - Temp1;
#endif

		TaskJac Temp2;
		// FIXME@20130531 Check algorithm
#ifdef __GNUC__
		LieGroup::internal::_cross(r_alpha_target[0], r_alpha_target[1], r_alpha_target[2], subsys.J(_ref).template bottomRows<3>(), Temp2);
		Temp2 = subsys.J(_ref).template topRows<3>()  - Temp2;
#else
		LieGroup::internal::_cross(r_alpha_target[0], r_alpha_target[1], r_alpha_target[2], subsys.J(_ref).bottomRows<3>(), Temp2);
		Temp2 = subsys.J(_ref).topRows<3>()  - Temp2;
#endif
		J.noalias() = _Tref.R().transpose()*(R_alpha_beta*Temp1 - Temp2);

		// FIXME Do we need these explicitly?
		// 		J.leftCols<EARTH_DOF>().setZero(); 
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Body index of the target body or \f$ \beta \f$
	//  ----------------------------------------------------------
	int _target; 

	//  ---------------------- Doxygen info ----------------------
	//! \brief Offset displacement of the target point relative to the target body frame, i.e. \f$ {^{\beta}r_{tar}} \f$
	//  ----------------------------------------------------------
	LieGroup::Displacement _rtarget;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Body index of the reference body or \f$ \alpha \f$
	//  ----------------------------------------------------------
	int _ref;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Offset transformation of the reference frame relative to the fixed ground frame, i.e. \f$ {^{-1}T_{ref}} \f$
	//  ----------------------------------------------------------
	LieGroup::HTransform _Tref;
};

} // namespace NRMKFoundation
