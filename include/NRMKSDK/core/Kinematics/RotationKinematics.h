//  ---------------------- Doxygen info ----------------------
//! \file RotationKinematics.h
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
//! \class RotationKinematics
//!
//! \brief
//! This implements a rotation kinematics class
//!
//! \details 
//!	Rotation tasks are described in terms of a rotation matrix, \f$ R \f$,
//! and its body angular velocity vector \f$ \omega \f$ such that \f$ \left \lceil \omega \right \rceil = R^T \dot{R} \f$. 
//! They are called the task rotation and task angular velocity, respectively.
//! The kinematic relationship with joint position and velocity vectors \f$ q \f$ and \f$ \dot{q} \f$ 
//! are defined by \f$ R = R(q) \f$ and \f$ \omega = J(q) \dot{q} \f$.
//! User should provide the algorithm to compute \f$ R \f$, \f$ \omega \f$, \f$ J \f$, and \f$ \dot{J} \f$ 
//! to implement a concrete rotation task.
//! 
//! \note
//! Users should inherit this class to define a customized kinematics.
//! See RotationAbsTaskKinematics and RotationRelTaskKinematics.
//! 
//! \tparam SubsysType Type of the subsys
//! \tparam RotationKinematicsType Derived rotation kinematics class 
//! 
//! \sa RotationAbsTaskKinematics
//! \sa RotationRelTaskKinematics
//  ----------------------------------------------------------
template <typename SubsysType, typename RotationKinematicsType>
class RotationKinematics : public Kinematics<SubsysType, 3, RotationKinematics<SubsysType, RotationKinematicsType>, LieGroup::Rotation, LieGroup::Vector3D>
{
public:
	typedef typename Kinematics<SubsysType, 3, RotationKinematics<SubsysType, RotationKinematicsType>, LieGroup::Rotation, LieGroup::Vector3D>::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename Kinematics<SubsysType, 3, RotationKinematics<SubsysType, RotationKinematicsType>, LieGroup::Rotation, LieGroup::Vector3D>::TaskJac TaskJac; //!< Typedef of task Jacobian

	//! \returns Reference to the derived object 
	inline RotationKinematicsType& derived() { return *static_cast<RotationKinematicsType*>(this); }
	//! \returns Constant reference to the derived object 
	inline const RotationKinematicsType& derived() const { return *static_cast<const RotationKinematicsType*>(this); }

public:
	//! \brief
	//! implements the rotation task kinematics
	//!
	//! \details
	//! The task rotation \f$ R \f$ and the task angular velocity \f$ \omega \f$ should be computed using the system's state. 
	//! It should be passed to the arguments R and w, respectively.
	//! 
	//! \param subsys System object of SubsysType
	//! \param R Task rotation or \f$ R \f$
	//! \param w Task angular velocity or \f$ \omega \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::Rotation & R, LieGroup::Vector3D & w) const
	{
		derived().kinematics(subsys, R, w);
	}
	
	//! \brief
	//! implements the rotation task kinematics
	//!
	//! \details
	//! The task rotation \f$ R \f$ should be computed using the system's state. 
	//! It should be passed to the arguments R.
	//! 
	//! \param subsys System object of SubsysType
	//! \param R Task rotation or \f$ R \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::Rotation & R) const
	{
		derived().kinematics(subsys, R);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the rotation task kinematics with the task Jacobian and its derivative
	//!
	//! \details
	//! The task rotation \f$ R \f$ and the task angular velocity \f$ \omega \f$ should be computed using the system's state. 
	//! It should be passed to the arguments R and w, respectively.
	//! Also, the task Jacobian \f$ J \f$ and its derivative \f$ \dot{J} \f$ should be computed and passed to the arguments J and Jdot, respectively.
	//! 
	//! \param subsys System object of SubsysType
	//! \param R Task rotation or \f$ R \f$
	//! \param w Task angular velocity or \f$ \omega \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//! \param Jdot Task Jacobian derivative matrix or \f$ \dot{J} \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::Rotation & R, LieGroup::Vector3D & w, TaskJac & J, TaskJac & Jdot) const
	{
		derived().jacobian(subsys, R, w, J, Jdot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the task transformation with the task Jacobian
	//!
	//! \details
	//! The task rotation \f$ R \f$ should be computed using the system's state and passed to the arguments R. 
	//! Also, the task Jacobian \f$ J \f$ should be computed and passed to the arguments J.
	//! 
	//! \param subsys System object of SubsysType
	//! \param R Task rotation or \f$ R \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::Rotation & R, TaskJac & J) const
	{
		derived().jacobian(subsys, R, J);
	}
};

//  ---------------------- Doxygen info ----------------------
//! \class RotationAbsTaskKinematics
//!
//! \brief
//! This implements an absolute rotation task kinematics class
//!
//! \details 
//!	Absolute rotation tasks are described in terms of the task rotation, \f$ {^{ref}R_{tar}} \f$,
//! and the task angular velocity vector \f$ {^{ref} \omega_{tar}} \f$, of the target body of index \f$ \beta \f$ (belonging to the subsys), 
//! with respect to the (fixed) ground. In particular, the target coordinate frame \f$ \{ tar \} \f$ can have
//! an offset rotation from the targte body frame \f$ \{ \beta \} \f$ or \f$ {^{\beta}R_{tar}} \f$, and the reference coordinate 
//! frame  \f$ \{ ref \} \f$ has an offset rotation from the fixed reference frame \f$ \{ -1 \} \f$ or \f$ {^{-1}R_{ref}} \f$. 
//! 
//! The task Jacobian and its derivative is computed in terms of the body Jacobians of the system. 
//! 
//! \note 
//! The system should be updated by update() before calling kinematics(). Furthermore, 
//! it should have updated body Jacobians by updateJacobian() before calling jacobian().
//! 
//! \tparam SubsysType Type of the subsys
//! 
//! \sa RotationKinematics
//! \sa RotationRelTaskKinematics
//  ----------------------------------------------------------
template<typename SubsysType>
class RotationAbsTaskKinematics : public RotationKinematics<SubsysType, RotationAbsTaskKinematics<SubsysType> >
{
public:
	typedef typename RotationKinematics<SubsysType, RotationAbsTaskKinematics<SubsysType> >::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename RotationKinematics<SubsysType, RotationAbsTaskKinematics<SubsysType> >::TaskJac TaskJac; //!< Typedef of task Jacobian matrix

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the absolute rotation task kinematics
	//!
	//! \param subsys System object
	//! \param target Body index of the target body or \f$ \beta \f$
	//! \param Rtarget Offset rotation of the target frame relative to the target body frame, i.e. \f$ {^{\beta}R_{tar}} \f$
	//! \param Rref Offset rotation of the reference frame relative to the fixed ground frame, i.e. \f$ {^{-1}R_{ref}} \f$
	//  ----------------------------------------------------------
	inline RotationAbsTaskKinematics(SubsysType const & subsys, 
		int target, LieGroup::Rotation const & Rtarget, LieGroup::Rotation const & Rref)
		: RotationKinematics<SubsysType, RotationAbsTaskKinematics<SubsysType> >()
		, _target(target), _Rtarget(Rtarget), _Rref(Rref)
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the absolute rotation task kinematics
	//!
	//! \note 
	//!	set() should be called after construction
	//!
	//! \param subsys System object
	//  ----------------------------------------------------------
	inline RotationAbsTaskKinematics(SubsysType const & subsys)
		: RotationKinematics<SubsysType, RotationAbsTaskKinematics<SubsysType> >()
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the data for absolute rotation task kinematics
	//!
	//! \param target Body index of the target body or \f$ \beta \f$
	//! \param Rtarget Offset rotation of the target frame relative to the target body frame, i.e. \f$ {^{\beta}R_{tar}} \f$
	//! \param Rref Offset rotation of the reference frame relative to the fixed ground frame, i.e. \f$ {^{-1}R_{ref}} \f$
	//  ----------------------------------------------------------
	inline void set(int target, LieGroup::Rotation const & Rtarget, LieGroup::Rotation const & Rref)
	{
		_target = target;
		_Rtarget = Rtarget;
		_Rref = Rref;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the absolute rotation task kinematics
	//!
	//! \param subsys System object of SubsysType
	//! \param R Task rotation or \f$ {^{ref}R_{tar}} \f$
	//! \param w Task angular velocity or \f$ {^{ref}\omega_{tar}} \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::Rotation & R, LieGroup::Vector3D & w) const
	{
		Body const & beta = subsys.body(_target);

		R = _Rref.transpose()*beta.T().R()*_Rtarget;
		w = _Rtarget.transpose()*beta.V().w(); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the absolute rotation task kinematics
	//!
	//! \param subsys System object of SubsysType
	//! \param R Task rotation or \f$ {^{ref}R_{tar}} \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::Rotation & R) const
	{
		Body const & beta = subsys.body(_target);

		R = _Rref.transpose()*beta.T().R()*_Rtarget;
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the absolute rotation task kinematics with the task Jacobian and its derivative
	//!
	//! \param subsys System object of SubsysType
	//! \param R Task rotation or \f$ {^{ref}R_{tar}} \f$
	//! \param w Task angular velocity or \f$ {^{ref}\omega_{tar}} \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//! \param Jdot Task Jacobian derivative matrix or \f$ \dot{J} \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::Rotation & R, LieGroup::Vector3D & w, TaskJac & J, TaskJac & Jdot) const
	{
		kinematics(subsys, R, w);

#ifdef __GNUC__
		J.noalias() = _Rtarget.transpose()*subsys.J(_target).template bottomRows<3>(); //_J[target];
		Jdot.noalias() = _Rtarget.transpose()*subsys.Jdot(_target).template bottomRows<3>(); //_Jdot[target];
#else
		J.noalias() = _Rtarget.transpose()*subsys.J(_target).bottomRows<3>(); //_J[target];
		Jdot.noalias() = _Rtarget.transpose()*subsys.Jdot(_target).bottomRows<3>(); //_Jdot[target];
#endif
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the absolute task rotation with the task Jacobian
	//!
	//! \param subsys System object of SubsysType
	//! \param R Task rotation or \f$ {^{ref}R_{tar}} \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::Rotation & R, TaskJac & J) const
	{
		Body const & beta = subsys.body(_target);

		R = _Rref.transpose()*beta.T().R()*_Rtarget;

#ifdef __GNUC__	
		J.noalias() = _Rtarget.transpose()*subsys.J(_target).template bottomRows<3>(); //_J[target];
#else
		J.noalias() = _Rtarget.transpose()*subsys.J(_target).bottomRows<3>(); //_J[target];
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
	//! \brief Offset rotation of the target frame relative to the target body frame, i.e. \f$ {^{\beta}R_{tar}} \f$
	//  ----------------------------------------------------------
	LieGroup::Rotation _Rtarget;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Offset rotation of the reference frame relative to the fixed ground frame, i.e. \f$ {^{-1}R_{ref}} \f$
	//  ----------------------------------------------------------
	LieGroup::Rotation _Rref;
};

//  ---------------------- Doxygen info ----------------------
//! \class RotationRelTaskKinematics
//!
//! \brief
//! This implements an relative rotation task kinematics class
//!
//! \details 
//!	Relative rotation tasks are described in terms of the task rotation, \f$ {^{ref}R_{tar}} \f$,
//! and the task angular velocity vector \f$ {^{ref} \omega_{tar}} \f$, of the target body of index \f$ \beta \f$ (belonging to the subsys), 
//! with respect to the reference body of index \f$ \alpha \f$ (belonging to the subsys). 
//! In particular, the target coordinate frame \f$ \{ tar \} \f$ can have
//! an offset rotation from the targte body frame \f$ \{ \beta \} \f$ or \f$ {^{\beta}R_{tar}} \f$, and the reference coordinate 
//! frame  \f$ \{ ref \} \f$ has an offset rotation from the reference body frame \f$ \{ \alpha \} \f$ or \f$ {^{\alpha}R_{ref}} \f$. 
//! 
//! The task Jacobian and its derivative is computed in terms of the body Jacobians of the system. 
//! 
//! \note 
//! The system should be updated by update() before calling kinematics(). Furthermore, 
//! it should have updated body Jacobians by updateJacobian() before calling jacobian().
//! 
//! \tparam SubsysType Type of the subsys
//! 
//! \sa RotationKinematics
//! \sa RotationAbsTaskKinematics
//  ----------------------------------------------------------
template<typename SubsysType>
class RotationRelTaskKinematics : public RotationKinematics<SubsysType, RotationRelTaskKinematics<SubsysType> >
{
public:
	typedef typename RotationKinematics<SubsysType, RotationRelTaskKinematics<SubsysType> >::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename RotationKinematics<SubsysType, RotationRelTaskKinematics<SubsysType> >::TaskJac TaskJac; //!< Typedef of task Jacobian matrix

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the relative rotation task kinematics
	//!
	//! \param subsys System object
	//! \param target Body index of the target body or \f$ \beta \f$
	//! \param Rtarget Offset rotation of the target frame relative to the target body frame, i.e. \f$ {^{\beta}R_{tar}} \f$
	//! \param ref Body index of the reference body or \f$ \alpha \f$
	//! \param Rref Offset rotation of the reference frame relative to the fixed ground frame, i.e. \f$ {^{-1}R_{ref}} \f$
	//  ----------------------------------------------------------
	inline RotationRelTaskKinematics(SubsysType const & subsys, 
		int target, LieGroup::Rotation const & Rtarget, int ref, LieGroup::Rotation const & Rref)
		: RotationKinematics<SubsysType, RotationRelTaskKinematics<SubsysType> >()
		, _target(target), _Rtarget(Rtarget), _ref(ref), _Rref(Rref)
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the relative rotation task kinematics
	//!
	//! \note 
	//!	set() should be called after construction
	//!
	//! \param subsys System object
	//  ----------------------------------------------------------
	inline RotationRelTaskKinematics(SubsysType const & subsys)
		: RotationKinematics<SubsysType, RotationRelTaskKinematics<SubsysType> >()
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the data for relative rotation task kinematics
	//!
	//! \param target Body index of the target body or \f$ \beta \f$
	//! \param Rtarget Offset rotation of the target frame relative to the target body frame, i.e. \f$ {^{\beta}R_{tar}} \f$
	//! \param ref Body index of the reference body or \f$ \alpha \f$
	//! \param Rref Offset rotation of the reference frame relative to the fixed ground frame, i.e. \f$ {^{-1}R_{ref}} \f$
	//  ----------------------------------------------------------
	inline void set(int target, LieGroup::Rotation const & Rtarget, int ref, LieGroup::Rotation const & Rref)
	{
		_target = target;
		_Rtarget = Rtarget;
		_ref = ref;
		_Rref = Rref;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the relative rotation task kinematics
	//!
	//! \param subsys System object of SubsysType
	//! \param R Task rotation or \f$ {^{ref}R_{tar}} \f$
	//! \param w Task angular velocity or \f$ {^{ref}\omega_{tar}} \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::Rotation & R, LieGroup::Vector3D & w) const
	{
		Body const & alpha = subsys.body(_ref);
		Body const & beta = subsys.body(_target);
		
		Eigen::Matrix3d R_alpha_beta = alpha.T().R().transpose()*beta.T().R();
		R = _Rref.transpose()*R_alpha_beta*_Rtarget;

		Eigen::Vector3d w_alpha_beta = beta.V().w() - R_alpha_beta.transpose()*alpha.V().w(); 
		//w = _Rtarget.irotate(w_alpha_beta);
		w = _Rtarget.transpose()*w_alpha_beta;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the relative rotation task kinematics
	//!
	//! \param subsys System object of SubsysType
	//! \param R Task rotation or \f$ {^{ref}R_{tar}} \f$
	//  ----------------------------------------------------------
	inline void kinematics(SubsysType const & subsys, LieGroup::Rotation & R) const
	{
		Body const & alpha = subsys.body(_ref);
		Body const & beta = subsys.body(_target);

		Eigen::Matrix3d R_alpha_beta = alpha.T().R().transpose()*beta.T().R();
		R = _Rref.transpose()*R_alpha_beta*_Rtarget;
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the relative rotation task kinematics with the task Jacobian and its derivative
	//!
	//! \param subsys System object of SubsysType
	//! \param R Task rotation or \f$ {^{ref}R_{tar}} \f$
	//! \param w Task angular velocity or \f$ {^{ref}\omega_{tar}} \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//! \param Jdot Task Jacobian derivative matrix or \f$ \dot{J} \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::Rotation & R, LieGroup::Vector3D & w, TaskJac & J, TaskJac & Jdot) const
	{
		Body const & alpha = subsys.body(_ref);
		Body const & beta = subsys.body(_target);

		Eigen::Matrix3d R_alpha_beta = alpha.T().R().transpose()*beta.T().R();
		R = _Rref.transpose()*R_alpha_beta*_Rtarget;

		Eigen::Vector3d w_alpha_beta = beta.V().w() - R_alpha_beta.transpose()*alpha.V().w(); 
		//w = _Rtarget.irotate(w_alpha_beta); 
		w = _Rtarget.transpose()*w_alpha_beta; 

		TaskJac Temp; 
		TaskJac dTemp;

#ifdef __GNUC__
		Temp.noalias() = R_alpha_beta.transpose()*subsys.J(_ref).template bottomRows<3>(); 
		
		LieGroup::internal::_cross(w_alpha_beta[0], w_alpha_beta[1], w_alpha_beta[2], Temp, dTemp);
		dTemp.noalias() -= R_alpha_beta.transpose()*subsys.Jdot(_ref).template bottomRows<3>();
		
		J.noalias() = _Rtarget.transpose()*(subsys.J(_target).template bottomRows<3>() - Temp);
		Jdot.noalias() = _Rtarget.transpose()*(subsys.Jdot(_target).template bottomRows<3>() + dTemp);
#else
		Temp.noalias() = R_alpha_beta.transpose()*subsys.J(_ref).bottomRows<3>(); 
		
		LieGroup::internal::_cross(w_alpha_beta[0], w_alpha_beta[1], w_alpha_beta[2], Temp, dTemp);
		dTemp.noalias() -= R_alpha_beta.transpose()*subsys.Jdot(_ref).bottomRows<3>();
		
		J.noalias() = _Rtarget.transpose()*(subsys.J(_target).bottomRows<3>() - Temp);
		Jdot.noalias() = _Rtarget.transpose()*(subsys.Jdot(_target).bottomRows<3>() + dTemp);
#endif
		// FIXME Do we need these explicitly?
// 		J.leftCols<EARTH_DOF>().setZero(); 
// 		Jdot.leftCols<EARTH_DOF>().setZero(); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the relative task rotation with the task Jacobian
	//!
	//! \param subsys System object of SubsysType
	//! \param R Task rotation or \f$ {^{ref}R_{tar}} \f$
	//! \param J Task Jacobian matrix or \f$ J \f$
	//  ----------------------------------------------------------
	inline void jacobian(SubsysType const & subsys, LieGroup::Rotation & R, TaskJac & J) const
	{
		Body const & alpha = subsys.body(_ref);
		Body const & beta = subsys.body(_target);

		Eigen::Matrix3d R_alpha_beta = alpha.T().R().transpose()*beta.T().R();
		R = _Rref.transpose()*R_alpha_beta*_Rtarget;

#ifdef __GNUC__
		J.noalias() = _Rtarget.transpose()*(subsys.J(_target).template bottomRows<3>() - R_alpha_beta.transpose()*subsys.J(_ref).template bottomRows<3>());
#else
		J.noalias() = _Rtarget.transpose()*(subsys.J(_target).bottomRows<3>() - R_alpha_beta.transpose()*subsys.J(_ref).bottomRows<3>());
#endif
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
	//! \brief Offset rotation of the target frame relative to the target body frame, i.e. \f$ {^{\beta}R_{tar}} \f$
	//  ----------------------------------------------------------
	LieGroup::Rotation _Rtarget;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Body index of the reference body or \f$ \alpha \f$
	//  ----------------------------------------------------------
	int _ref;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Offset rotation of the reference frame relative to the reference body frame, i.e. \f$ {^{\alpha}T_{ref}} \f$
	//  ----------------------------------------------------------
	LieGroup::Rotation _Rref;
};

} // namespace NRMKFoundation
