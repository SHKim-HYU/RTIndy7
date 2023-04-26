//  ---------------------- Doxygen info ----------------------
//! \file Interpolator.h
//!
//! \brief
//! Header file for the class Interpolator (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a interpolator base class
//! to be used for the interface of NRMKFoundation library.
//!  
//! \n
//! \n
//! \n
//! \copydetails Neuromeka Foundation Library
//! \n
//! \n
//! \n
//! Neuromeka Co., Ltd. \n
//! South Korea\n
//! \n
//! http://www.neuromeka.com\n
//!
//! \date May 2016
//! 
//! \version 1.9.4
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!	
//!
//! \note v1.9.4 (20160513): Added synchronization option for vector interpolator
//!
//! \note Copyright (C) 2013-2016 Neuromeka Co., Ltd.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#include <Eigen/Eigen>

#include "PolynomialAlgorithm.h"
#include "MultipointAlgorithm.h"

namespace NRMKFoundation
{
namespace internal
{
}

//  ---------------------- Doxygen info ----------------------
//! \class Interpolator
//!
//! \brief
//! This implements a base two-point interpolator class
//!
//! \details
//! Class Interpolator implements any type of two-point interpolators, i.e. given a pair of initial and final boundary conditions,
//! which generates the intermediate trajectories consisting of a set of position-level, velocity-level, and acceleration-level task
//! values for any kind of tasks at intermediate time instances. They are called the desired task position, velocity, and acceleration, respectively.
//!
//! Since there may exists many types to represent task position, task velocity, and task acceleration,
//! they are specified as the template arguments. 
//!	The scheme to interpolate the trajectory is defined by deriving this class by static polymorphism in terms of the CRTP pattern.
//! (See http://en.wikipedia.org/wiki/Curiously_recurring_template_pattern)
//!  
//! \tparam _DIM Dimension of the task variable
//! \tparam InterpolatorType Derived class 
//! \tparam _PosType Type of the position-level task variable. 
//!		The default type is Eigen::Matrix<double, _DIM, 1>.
//! \tparam _VelType Type of the velocity-level task variable.
//!		The default type is Eigen::Matrix<double, _DIM, 1>.
//! \tparam _AccType Type of the acceleration-level task variable.
//!		The default type is same as _VelType.
//!
//! \sa PolynomialInterpolator
//! \sa RotationInterpolator
//! \sa HTransformInterpolator
//  ----------------------------------------------------------
template<int _DIM, typename InterpolatorType, typename _PosType = Eigen::Matrix<double, _DIM, 1>, typename _VelType = Eigen::Matrix<double, _DIM, 1>, typename _AccType = _VelType>
class Interpolator
{	
public:
	//  ---------------------- Doxygen info ----------------------
	//! \enum
	//! 
	//! \brief Provides compile-time constants 
	//  ----------------------------------------------------------
	enum 
	{	
		DIM = _DIM, //!< Task dimension
	};

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides Typedefs 
	//  ----------------------------------------------------------
	typedef _PosType PosType; //!< Typedef of the type for position variable
	typedef _VelType VelType; //!< Typedef of the type for velocity variable
	typedef _AccType AccType; //!< Typedef of the type for acceleration variable
	typedef Eigen::Matrix<double, DIM, 1> TaskVec; //!< Typedef of task vector
	typedef Eigen::Matrix<double, DIM, DIM> TaskMat; //!< Typedef of task matrix

	//! \returns Reference to the derived object 
	inline InterpolatorType& derived() { return *static_cast<InterpolatorType*>(this); }
	//! \returns Constant reference to the derived object 
	inline const InterpolatorType& derived() const { return *static_cast<const InterpolatorType*>(this); }

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! fixes the interpolated trajectory
	//!
	//! \param pos Initial task position
	//! \param vel Initial task velocity
	//! \param acc Initial task acceleration
	//  ----------------------------------------------------------
	inline void setTraj(PosType const & pos, VelType const & vel = TaskVec::Zero(), AccType const & acc = TaskVec::Zero())
	{
		derived().setTraj(pos, vel, acc);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the initial condition of the interpolated trajectory
	//!
	//! \note
	//! This should be called first. 
	//! 
	//! \param t_ini Initial time
	//! \param pos_ini Initial task position
	//! \param vel_ini Initial task velocity
	//! \param acc_ini Initial task acceleration
	//  ----------------------------------------------------------
	inline void setInitialTraj(double t_ini, PosType const & pos_ini, VelType const & vel_ini = TaskVec::Zero(), AccType const & acc_ini = TaskVec::Zero())
	{
		derived().setInitialTraj(t_ini, pos_ini, vel_ini, acc_ini);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the final condition of the interpolated trajectory
	//!
	//! \note
	//! This should be called after setInitialTraj(). 
	//! 
	//! \param t_final Final time
	//! \param pos_final Final task position
	//! \param vel_final Final task velocity
	//! \param acc_final Final task acceleration
	//  ----------------------------------------------------------
	inline void setTargetTraj(double t_final, PosType const & pos_final, VelType const & vel_final = TaskVec::Zero(), AccType const & acc_final = TaskVec::Zero())
	{
		derived().setTargetTraj(t_final, pos_final, vel_final, acc_final);
	}

// 	//  ---------------------- Doxygen info ----------------------
// 	//! \brief
// 	//! sets the final condition of the interpolated trajectory
// 	//!
// 	//! \details
// 	//! Only symmetric bounds are allowed. That is, minimum velocity is the 
// 	//! negative of the maximum velocity.
// 	//! This may not be called but, if it is called, 
// 	//! it should be called after setFinalTraj() before calling traj(). 
// 	//! 
// 	//! \param vel_max Maximal velocity 
// 	//! \param acc_max Maximal acceleration
// 	//! \param jerk_max Maximal jerk
// 	//  ----------------------------------------------------------
// 	void setBoundaryCond(VelType const & vel_max, AccType const & acc_max, AccType const & jerk_max)
// 	{
// 		derived().setBoundaryCond(vel_max, acc_max, jerk_max);
// 	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the desired trajectory for the time instant
	//! 
	//! \note
	//! This should be called after setTargetTraj(). 
	//!
	//! \param t Desired time
	//! \param des_pos Desired task position 
	//! \param des_vel Desired task velocity 
	//! \param des_acc Desired task acceleration 
	//  ----------------------------------------------------------
	inline void traj(double t, PosType & des_pos, VelType & des_vel, AccType & des_acc)
	{
		derived().traj(t, des_pos, des_vel, des_acc);
	}

};	// class Interpolator

//  ---------------------- Doxygen info ----------------------
//! \class VectorInterpolator
//!
//! \brief
//! This implements a base two-point polynomial interpolator class
//!
//! \details 
//! This class implements a two-point polynomial interpolator algorithm 
//! for tasks described in terms of vectors, such as functional tasks and quasi-coordinate tasks.
//! Given a set of initial conditions \f$ p_0 \f$, \f$ v_0 \f$, and \f$ a_0 \f$ (all belonging to \f$ \mathbb{R}^m \f$) 
//! at time \f$ t_0 \f$ and a set of final conditions \f$ p_f \f$, \f$ v_f \f$, and \f$ a_f \f$ (all belonging to \f$ \mathbb{R}^m \f$) 
//! at time \f$ t_f \f$, the intermediate trajectory at time \f$ t \f$ consisting of \f$ p_{des}(t) \f$, \f$ v_{des}(t) \f$, and \f$ a_{des}(t) \f$
//! is computed by the polynomial equation in time \f$ t \f$.
//!  
//! The task is a vector, and each component is interpolated by a polynomial interpolator algorithm for a scalar variable.
//! That is, this class has as many scalar interpolator algorithms as the task dimension.
//! Since there are a number of different polynomial algorithms, a class implementing specific polynomial 
//! algorithm for a scalar variable should be provided as a template argument. 
//! A number of polynomial algorithms are provided.
//! 
//! \tparam DIM Dimension of the task variable
//! \tparam PolynomialAlgorithm Polynomial interpolator algorithm type
//!
//! \sa Interpolator
//! \sa internal::PolynomialInterpolatorType
//  ----------------------------------------------------------
template<int DIM, typename PolynomialAlgorithm>
class VectorInterpolator : public Interpolator<DIM, VectorInterpolator<DIM, PolynomialAlgorithm> >
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides Typedefs 
	//  ----------------------------------------------------------
	typedef Interpolator<DIM, VectorInterpolator<DIM, PolynomialAlgorithm> > Base; //!< Typedef of the type for base class
	
	typedef typename Base::PosType PosType; //!< Typedef of the type for position variable
	typedef typename Base::VelType VelType; //!< Typedef of the type for velocity variable
	typedef typename Base::AccType AccType; //!< Typedef of the type for acceleration variable

	typedef typename Base::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename Base::TaskMat TaskMat; //!< Typedef of task Jacobian

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! fixes the interpolated trajectory
	//!
	//! \param p Initial task pition
	//! \param v Initial task vocity
	//! \param a Initial task aeleration
	//  ----------------------------------------------------------
	inline void setTraj(PosType const & p, VelType const & v = TaskVec::Zero(), AccType const & a = TaskVec::Zero())
	{
// #ifdef __RTX__
// 		_tf = -DBL_MAX;
// #else
		_tf = -std::numeric_limits<double>::infinity();
//#endif

		_pf = p;
		_vf = v;
		_af = a;
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the initial condition of the interpolated trajectory
	//!
	//! \note
	//! This should be called first. 
	//! 
	//! \param t0 Initial time or \f$ t_0 \f$
	//! \param p0 Initial task position vector or \f$ p_0 \f$
	//! \param v0 Initial task velocity vector or \f$ v_0 \f$
	//! \param a0 Initial task acceleration vector or \f$ a_0 \f$
	//  ----------------------------------------------------------
	inline void setInitialTraj(double t0, PosType const & p0, VelType const & v0 = TaskVec::Zero(), AccType const & a0 = TaskVec::Zero())
	{
		_t0 = t0; 
		_p0 = p0;
		_v0 = v0;
		_a0 = a0;

		/// FIXME @20160229
		_tf = internal::_traj_time_undefined;
		_pf = p0;
		_vf = v0;
		_af = a0;

		for (int i = 0; i < DIM; i++)
			_alg[i].setInitialTraj(t0, p0[i], v0[i], a0[i]);

		/// ADDED @20160509
		_sync = false;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the final condition of the interpolated trajectory
	//!
	//! \note
	//! This should be called after setInitialTraj(). 
	//! 
	//! \param tf Final time or \f$ t_f \f$
	//! \param pf Final task position vector or \f$ p_f \f$
	//! \param vf Final task velocity vector or \f$ v_f \f$
	//! \param af Final task acceleration vector or \f$ a_f \f$
	//  ----------------------------------------------------------
	inline void setTargetTraj(double tf, PosType const & pf, VelType const & vf = TaskVec::Zero(), AccType const & af = TaskVec::Zero())
	{
		_tf = tf;
		_pf = pf;
		_vf = vf;
		_af = af;
		_isTargetReached = false;

		double maxDuration = 0;
		for (int i = 0; i < DIM; i++)
		{
			_alg[i].setTargetTraj(tf, pf[i], vf[i], af[i]);
			if (_alg[i].duration() > maxDuration)
				maxDuration = _alg[i].duration();
		}

		/// ADDED @20160509
		if (_sync)
		{
			for (int i = 0; i < DIM; i++) 
				_alg[i].setDuration(maxDuration);
		}
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the boundary condition of the interpolated trajectory
	//!
	//! \details
	//! One may specify the allowable maximum task velocity, acceleration, and jerk vectors. 
	//! Only symmetric bounds are allowed. That is, minimum value is the negative of the maximum. 
	//! Depending on the employed scalar interpolator algorithm this may do nothing.
	//!
	//! \note
	//! This may not be called but, if it is to be called, 
	//! it should be called after setTargetTraj() before calling traj(). 
	//! 
	//! \param vel_max Maximal task velocity vector
	//! \param acc_max Maximal task acceleration vector
	//! \param jerk_max Maximal task jerk vector
 	//  ----------------------------------------------------------
	inline void setBoundaryCond(VelType const & vel_max, AccType const & acc_max = TaskVec::Zero(), AccType const & jerk_max = TaskVec::Zero())
	{
		for (int i = 0; i < DIM; i++)
			_alg[i].setBoundaryCond(vel_max[i], acc_max[i], jerk_max[i]);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the desired trajectory for the time instant
	//! 
	//! \details 
	//! The desired time should be between the initial time \f$ t_0 \f$ and the final time \f$ t_f \f$ .
	//! For times before the initial time, i.e. \f$ t < t_0 \f$, the desired trajectory is fixed to the initial condition.
	//! For times after the final time, i.e. \f$ t > t_f \f$, the desired trajectory is fixed to the final condition.
	//! 
	//! \note
	//! This should be called after setTargetTraj(). 
	//!
	//! \param t Desired time
	//! \param pd Desired task position or \f$ p_{des} \f$
	//! \param vd Desired task velocity or \f$ v_{des} \f$
	//! \param ad Desired task acceleration or \f$ a_{des} \f$ 
	//  ----------------------------------------------------------
	inline void traj(double t, PosType & pd, VelType & vd, AccType & ad)
	{
		_isTargetReached = true;
		if (t > _tf)
		{
			pd = _pf;
			vd = _vf;
			ad = _af; 
		}
		else if (t < _t0)
		{
			_isTargetReached = false;
			pd = _p0;
			vd = _v0;
			ad = _a0;
		}
		else
		{
			_isTargetReached = false;
			for (int i = 0; i < DIM; i++)
				_alg[i].traj(t, pd[i], vd[i], ad[i]);
		}
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the period for cyclic update of trajectory
	//!
	//! \param delT period (in sec) (default value = 0)
	//  ----------------------------------------------------------
	inline void setPeriod(double delT)
	{
		for (int i = 0; i < DIM; i++)
			_alg[i].setPeriod(delT); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! access the component interpolator the desired trajectory for the time instant
	//! 
	//! \param k Index
	//  ----------------------------------------------------------
	PolynomialAlgorithm & algorithm(int k)
	{
		return _alg[k];
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! This returns the final trajectory time.
	//  ----------------------------------------------------------
	double tf() const { return _tf; }
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! This returns whether the robot reached target.
	//  ----------------------------------------------------------
	bool isTargetReached() { return _isTargetReached; }

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! This activates sync option.	
	void setSync(bool sync = true) { _sync = sync; }

	int getCurSeg()
	{
		int largeSeg = 0;
		for (int i = 0; i < DIM; i++)
		{
			if (largeSeg < _alg[i].getCurSeg())
				largeSeg = _alg[i].getCurSeg();
		}
		return largeSeg;
	}
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Array of scalar interpolator algorithms (of type PolynomialAlgorithm)
	//  ----------------------------------------------------------
	PolynomialAlgorithm _alg[DIM];

	//  ---------------------- Doxygen info ----------------------
	//! \brief Initial time
	//  ----------------------------------------------------------
	double _t0;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Initial task position vector
	//  ----------------------------------------------------------
	PosType _p0;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Initial task velocity vector
	//  ----------------------------------------------------------
	VelType _v0;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Initial task acceleration vector
	//  ----------------------------------------------------------
	AccType _a0;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Final time
	//  ----------------------------------------------------------
	double _tf;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Final task position vector
	//  ----------------------------------------------------------
	PosType _pf;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Final task velocity vector
	//  ----------------------------------------------------------
	VelType _vf;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Final task acceleration vector
	//  ----------------------------------------------------------
	AccType _af;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Flag to verify whether robot is in motion
	//  ----------------------------------------------------------
	bool _isTargetReached;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Flag to sychronize trajectory
	//  ----------------------------------------------------------
	bool _sync;

};

} // namespace NRMKFoundation
