//  ---------------------- Doxygen info ----------------------
//! \file HTransformInterpolator.h
//!
//! \brief
//! Header file for the class HTransformInterpolator (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a controller base class
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
//! South KoreaY\n
//! \n
//! http://www.neuromeka.com\n
//!
//! \date March 2016
//! 
//! \version 1.9.4
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//! \note 
//!	 - v1.9.4 (20160316): exposed alg()
//!
//! \note Copyright (C) 2013-2016 Neuromeka Co., Ltd.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#include <Eigen/Eigen>

#include "LieGroup/LieGroup.h"

//#include "Interpolator.h"

namespace NRMKFoundation
{
namespace internal
{
}

//  ---------------------- Doxygen info ----------------------
//! \class HTransformInterpolator
//!
//! \brief
//! This implements a homogeneous transformation interpolator class.
//!
//! \details
//! Homogeneous transformation interpolator interpolates three-dimensional rigid body motion task, which is described by 
//! the homogeneous transformation matrix as the task position, the body twist as the task velocity, and 
//! the body acceleration as the task acceleration. Homogeneous transformation matrices are represented by LieGroup::HTransform, and 
//! angular velocities and accelerations are represented by LieGroup::Twist. That is,
//! given a set of initial conditions \f$ T_0 \f$, \f$ V_0 \f$, and \f$ \dot{V}_0 \f$ 
//! at time \f$ t_0 \f$ and a set of final conditions \f$ T_f \f$, \f$ V_f \f$, and \f$ \dot{V}_f \f$  
//! at time \f$ t_f \f$, the intermediate trajectory at time \f$ t \f$ consisting of \f$ T_{des}(t) \f$, \f$ V_{des}(t) \f$, and \f$ \dot{V}_{des}(t) \f$
//! is computed.
//! 
//! Since homogeneous transformation matrices comprises the Lie group \f$ SE(3) \f$, 
//! the interpolation is done in terms of the corresponding Lie algebra vector (of type LieGroup::Twist) \f$ \lambda \f$.
//! Since there are many types of interpolator algorithms for six-dimensional vectors (i.e. \f$ \lambda \f$),
//! it is passed as a template argument.
//! 
//! \tparam InterpolatorAlgorithm Six-dimensional vector interpolator algorithm type
//!
//! \sa Interpolator
//  ----------------------------------------------------------
template<typename InterpolatorAlgorithm>
class HTransformInterpolator
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides Typedefs 
	//  ----------------------------------------------------------
	typedef LieGroup::HTransform PosType; //!< Typedef of the type for position variable
	typedef LieGroup::Twist VelType; //!< Typedef of the type for velocity variable
	typedef LieGroup::Twist AccType; //!< Typedef of the type for acceleration variable
	
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! fixes the interpolated trajectory
	//! 
	//! \param T Initial task transformation or \f$ T_0 \f$
	//! \param V Initial task twist or \f$ V_0 \f$
	//! \param Vdot Initial task acceleration or \f$ \dot{V}_0 \f$
	//  ----------------------------------------------------------
	void setTraj(LieGroup::HTransform const & T, LieGroup::Twist const & V = LieGroup::Twist(), LieGroup::Twist const & Vdot = LieGroup::Twist())
	{
		_T0 = T;

		/// FIXME @20161017
#if !defined(__GNUC__)
		_interpolator.setTraj(LieGroup::Twist(), V, Vdot);
#else
		_interpolator.setTraj(Vector6d::Zero(), V, Vdot);
#endif
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the initial condition of the interpolated trajectory
	//!
	//! \note
	//! This should be called first. 
	//! 
	//! \param t0 Initial time or \f$ t_0 \f$
	//! \param T0 Initial task transformation or \f$ T_0 \f$
	//! \param V0 Initial task twist or \f$ V_0 \f$
	//! \param Vdot0 Initial task acceleration or \f$ \dot{V}_0 \f$
	//  ----------------------------------------------------------
	void setInitialTraj(double t0, LieGroup::HTransform const & T0, LieGroup::Twist const & V0 = LieGroup::Twist(), LieGroup::Twist const & Vdot0 = LieGroup::Twist())
	{
		_T0 = T0;
#if !defined(__GNUC__)
		_interpolator.setInitialTraj(t0, LieGroup::Twist(), V0, Vdot0);
#else
		_interpolator.setInitialTraj(t0, Vector6d::Zero(), V0, Vdot0);
#endif
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the final condition of the interpolated trajectory
	//!
	//! \note
	//! This should be called after setInitialTraj(). 
	//! 
	//! \param tf Final time or \f$ t_f \f$
	//! \param Tf Final task transformation or \f$ T_f \f$
	//! \param Vf Final task twist or \f$ V_f \f$
	//! \param Vdotf Final task acceleration or \f$ \dot{V}_f \f$
	//  ----------------------------------------------------------
	void setTargetTraj(double tf, LieGroup::HTransform const & Tf, LieGroup::Twist const & Vf = LieGroup::Twist(), LieGroup::Twist const & Vdotf = LieGroup::Twist())
	{
		LieGroup::HTransform T = _T0.icascade(Tf);
		LieGroup::Twist lambda_f = T.expCoord(); 
		
		double alpha;
		double beta;
		double theta_sqr;
		lambda_f.compute_terms(&alpha, &beta, &theta_sqr);

		Matrix6d dexpinv = lambda_f.dexp_inv(alpha, beta, theta_sqr);
		LieGroup::Twist lambdadot_f = Vf.upperBlockTriTransposedMult(dexpinv);

		Matrix6d ddt_dexp = lambda_f.ddt_dexp(alpha, beta, theta_sqr, 
			lambda_f.w().dot(lambda_f.v()), lambda_f.w().dot(lambdadot_f.w()), lambda_f.w().dot(lambdadot_f.v()), lambdadot_f.w().dot(lambda_f.v()),
			lambdadot_f);

		LieGroup::Twist Temp = Vdotf - lambdadot_f.upperBlockTriTransposedMult(ddt_dexp);
		LieGroup::Twist lambdaddot_f = Temp.upperBlockTriTransposedMult(dexpinv);

		_interpolator.setTargetTraj(tf, lambda_f, lambdadot_f, lambdaddot_f);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the boundary condition of the interpolated trajectory
	//!
	//! \details
	//! One may specify the allowable maximum velocity, acceleration, and jerk vectors on the Lie algebra trajectory \f$ \lambda \f$. 
	//! Only symmetric bounds are allowed. That is, minimum value is the negative of the maximum. 
	//! Depending on the employed scalar interpolator algorithm this may do nothing.
	//!
	//! \note
	//! This may not be called but, if it is to be called, 
	//! it should be called after setFinalTraj() before calling traj(). 
	//! 
	//! \param lambdadot_max Maximum velocity vector on \f$ \dot{\lambda} \f$
	//! \param lambdaddot_max Maximum acceleration vector on \f$ \ddot{\lambda} \f$
	//! \param lambdadddot_max Maximum jerk vector on \f$ \dddot{\lambda} \f$
	//  ----------------------------------------------------------
	void setBoundaryCond(LieGroup::Twist const & lambdadot_max, LieGroup::Twist const & lambdaddot_max, LieGroup::Twist const & lambdadddot_max)
	{
		_interpolator.setBoundaryCond(lambdadot_max, lambdaddot_max, lambdadddot_max);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the desired trajectory for the time instant
	//! 
	//! \note
	//! This should be called after setTargetTraj(). 
	//!
	//! \param t Desired time
	//! \param Td Desired task transformation or \f$ T_d \f$
	//! \param Vd Desired task twist or \f$ V_d \f$
	//! \param Vdotd Desired task acceleration or \f$ \dot{V}_d \f$
	//  ----------------------------------------------------------
	inline void traj(double t, LieGroup::HTransform & Td, LieGroup::Twist & Vd, LieGroup::Twist & Vdotd)
	{
		LieGroup::Twist lambda_d;
		LieGroup::Twist lambdadot_d;
		LieGroup::Twist lambdaddot_d;

		_interpolator.traj(t, lambda_d, lambdadot_d, lambdaddot_d);

		double alpha;
		double beta;
		double theta_sqr;
		lambda_d.compute_terms(&alpha, &beta, &theta_sqr);

		LieGroup::HTransform exp = lambda_d.exp(alpha, beta, theta_sqr);
		Matrix6d dexp = lambda_d.dexp(alpha, beta, theta_sqr);
		Matrix6d ddt_dexp = lambda_d.ddt_dexp(alpha, beta, theta_sqr, 
			lambda_d.w().dot(lambda_d.v()), lambda_d.w().dot(lambdadot_d.w()), lambda_d.w().dot(lambdadot_d.v()), lambdadot_d.w().dot(lambda_d.v()),
			lambdadot_d);

		Td = _T0.cascade(exp);
		Vd = lambdadot_d.upperBlockTriTransposedMult(dexp);
		Vdotd = lambdaddot_d.upperBlockTriTransposedMult(dexp) + lambdadot_d.upperBlockTriTransposedMult(ddt_dexp);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the period for cyclic update of trajectory
	//!
	//! \param delT period (in sec) (default value = 0)
	//  ----------------------------------------------------------
	inline void setPeriod(double delT)
	{
		_interpolator.setPeriod(delT); 
	}
	
	/// ADDED @20160316
	InterpolatorAlgorithm & alg()  
	{
		return _interpolator;
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! This returns whether the robot reached target.
	//  ----------------------------------------------------------
	inline bool isTargetReached() { return _interpolator.isTargetReached(); }

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Six-dimensional vector interpolator algorithms (of type InterpolatorAlgorithm)
	//  ----------------------------------------------------------
	InterpolatorAlgorithm _interpolator;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Initial task transformation
	//  ----------------------------------------------------------
	LieGroup::HTransform _T0;
};

} // namespace NRMKFoundation
