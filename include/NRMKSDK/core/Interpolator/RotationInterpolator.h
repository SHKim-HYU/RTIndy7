//  ---------------------- Doxygen info ----------------------
//! \file RotationInterpolator.h
//!
//! \brief
//! Header file for the class RotationInterpolator (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a rotation interpolator
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
//!//! \note 
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
//! \class RotationInterpolator
//!
//! \brief
//! This implements a rotation interpolator class.
//!
//! \details
//! Rotational interpolator interpolates three-dimensional orientation task, which is described by 
//! the rotation matrix as the task position, the body angular velocity as the task velocity, and 
//! the body angular acceleration as the task acceleration. Rotation matrices are represented by LieGroup::Rotation, and 
//! angular velocities and accelerations are represented by LieGroup::Vector3D. That is,
//! given a set of initial conditions \f$ R_0 \f$, \f$ \omega_0 \f$, and \f$ \dot{\omega}_0 \f$ 
//! at time \f$ t_0 \f$ and a set of final conditions \f$ R_f \f$, \f$ \omega_f \f$, and \f$ \dot{\omega}_f \f$  
//! at time \f$ t_f \f$, the intermediate trajectory at time \f$ t \f$ consisting of \f$ R_{des}(t) \f$, \f$ \omega_{des}(t) \f$, and \f$ \dot{\omega}_{des}(t) \f$
//! is computed.
//! 
//! Since rotation matrices comprises the Lie group \f$ SO(3) \f$, 
//! the interpolation is done in terms of the corresponding Lie algebra vector (of type LieGroup::Vector3D) \f$ \xi \f$.
//! Since there are many types of interpolator algorithms for three-dimensional vectors (i.e. \f$ \xi \f$),
//! it is passed as a template argument.
//! 
//! \tparam InterpolatorAlgorithm Three-dimensional vector interpolator algorithm type
//!
//! \sa Interpolator
//  ----------------------------------------------------------
template<typename InterpolatorAlgorithm>
class RotationInterpolator
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides Typedefs 
	//  ----------------------------------------------------------
	typedef LieGroup::Rotation PosType; //!< Typedef of the type for position variable
	typedef LieGroup::Vector3D VelType; //!< Typedef of the type for velocity variable
	typedef LieGroup::Vector3D AccType; //!< Typedef of the type for acceleration variable

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! fixes the interpolated trajectory
	//! 
	//! \param R Initial task rotation vector or \f$ R_0 \f$
	//! \param w Initial task angular velocity vector or \f$ \omega_0 \f$
	//! \param wdot Initial task angular acceleration vector or \f$ \dot{\omega}_0 \f$
	//  ----------------------------------------------------------
	void setTraj(LieGroup::Rotation const & R, LieGroup::Vector3D const & w = LieGroup::Vector3D(), LieGroup::Vector3D const & wdot = LieGroup::Vector3D())
	{
		_R0 = R;
		_interpolator.setTraj(LieGroup::Vector3D(), w, wdot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the initial condition of the interpolated trajectory
	//!
	//! \note
	//! This should be called first. 
	//! 
	//! \param t0 Initial time or \f$ t_0 \f$
	//! \param R0 Initial task rotation vector or \f$ R_0 \f$
	//! \param w0 Initial task angular velocity vector or \f$ \omega_0 \f$
	//! \param wdot0 Initial task angular acceleration vector or \f$ \dot{\omega}_0 \f$
	//  ----------------------------------------------------------
	void setInitialTraj(double t0, LieGroup::Rotation const & R0, LieGroup::Vector3D const & w0 = LieGroup::Vector3D(), LieGroup::Vector3D const & wdot0 = LieGroup::Vector3D())
	{
		_R0 = R0;
		_interpolator.setInitialTraj(t0, LieGroup::Vector3D(), w0, wdot0);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the final condition of the interpolated trajectory
	//!
	//! \note
	//! This should be called after setInitialTraj(). 
	//! 
	//! \param tf Final time or \f$ t_f \f$
	//! \param Rf Final task rotation vector or \f$ R_f \f$
	//! \param wf Final task angular velocity vector or \f$ \omega_f \f$
	//! \param wdotf Final task angular acceleration vector or \f$ \dot{\omega}_f \f$
	//  ----------------------------------------------------------
	void setTargetTraj(double tf, LieGroup::Rotation const & Rf, LieGroup::Vector3D const & wf = LieGroup::Vector3D(), LieGroup::Vector3D const & wdotf = LieGroup::Vector3D())
	{
		LieGroup::Rotation R = _R0.icascade(Rf);
		LieGroup::Vector3D xi_f = R.expCoord(); 

		double alpha;
		double beta;
		double theta_sqr;
		xi_f.compute_terms(&alpha, &beta, &theta_sqr);

		Eigen::Matrix3d dexpinv = xi_f.dexp_inv(alpha, beta, theta_sqr);
		LieGroup::Vector3D xidot_f = dexpinv.transpose()*wf;

		Eigen::Matrix3d ddt_dexp = xi_f.ddt_dexp(alpha, beta, theta_sqr, xi_f.dot(xidot_f), xidot_f);
		LieGroup::Vector3D xiddot_f = dexpinv.transpose()*(wdotf - ddt_dexp.transpose()*xidot_f);

		_interpolator.setTargetTraj(tf, xi_f, xidot_f, xiddot_f);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the boundary condition of the interpolated trajectory
	//!
	//! \details
	//! One may specify the allowable maximum velocity, acceleration, and jerk vectors on the Lie algebra trajectory \f$ \xi \f$. 
	//! Only symmetric bounds are allowed. That is, minimum value is the negative of the maximum. 
	//! Depending on the employed scalar interpolator algorithm this may do nothing.
	//!
	//! \note
	//! This may not be called but, if it is to be called, 
	//! it should be called after setFinalTraj() before calling traj(). 
	//! 
	//! \param xidot_max Maximal velocity vector on \f$ \dot{\xi} \f$
	//! \param xiddot_max Maximal acceleration vector on \f$ \ddot{\xi} \f$
	//! \param xidddot_max Maximal jerk vector on \f$ \dddot{\xi} \f$
	//  ----------------------------------------------------------
	void setBoundaryCond(LieGroup::Vector3D const & xidot_max, LieGroup::Vector3D const & xiddot_max, LieGroup::Vector3D const & xidddot_max)
	{
		_interpolator.setBoundaryCond(xidot_max, xiddot_max, xidddot_max);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the desired trajectory for the time instant
	//! 
	//! \note
	//! This should be called after setTargetTraj(). 
	//!
	//! \param t Desired time
	//! \param Rd Desired task position or \f$ R_{des} \f$
	//! \param wd Desired task velocity or \f$ \omega_{des} \f$
	//! \param wdotd Desired task acceleration or \f$ \dot{\omega}_{des} \f$ 
	//  ----------------------------------------------------------
	inline void traj(double t, LieGroup::Rotation & Rd, LieGroup::Vector3D & wd, LieGroup::Vector3D & wdotd)
	{
		LieGroup::Vector3D xi_d;
		LieGroup::Vector3D xidot_d;
		LieGroup::Vector3D xiddot_d;

		_interpolator.traj(t, xi_d, xidot_d, xiddot_d);

		double alpha;
		double beta;
		double theta_sqr;

		xi_d.compute_terms(&alpha, &beta, &theta_sqr);

		LieGroup::Rotation exp = xi_d.exp(alpha, beta, theta_sqr);
		Eigen::Matrix3d dexp = xi_d.dexp(alpha, beta, theta_sqr);
		Eigen::Matrix3d ddt_dexp = xi_d.ddt_dexp(alpha, beta, theta_sqr, xi_d.dot(xidot_d), xidot_d);

		Rd = _R0*exp;
		wd = dexp.transpose()*xidot_d;
		wdotd = dexp.transpose()*xiddot_d + ddt_dexp.transpose()*xidot_d;
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
	bool isTargetReached() { return _interpolator.isTargetReached(); }

	int getCurSeg()
	{
		return _interpolator.getCurSeg();		
	}
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Three-dimensional vector interpolator algorithms (of type InterpolatorAlgorithm)
	//  ----------------------------------------------------------
	InterpolatorAlgorithm _interpolator;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Initial task rotation
	//  ----------------------------------------------------------
	LieGroup::Rotation _R0;

};

} // namespace NRMKFoundation
