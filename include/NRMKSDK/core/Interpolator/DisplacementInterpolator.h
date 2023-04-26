//  ---------------------- Doxygen info ----------------------
//! \file DisplacementInterpolator.h
//!
//! \brief
//! Header file for the class DisplacementInterpolator (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a displacement interpolator
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
//! \date February 2016
//! 
//! \version 1.9.3
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013-2016 Neuromeka Co., Ltd.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#include <Eigen/Eigen>

#include "LieGroup/LieGroup.h"

#include "InterpolatorConstants.h"

namespace NRMKFoundation
{
namespace internal
{
}

//  ---------------------- Doxygen info ----------------------
//! \class DisplacementInterpolator
//!
//! \brief
//! This implements a linear displacement interpolator class.
//!
//! \details
//! Displacement interpolator interpolates three-dimensional displacement task. 
//! Displacement vectors are represented by LieGroup::Displacement, and 
//! their velocities and accelerations are represented by LieGroup::Vector3D. That is,
//! given a set of initial conditions \f$ r_0 \f$, \f$ \dot{r}_0 \f$, and \f$ \ddot{r}_0 \f$ 
//! at time \f$ t_0 \f$ and a set of final conditions \f$ r_f \f$, \f$ \dot{r}_f \f$, and \f$ \ddot{r}_f \f$  
//! at time \f$ t_f \f$, the intermediate trajectory at time \f$ t \f$ consisting of \f$ r_{des}(t) \f$, \f$ \dot{r}_{des}(t) \f$, and \f$ \ddot{r}_{des}(t) \f$
//! is computed.
//! 
//! \tparam InterpolatorType Scalar component interpolator algorithm type 
//! \tparam MODE Displacement interpolator mode. If you don't specify this, linear trajectory is generated.
//  ----------------------------------------------------------
template<typename InterpolatorType, DisplacementInterpolatorMode MODE = DISPLACEMENT_INTERPOLATOR_LINEAR>
class DisplacementInterpolator
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides Typedefs 
	//  ----------------------------------------------------------
	typedef LieGroup::Displacement PosType; //!< Typedef of the type for position variable
	typedef LieGroup::Vector3D VelType; //!< Typedef of the type for velocity variable
	typedef LieGroup::Vector3D AccType; //!< Typedef of the type for acceleration variable

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! fixes the interpolated trajectory
	//! 
	//! \param r Initial task rotation vector or \f$ r_0 \f$
	//! \param rdot Initial task angular velocity vector or \f$ \dot{r}_0 \f$
	//! \param rddot Initial task angular acceleration vector or \f$ \ddot{r}_0 \f$
	//  ----------------------------------------------------------
	void setTraj(PosType const & r, VelType const & rdot = VelType(), AccType const & rddot = AccType())
	{
		_interpolator.setTraj(r, rdot, rddot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the initial condition of the interpolated trajectory
	//!
	//! \note
	//! This should be called first. 
	//! 
	//! \param t0 Initial time or \f$ t_0 \f$
	//! \param r0 Initial task rotation vector or \f$ r_0 \f$
	//! \param rdot0 Initial task angular velocity vector or \f$ \dot{r}_0 \f$
	//! \param rddot0 Initial task angular acceleration vector or \f$ \ddot{r}_0 \f$
	//  ----------------------------------------------------------
	void setInitialTraj(double t0, PosType const & r0, VelType const & rdot0 = VelType(), AccType const & rddot0 = AccType())
	{
		_interpolator.setInitialTraj(t0, r0, rdot0, rddot0);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the final condition of the interpolated trajectory
	//!
	//! \note
	//! This should be called after setInitialTraj(). 
	//! 
	//! \param tf Final time or \f$ t_f \f$
	//! \param rf Final task rotation vector or \f$ r_f \f$
	//! \param rdotf Final task angular velocity vector or \f$ \dot{r}_f \f$
	//! \param rddotf Final task angular acceleration vector or \f$ \ddot{r}_f \f$
	//  ----------------------------------------------------------
	void setTargetTraj(double tf, PosType const & rf, VelType const & rdotf = VelType(), AccType const & rddotf = AccType())
	{
		_interpolator.setTargetTraj(tf, rf, rdotf, rddotf);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the boundary condition of the interpolated trajectory
	//!
	//! \details
	//! One may specify the allowable maximum velocity, acceleration, and jerk vectors on the displacement trajectory \f$ r \f$. 
	//!
	//! \note
	//! This may not be called but, if it is to be called, 
	//! it should be called after setFinalTraj() before calling traj(). 
	//! 
	//! \param rdot_max Maximal velocity vector on \f$ \dot{r} \f$
	//! \param rddot_max Maximal acceleration vector on \f$ \ddot{r} \f$
	//! \param rdddot_max Maximal jerk vector on \f$ \dddot{r} \f$
	//  ----------------------------------------------------------
	void setBoundaryCond(VelType const & rdot_max, AccType const & rddot_max = AccType(), AccType const & rdddot_max = AccType())
	{
		_interpolator.setBoundaryCond(rdot_max, rddot_max, rdddot_max);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the desired trajectory for the time instant
	//! 
	//! \note
	//! This should be called after setTargetTraj(). 
	//!
	//! \param t Desired time
	//! \param rd Desired task position or \f$ r_{des} \f$
	//! \param rdotd Desired task velocity or \f$ \dot{r}_{des} \f$
	//! \param rddotd Desired task acceleration or \f$ \ddot{r}_{des} \f$ 
	//  ----------------------------------------------------------
	inline void traj(double t, PosType & rd, VelType & rdotd, AccType & rddotd)
	{
		_interpolator.traj(t, rd, rdotd, rddotd);
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
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! This returns whether the robot reached target.
	//  ----------------------------------------------------------
	inline bool isTargetReached() { return _interpolator.isTargetReached(); }

private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Three-dimensional vector interpolator algorithms 
	//  ----------------------------------------------------------
	NRMKFoundation::VectorInterpolator<3, InterpolatorType> _interpolator;
};

//  ---------------------- Doxygen info ----------------------
//! \class DisplacementInterpolator 
//!
//! \brief
//! This implements a circular arc displacement interpolator class by partial specialization.
//! You should define the type as NRMKFoundation::DisplacementInterpolator<InterpolatorType, DISPLACEMENT_INTERPOLATOR_CIRCULAR>.
//!
//! \tparam InterpolatorType Scalar component interpolator algorithm type 
//  ----------------------------------------------------------
template<typename InterpolatorType>
class DisplacementInterpolator<InterpolatorType, DISPLACEMENT_INTERPOLATOR_CIRCULAR>
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides Typedefs 
	//  ----------------------------------------------------------
	typedef LieGroup::Displacement PosType; //!< Typedef of the type for position variable
	typedef LieGroup::Vector3D VelType; //!< Typedef of the type for velocity variable
	typedef LieGroup::Vector3D AccType; //!< Typedef of the type for acceleration variable

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the initial condition of the interpolated trajectory
	//!
	//! \note
	//! This should be called first. 
	//! 
	//! \param t0 Initial time or \f$ t_0 \f$
	//! \param r0 Initial task rotation vector or \f$ r_0 \f$
	//! \param rdot0 Initial task angular velocity vector or \f$ \dot{r}_0 \f$
	//! \param rddot0 Initial task angular acceleration vector or \f$ \ddot{r}_0 \f$
	//  ----------------------------------------------------------
	void setInitialTraj(double t0, PosType const & r0, VelType const & rdot0 = VelType(), AccType const & rddot0 = AccType())
	{
		_t0 = t0;
		_r0 = r0;
		_rdot0 = rdot0;
		_rddot0 = rddot0;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the final condition of the interpolated trajectory
	//!
	//! \note
	//! This should be called after setInitialTraj(). 
	//! 
	//! \param tf Final time or \f$ t_f \f$
	//! \param rf Final task rotation vector or \f$ r_f \f$
	//! \param rdotf Final task angular velocity vector or \f$ \dot{r}_f \f$
	//! \param rddotf Final task angular acceleration vector or \f$ \ddot{r}_f \f$
	//  ----------------------------------------------------------
	void setTargetTraj(double tf, PosType const & rf, VelType const & rdotf = VelType(), AccType const & rddotf = AccType())
	{
		_tf = tf;
		_rf = rf;
		_rdotf = rdotf;
		_rddotf = rddotf;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the center of the circular arc
	//!
	//! \note
	//! This should be called after setTargetTraj(). 
	//! 
	//! \param o Center position
	//! \return the flag whether the axis has been fixed
	//  ----------------------------------------------------------
	bool setCenter(PosType const & o)
	{
		_TC.r() = o;	
		
		LieGroup::Displacement X0 = _r0 - _TC.r();
		LieGroup::Displacement X1 = _rf - _TC.r();
		
		return _determine(X0.cross(X1));
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the axis of the circular arc
	//!
	//! \note
	//! This should be called after setCenter(). 
	//! 
	//! \param z Axis direction
	//  ----------------------------------------------------------
	void setAxis(LieGroup::Vector3D const & z)
	{
		_determine(z);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the algorithm to trace the larger arc
	//!
	//! \note
	//! This should be called after setCenter() and setAxis(). 
	//  ----------------------------------------------------------
	void traceLargeArc()
	{
		if (_theta1 >= 0)
			_theta1 -= 2*M_PI;
		else 
			_theta1 += 2*M_PI;

		_th_interpolator.setInitialTraj(_t0, 0);
		_th_interpolator.setTargetTraj(_tf, _theta1);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the boundary condition of the interpolated trajectory
	//!
	//! \details
	//! One may specify the allowable maximum velocity, acceleration, and jerk vectors on the displacement trajectory \f$ r \f$. 
	//!
	//! \note
	//! This may not be called but, if it is to be called, 
	//! it should be called after setFinalTraj() before calling traj(). 
	//! 
	//! \param rdot_max Maximal velocity vector on \f$ \dot{r} \f$
	//! \param rddot_max Maximal acceleration vector on \f$ \ddot{r} \f$
	//! \param rdddot_max Maximal jerk vector on \f$ \dddot{r} \f$
	//  ----------------------------------------------------------
	void setBoundaryCond(VelType const & rdot_max, AccType const & rddot_max = AccType(), AccType const & rdddot_max = AccType())
	{
		//_interpolator.setBoundaryCond(rdot_max, rddot_max, rdddot_max);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the desired trajectory for the time instant
	//! 
	//! \note
	//! This should be called after setTargetTraj(). 
	//!
	//! \param t Desired time
	//! \param rd Desired task position or \f$ r_{des} \f$
	//! \param rdotd Desired task velocity or \f$ \dot{r}_{des} \f$
	//! \param rddotd Desired task acceleration or \f$ \ddot{r}_{des} \f$ 
	//  ----------------------------------------------------------
	inline void traj(double t, PosType & rd, VelType & rdotd, AccType & rddotd)
	{
		double r, rdot, rddot;
		_r_interpolator.traj(t, r, rdot, rddot);

		double th, thdot, thddot;
		_th_interpolator.traj(t, th, thdot, thddot);

		double z, zdot, zddot;
		_z_interpolator.traj(t, z, zdot, zddot);

		double s = ::sin(th);
		double c = ::cos(th);

		double rs = r*s;
		double rc = r*c;
		double rdots = rdot*s;
		double rdotc = rdot*c;

		double thdot2 = thdot*thdot;

		rd.set(rc, rs, z);
		rdotd.set(rdotc - rs*thdot, rdots + rc*thdot, zdot);

		rddotd.set(	rddot*c - 2*rdots*thdot - rc*thdot2 - rs*thddot, 
					rddot*s + 2*rdotc*thdot - rs*thdot2 + rc*thddot, 
					zddot);

		rd.transform(_TC);
		rdotd.transform(_TC);
		rddotd.transform(_TC);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the period for cyclic update of trajectory
	//!
	//! \param delT period (in sec) (default value = 0)
	//  ----------------------------------------------------------
	inline void setPeriod(double delT)
	{
		_r_interpolator.setPeriod(delT);
		_th_interpolator.setPeriod(delT);
		_z_interpolator.setPeriod(delT);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! This returns whether the robot reached target.
	//  ----------------------------------------------------------
	bool isTargetReached() { return (_r_interpolator.isTargetReached() && _th_interpolator.isTargetReached() && _z_interpolator.isTargetReached()); }
	
private:
	inline bool _determine(LieGroup::Vector3D const & z)
	{
		if (z.norm() == 0)
			return false;

		_TC.z() = z;
		_TC.z().normalize();

		LieGroup::Displacement X0 = _r0 - _TC.r();
		LieGroup::Displacement X1 = _rf - _TC.r();

		double r0 = X0.norm();
		double r1 = X1.norm();

		_r_interpolator.setInitialTraj(_t0, r0);
		_r_interpolator.setTargetTraj(_tf, r1);

		_TC.x() = X0/r0;
		_TC.y() = _TC.z().cross(_TC.x());
		
		_theta1 = ::atan2(_TC.y().dot(X1), _TC.x().dot(X1)); 

		_th_interpolator.setInitialTraj(_t0, 0);
		_th_interpolator.setTargetTraj(_tf, _theta1);

		double z1 = _TC.z().dot(X1);
		_z_interpolator.setInitialTraj(_t0, 0);
		_z_interpolator.setTargetTraj(_tf, z1);

		return true;
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	
private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Interpolator algorithms (of type InterpolatorType)
	//  ----------------------------------------------------------
	InterpolatorType _r_interpolator;
	InterpolatorType _th_interpolator;
	InterpolatorType _z_interpolator;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Circular arc angle
	//  ----------------------------------------------------------
	double _theta1;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Homogeneous transformation matrix to the arc plane
	//  ----------------------------------------------------------
	LieGroup::HTransform _TC;

	double _t0;
	PosType _r0;
	VelType _rdot0;
	AccType _rddot0;

	double _tf;
	PosType _rf;
	VelType _rdotf;
	AccType _rddotf;
};

} // namespace NRMKFoundation
