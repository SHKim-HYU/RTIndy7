//  ---------------------- Doxygen info ----------------------
//! \file InterpolatorConstants.h
//!
//! \brief
//! Header file for the constants used in Interpolator module (Internal API of the NRMKFoundation Libraries)
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
//! \date April 2013
//! 
//! \version 0.1
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013 Neuromeka Co., Ltd.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#include <limits>

namespace NRMKFoundation
{
namespace internal
{
	/// FIXME @20160229
	static const double _traj_time_undefined = std::numeric_limits<double>::infinity();
}

//  ---------------------- Doxygen info ----------------------
//! \enum Polynomial interpolator mode
//! 
//! \brief Provides polynomial interpolator mode
//  ----------------------------------------------------------
enum PolynomialInterpolatorMode
{
	//! \details constant interpolator
	POLYNOMIAL_INTERPOLATOR_NONE = 0,
	//! \details linear interpolator
	POLYNOMIAL_INTERPOLATOR_LINEAR,
	//! \details two-segments parabolic interpolator
	POLYNOMIAL_INTERPOLATOR_PARABOLIC,
	//! \details cubic interpolator
	POLYNOMIAL_INTERPOLATOR_CUBIC,
	//! \details quintic interpolator
	POLYNOMIAL_INTERPOLATOR_QUINTIC,
	//! \details septic interpolator
	POLYNOMIAL_INTERPOLATOR_SEPTIC,
	//! \details linear interpolator with parabolic blends
	POLYNOMIAL_INTERPOLATOR_LINEAR_PARABOLIC_BLEND,
	POLYNOMIAL_TRAPEZIODAL = POLYNOMIAL_INTERPOLATOR_LINEAR_PARABOLIC_BLEND,
	//! \details linear interpolator with parabolic blends (synchronized)
	POLYNOMIAL_TRAPEZIODAL_SYNC,
	//! \details double-S trajectory
	POLYNOMIAL_INTERPOLATOR_DOUBLE_S,
	//! \details Cspline trajectory
	POLYNOMIAL_INTERPOLATOR_CSPLINE,
	//! \details Bspline trajectory
	POLYNOMIAL_INTERPOLATOR_BSPLINE,
};

//  ---------------------- Doxygen info ----------------------
//! \enum CSpline interpolator mode
//! 
//! \brief Provides CSpline interpolator mode
//  ----------------------------------------------------------
enum CSplineMode
{
	//! \details initial and final accelerations are zero. 
	//!  This is the default mode.
	NATURAL_BOUNDARY_CONDITIONS = 0,
	//! \details initial acceleration is equal to the acceleration at the second point
	//!	last acceleration is equal to the acceleration at second-to-last point			 
	PARABOLIC_RUNOUT,
	//! \details initial and final velocities are zero. 
	ZERO_SLOPE, 
	//! \details initial and final velocities are assigned. 
	ASSIGNED_BOUNDARY_VELOCITIES,
	//! \details initial and final accelerations are assigned. 
	//!  This is not implemented yet.
	ASSIGNED_BOUNDARY_ACCELERATIONS,
	//! \details initial and final velocities and accelerations are equal to each other 
	//!  to generate a periodic trajectory. 
	CLOSED_LOOP
};

//  ---------------------- Doxygen info ----------------------
//! \enum Displacement interpolator mode
//! 
//! \brief Provides displacement interpolator mode
//  ----------------------------------------------------------
enum DisplacementInterpolatorMode
{
	//! \details linear interpolator
	DISPLACEMENT_INTERPOLATOR_LINEAR = 0,
	//! \details circular arc interpolator
	DISPLACEMENT_INTERPOLATOR_CIRCULAR,
};

enum 
{
	//! \details The number of data is dynamically changing
	DynamicSize = -1,
};

} // namespace NRMKFoundation
