//  ---------------------- Doxygen info ----------------------
//! \file PerformanceMeasure.h
//!
//! \brief
//! Header file for the class PerformanceMeasure (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a performance measure for redundancy resolution
//! to be used for the interface of NRMKFoundation library
//!
//! THIS IS EXPERIMENTAL, WHICH IS SUBJECT TO CHANGE!!!!
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
//! \date April 2016
//! 
//! \version 1.9.4
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note 
//!  - v1.9.4(20160406) : initial developemnt
//!           
//!
//!	 Copyright (C) 2013-2016 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#include <Eigen/Eigen>

#include "NRMKCommon.h"

namespace NRMKFoundation
{
namespace internal
{
}

template<int DIM, typename PerformanceMeasureType>
class PerformanceMeasure
{	
public:
	typedef Eigen::Matrix<double, DIM, 1> VecType;
	typedef Eigen::Matrix<double, DIM, DIM> MatType;
	
public:
	//! \returns Reference to the derived object 
	inline PerformanceMeasureType& derived() { return *static_cast<PerformanceMeasureType*>(this); }
	//! \returns Reference to the derived object 
	inline const PerformanceMeasureType& derived() const { return *static_cast<const PerformanceMeasureType*>(this); }

public:
	double getGradient(VecType const &q, VecType & grad) const
	{
		return derived().getGradient(q, grad); 
	}

	double getGradient(VecType const &q, VecType & grad, MatType & hess) const
	{
		return derived().getGradient(q, grad, hess); 
	}
};

template<int DIM>
class JointRangeAvailability : public PerformanceMeasure<DIM, JointRangeAvailability<DIM> >
{
public:
	typedef typename PerformanceMeasure<DIM, JointRangeAvailability<DIM> >::VecType VecType;
	typedef typename PerformanceMeasure<DIM, JointRangeAvailability<DIM> >::MatType MatType;

public:
	JointRangeAvailability() 
	{
		//_kappa = 1;
		_q0.setZero();
		_W.setConstant(1);
	}

	//void setGain(double kappa) 
	//{
	//	_kappa = kappa; 
	//}

	void setWeight(VecType const & W) 
	{
		_W = W;
	}

	void setNeutralPos(VecType const & q0)
	{
		_q0 = q0;
	}

	double getGradient(VecType const &q, VecType & grad) const
	{
		VecType dq = q - _q0;
		grad = dq.cwiseProduct(_W);

		return 0.5*dq.dot(grad);
	}

	double getGradient(VecType const &q, VecType & grad, MatType & hess) const
	{
		double value = gradGain(q, qrad);

		hess.setZero();
		hess.diag() = _W;

		return value; 
	}

private:
	//double _kappa; 

	VecType _W; 
	VecType _q0; 
};

} // namespace NRMKFoundation
