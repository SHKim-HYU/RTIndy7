//  ---------------------- Doxygen info ----------------------
//! \file PolynomialAlgorithm.h
//!
//! \brief
//! Header file for the class _PolynomialInterpolator (Internal API of the NRMKFoundation Libraries)
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
//! \note Copyright (C) 2013 Neuromeka Co., Ltd.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#include <list>
#include <limits>
#include <Eigen/Eigen>

#include "Interpolator/InterpolatorConstants.h"

namespace NRMKFoundation
{

namespace Helper
{
template<typename PosType, typename VelType = PosType, typename AccType = VelType>
class TrajectoryData
{
public:
	inline TrajectoryData(PosType const & p, VelType const & v, AccType const & a, double duration = std::numeric_limits<double>::infinity(), 
						int reuseCount = 0, NRMKFoundation::PolynomialInterpolatorMode mode = NRMKFoundation::POLYNOMIAL_INTERPOLATOR_NONE)
		: _t(duration), _p(p), _v(v), _a(a), _mode(mode), _reuse(reuseCount)	// FIXED by THACHDO 20150717
	{
	}

	inline double duration() const { return _t; }
	inline PosType const & position() const { return _p; }
	inline VelType const & velocity() const { return _v; }
	inline AccType const & acceleration() const { return _a; }

	inline NRMKFoundation::PolynomialInterpolatorMode mode() const { return _mode; }
	inline int reuse() { return --_reuse; }

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	double		_t;

	PosType	_p;
	VelType	_v;
	AccType	_a;

	NRMKFoundation::PolynomialInterpolatorMode	_mode;
	int	_reuse;
};

template<typename PosType, typename VelType = PosType, typename AccType = VelType>
class TrajectoryDataList
{
public:
	inline TrajectoryDataList() : _data(), _tf(0), _Ttraj(0)
	{
	}

	inline TrajectoryDataList(PosType const & p, VelType const & v, AccType const & a) : _data(), _p(p), _v(v), _a(a), _tf(0), _Ttraj(0)	// FIXED by THACHDO 20150717
	{
	}

	inline void reset(double t0 = 0)
	{
		_data.clear();

		_tf = t0;

		_Ttraj = 0; 
	}

	inline void init(double t0, PosType const & p, VelType const & v, AccType const & a)
	{
		 _data.clear();
		 
		 _tf = t0;
		 _p = p;
		 _v = v;
		 _a = a; 

		 _Ttraj = 0; 
	}

	inline void add(PosType const & p, VelType const & v, AccType const & a, double duration = std::numeric_limits<double>::infinity(), int reuse = 0, NRMKFoundation::PolynomialInterpolatorMode mode = NRMKFoundation::POLYNOMIAL_INTERPOLATOR_NONE) 
	{
		_Ttraj += duration;

		TrajectoryData<PosType, VelType, AccType> data(p, v, a, duration, reuse, mode);
		_data.push_back(data);
	}

	inline int update(double t)
	{
		if (t >= _tf)
		{
			if (_data.empty())
			{
				return -1;
			}
			else
			{
				_p = _data.front().position();
				_v = _data.front().velocity();
				_a = _data.front().acceleration();

				_tf = t + _data.front().duration(); 

				if (_data.front().reuse() >= 0)
					_data.push_back(_data.front());

				_data.pop_front();

				return 1;
			}
		}
		else
		{
			return 0;
		}
	}

	inline PosType const & position() const { return _p; }
	inline VelType const & velocity() const { return _v; }
	inline AccType const & acceleration() const { return _a; }
	inline AccType const & tf() const { return _tf; }

	inline double duration() const { return _Ttraj; }

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	std::list<TrajectoryData<PosType, VelType, AccType> > _data;

	PosType		_p;
	VelType		_v;
	AccType		_a;

	double	_tf;

	double _Ttraj;
};

template<typename T>
class StepGenerator
{
public:
	inline StepGenerator() : _data(T(0), T(0), T(0)) 
	{
	}

	inline StepGenerator(T const & p) : _data(p, T(0), T(0))
	{
	}

	inline void generate(double t, T & r, T & rdot)
	{		
		_data.update(t);

		r = _data.position();
		rdot = _data.velocity();
	}

	inline void generate(double t, T & r, T & rdot, T & rddot)
	{		
		_data.update(t);

		r = _data.position();
		rdot = _data.velocity();
		rddot = _data.acceleration();
	}

	inline void add(T const & r, double duration = std::numeric_limits<double>::infinity(), int reuse = 0)
	{
		_data.add(r, T(0), T(0), duration, reuse);
	}

	inline double duration() const { return _data.duration(); }


	inline void reset(double t0)
	{
		_data.reset(t0);
	}

	inline void init(double t0, T const & p, T const & v, T const & a)
	{
		_data.init(t0, p, v, a);
	}

private:
	TrajectoryDataList<T> _data;
};

} // namespace NRMKFoundation::Helper


} // namespace NRMKFoundation
