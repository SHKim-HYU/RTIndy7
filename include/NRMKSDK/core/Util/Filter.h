//  ---------------------- Doxygen info ----------------------
//! \file Filter.h
//!
//! \brief
//! Header file for the class Filter (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a digital filter class
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
//! \date February 2014
//! 
//! \version 1.6
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013-2014 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#include "Eigen/Eigen"

namespace NRMKFoundation
{

namespace internal
{
	template<typename T>
	struct HasScalarAssignmentOp
	{
		static const bool value = false; //!< value is false for general joint types
	};

	template<>
	struct HasScalarAssignmentOp<double>
	{
		static const bool value = true; //!< value is false for general joint types
	};

// 	template<>
// 	struct HasScalarAssignmentOp<float>
// 	{
// 		static const bool value = true; //!< value is false for general joint types
// 	};

	template <bool hasScalarAssignmentOp>
	struct scalar_assignment_algorithm
	{
		template<typename T>
		inline static void SetConstant(T & lhs, double rhs)
		{
			lhs = rhs;
		}

		template<typename T>
		inline static void SetZero(T & lhs)
		{
			SetConstant(lhs,  0);
		}
	};

	template <>
	struct scalar_assignment_algorithm<false>
	{
		template<typename T>
		inline static void SetConstant(T & lhs, double rhs)
		{
			lhs.setConstant(rhs);
		}

		template<typename T>
		inline static void SetZero(T & lhs)
		{
			lhs.setZero();
		}
	};
}

/**
 *Filter class. 
 *@brief Filter Filters below are digital equivalents using Tustin's method
 */
template<int _ORDER = 0, typename _Type = double>
class Filter
{
public:
	enum
	{
		ORDER = _ORDER,
	};

	typedef Eigen::Matrix<double, ORDER + 1, 1> CoeffVec;
	typedef _Type Type;

public:
	Filter() 
		: _a(CoeffVec::Zero())	// denominator polynomial coefficients (in 1/z), i.e. _a[0] + _a[1]*(1/z) + _a[2]*(1/z^2) + ...
		, _b(CoeffVec::Zero())   // numerator polynomial coefficients (in 1/z)
	{
		_a[0] = 1;	
		_b[0] = 1;

		for (int i = 0; i < ORDER + 1; i++)
		{
			internal::scalar_assignment_algorithm<internal::HasScalarAssignmentOp<Type>::value>::SetZero(_raw[i]); 
			internal::scalar_assignment_algorithm<internal::HasScalarAssignmentOp<Type>::value>::SetZero(_filtered[i]); 
		}
	}

	void filter(Type const & raw, Type & filtered)
	{
		_raw[0] = raw;  // reserve for next iteration.
		_filtered[0] = _raw[0] * _b[0];

		for (int i = 1; i < ORDER + 1; i++) // Note that _a[0] is not used, since it is normalized.
			_filtered[0] +=  _raw[i] * _b[i] - _filtered[i] * _a[i];

		for (int i = 0; i < ORDER; i++)	 // reserve for next iteration.
		{ 
			_filtered[i + 1] = _filtered[i];
			_raw[i + 1] = _raw[i];
		}

		filtered = _filtered[0];
	}

// 	void reset(Type const & raw, Type const & filtered = Type::Zero())
// 	{
// 		for (int i = 1; i < ORDER + 1; i++)
// 		{ 
// 			_raw[i] = raw; 
// 			_filtered[i] = filtered; 
// 		}
// 	}
	
	Type const & raw(int k) const { return _raw[k]; }
	Type const & filtered(int k) const { return _filtered[k]; }

	Type & raw(int k) { return _raw[k]; }
	Type & filtered(int k) { return _filtered[k]; }

	void setTransferFunction(CoeffVec const & den, CoeffVec const & num)
	{
		_a = den;
		_b = num;
	}

	CoeffVec const & a() const { return _a; }
	CoeffVec const & b() const { return _b; }

protected:  
	CoeffVec	_a;
	CoeffVec	_b;

	Type _filtered[ORDER + 1];
	Type _raw[ORDER + 1];
};

/**
 *FilterLowPass class. 
 *@brief FilterLowPass low-pass filter implementing \f$ \frac{w}{s + w} \f$
 */
template<typename _Type = double>
class FilterLowPass : public Filter<1, _Type>
{
public:
	typedef Filter<1, _Type> FilterType;

	typedef typename FilterType::CoeffVec CoeffVec;
	typedef typename FilterType::Type Type;

public:
	void setParam(double sampFreq, double cutOffFreq)
	{
		double T = 1.0/sampFreq;
		double w = 2*M_PI*cutOffFreq;
		w = (2/T)*::tan(w*T/2);  // warping to match cut-off frequency

		double p = (T*w/2) + 1;
		double m = (T*w/2) - 1;

		_b[0] =  (T*w/2)/p;
		_b[1] =  _b[0];
		_a[0] =  1;
		_a[1] =  m/p;
	}

private:
	using FilterType::_a;
	using FilterType::_b;
};

/**
 *FilterDerivative class. 
 *@brief FilterDerivative derivative implementing \f$ \frac{sw}{s+w} \f$
 */
template<typename _Type = double>
class FilterDerivative : public Filter<1, _Type>
{
public:
	typedef Filter<1, _Type> FilterType;

	typedef typename FilterType::CoeffVec CoeffVec;
	typedef typename FilterType::Type Type;

public:
	void setParam(double sampFreq, double cutOffFreq)
	{
		double T = 1.0/sampFreq;
		double w = 2*M_PI*cutOffFreq;
		w = (2/T)*::tan(w*T/2);   // warping to match cut-off frequency

		_b[0] =  2*w/(w*T + 2);
		_b[1] = -_b[0];
		_a[0] =  1;
		_a[1] =  (w*T-2)/(w*T+2);
	}

private:
	using FilterType::_a;
	using FilterType::_b;
};

/**
 *FilterLeadLag class. 
 *@brief Filter lead-lag filter implementing  \frac{(w_2w_3/w_1)(s+w_1)}{(s+w_2)(s+w_3)} \f$
 */
template<typename _Type = double>
class FilterLeadLag : public Filter<2, _Type>
{
public:
	typedef Filter<2, _Type> FilterType;

	typedef typename FilterType::CoeffVec CoeffVec;
	typedef typename FilterType::Type Type;

public:
	void setParam(double sampFreq, double f1, double f2, double f3)
	{
		double T = 1.0/sampFreq;

		double w1 = 2*M_PI*f1;
		w1 = (2/T)*::tan(w1*T/2);  
		double w2 = 2*M_PI*f2;
		w2 = (2/T)*::tan(w2*T/2);  
		double w3 = 2*M_PI*f3;
		w3 = (2/T)*::tan(w3*T/2);  

		double g0 = (w2*w3/w1); // unity DC gain
		double p2 = T*w2/2 + 1;
		double m2 = T*w2/2 - 1;
		double p3 = T*w3/2 + 1;
		double m3 = T*w3/2 - 1;
		double pp = p2*p3;
		double  c = g0*T/2/pp;

		_b[0] =  c*(w1+1);
		_b[1] =  c*w1;
		_b[2] = -c;
		_a[0] =  1;
		_a[1] =  (m2*p3+p2*m3)/pp;
		_a[2] =  m2*m3/pp;
	}

private:
	using FilterType::_a;
	using FilterType::_b;
};

template<int _SIZE, typename _Type = double>
class FilterMovingAverage : public Filter<_SIZE - 1, _Type>
{
public:
	typedef Filter<_SIZE - 1, _Type> FilterType;

	typedef typename FilterType::CoeffVec CoeffVec;
	typedef typename FilterType::Type Type;

public:
	FilterMovingAverage() :  FilterType()
	{
		_b[0] = 1.0/_SIZE;

		for (int i = 1; i <= FilterType::ORDER; i++)
		{
			_a[i] = 0;
			_b[i] = _b[0];
		}
	}

private:
	using FilterType::_a;
	using FilterType::_b;
};

// template<int _DIM, int _SIZE = 1>
// class SimpleMovingAverage
// {
// public:
// 	enum
// 	{
// 		DIM = _DIM,
// 		SIZE = _SIZE,
// 	};
// 
// 	typedef Eigen::Matrix<double, DIM, 1> Type;
// 
// public:
// 	SimpleMovingAverage(int size = 1) 
// 		: _count(0), _size(size), _tail(0), _filtered(Type::Zero())
// 	{
// 		for (int i = 0; i < SIZE; i++)
// 		{
// 			_raw[i].setZero();
// 		}
// 	}
// 
// 	void filter(Type const & raw, Type & filtered)
// 	{
// 		if (_count < size)
// 		{
// 			_raw[_count++] = raw;
// 			_filtered = (_filtered*(_count - 1) + raw)/_count;
// 		}
// 		else
// 		{
// 			Type tail = _raw[tail];
// 			_raw[tail++] = raw;
// 			_tail %= SIZE;
// 
// 			_filtered += (raw - _tail)/_size;
// 		}
// 
// 		filtered = _filtered;
// 	}
// 
// private:
// 	int _count;
// 	int _tail;
// 	int _size;
// 
// 	Type _filtered;
// 	Type _raw[SIZE];
// };

// Discrete Derivative
template<typename _Type = double>
class FilterDiscreteDerivative : public Filter<1, _Type>
{
public:
	typedef Filter<1, _Type> FilterType;

	typedef typename FilterType::CoeffVec CoeffVec;
	typedef typename FilterType::Type Type;

public:
	void setParam(double Ts, double K = 1)
	{
		_b[0] =  K/Ts;
		_b[1] = -_b[0];
		_a[0] =  1;
		_a[1] =  0;
	}

private:
	using FilterType::_a;
	using FilterType::_b;
};

// Discrete-Time Integral
template<typename _Type = double>
class FilterDiscreteIntegral :public Filter<1, _Type>
{
public:
	typedef Filter<1, _Type> FilterType;

	typedef typename FilterType::CoeffVec CoeffVec;
	typedef typename FilterType::Type Type;

public:
	void setParam(double Ts, double K = 1)
	{
		_b[0] = 0; 
		_b[1] =  K*Ts;
		_a[0] = 1;
		_a[1] =  -1;
	}

private:
	using FilterType::_a;
	using FilterType::_b;
};

template<typename _Type>
class Saturation
{
public:
	typedef _Type Type;

	enum
	{
		DIM = _Type::SizeAtCompileTime,
	};

public:
	Saturation()
	{
		internal::scalar_assignment_algorithm<internal::HasScalarAssignmentOp<Type>::value>::SetConstant(_ulimit, 0.5); 
		internal::scalar_assignment_algorithm<internal::HasScalarAssignmentOp<Type>::value>::SetConstant(_llimit, -0.5); 
	}

	void setUpperLimit(Type const & limit)
	{
		_ulimit = limit;
	}

	void setLowerLimit(Type const & limit)
	{
		_llimit = limit;
	}

	void setUpperLimit(double limit)
	{
		internal::scalar_assignment_algorithm<internal::HasScalarAssignmentOp<Type>::value>::SetConstant(_ulimit, limit); 
	}

	void setLowerLimit(double limit)
	{
		internal::scalar_assignment_algorithm<internal::HasScalarAssignmentOp<Type>::value>::SetConstant(_llimit, limit); 
	}

	void apply(Type & raw)
	{
		for (int i = 0; i < DIM; i++)
		{
			if (raw[i] < _llimit[i])
				raw[i] = _llimit[i];
			else if (raw[i] > _ulimit[i])
				raw[i] = _ulimit[i];
		}		
	}

private:
	Type _ulimit;
	Type _llimit;
};

template<>
class Saturation<double>
{
public:
	Saturation()	: _ulimit(0.5), _llimit(-0.5)
	{
	}

	void setUpperLimit(double limit)
	{
		_ulimit = limit;
	}

	void setLowerLimit(double limit)
	{
		_llimit = limit;
	}

	void apply(double & raw)
	{
		//raw.derived().resize(DIM);
		//raw = (raw.derived().cwiseMin(_ulimit)).cwiseMax(_llimit); 
		if (raw < _llimit)
			raw = _llimit;
		else if (raw > _ulimit)
			raw = _ulimit;
	}

private:
	double _ulimit;
	double _llimit;
};

template<typename _Type>
class DeadZone
{
public:
	typedef _Type Type;

	enum
	{
		DIM = _Type::SizeAtCompileTime,
	};

public:
	DeadZone(Type const & upper_limit = Type::Constant(0.5), Type const & lower_limit = Type::Constant(-0.5))
	{
		internal::scalar_assignment_algorithm<internal::HasScalarAssignmentOp<Type>::value>::SetConstant(_ulimit, 0.5); 
		internal::scalar_assignment_algorithm<internal::HasScalarAssignmentOp<Type>::value>::SetConstant(_llimit, -0.5); 
	}

	void setUpperLimit(Type const & limit)
	{
		_ulimit = limit;
	}

	void setLowerLimit(Type const & limit)
	{
		_llimit = limit;
	}

	void setUpperLimit(double limit)
	{
		internal::scalar_assignment_algorithm<internal::HasScalarAssignmentOp<Type>::value>::SetConstant(_ulimit, limit); 
	}

	void setLowerLimit(double limit)
	{
		internal::scalar_assignment_algorithm<internal::HasScalarAssignmentOp<Type>::value>::SetConstant(_llimit, limit); 
	}

	void apply(Type & raw)
	{
		for (int i = 0; i < DIM; i++)
		{
			if (raw[i] < _llimit[i])
				raw[i] -= _llimit[i];
			else if (raw[i] <= _ulimit[i])
				raw[i] = 0.0;
			else 
				raw[i] -= _ulimit[i];
		}		
	}

private:
	Type _ulimit;
	Type _llimit;
};

template<>
class DeadZone<double>
{
public:
	DeadZone()	: _ulimit(0.5), _llimit(-0.5)
	{
	}

	void setUpperLimit(double limit)
	{
		_ulimit = limit;
	}

	void setLowerLimit(double limit)
	{
		_llimit = limit;
	}

	void apply(double & raw)
	{
		//raw.derived().resize(DIM);
		if (raw < _llimit)
			raw -= _llimit;
		else if (raw <= _ulimit)
			raw = 0.0;
		else 
			raw -= _ulimit;
	}

private:
	double _ulimit;
	double _llimit;
};

}