//  ---------------------- Doxygen info ----------------------
//! \file NRMKCommon.h
//!
//! \brief
//! Header file for the common type definitions  (API of the NRMKFoundation Libraries)
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
//! \date March 2016
//! 
//! \version 1.9.4
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note 
//!	 - v1.9.4(20160312) : added class IndexSet
//!
//! Copyright (C) 2013-2016 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------
#pragma once

#ifndef _NRMKCOMMON_H_
#define _NRMKCOMMON_H_

namespace NRMKFoundation
{
namespace internal
{
/// ADDED @20160311
//  ---------------------- Doxygen info ----------------------
//! \class IndexSet
//!
//! \brief
//! This implements a index set class.
//!
//! \tparam DIM Dimension
//! \tparam NUM_KINDS Number of component sets
//  ----------------------------------------------------------
template<int DIM, int NUM_KINDS = 2> 
class IndexSet 
{
public:
	typedef Eigen::Matrix<int, DIM, 1> Type; 

	typedef Eigen::Matrix<double, DIM, 1> VecType;
	typedef Eigen::Matrix<int, Eigen::Dynamic, 1, Eigen::ColMajor, DIM, 1> IndexVecType;

	//typedef Eigen::Matrix<double, DIM, DIM> MatType;

	//typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, DIM, 1> VarVecType;
	// typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, DIM, DIM> VarMatType;
		
public:
	inline IndexSet() : _size(0)
	{
		_set.setZero();	// '0' means that the index does not belong to the set
	}

	//! \brief adds the index to the set
	inline void addIndex(int index, int value = 1)
	{
		if (_set[index] == 0) 
		{
			_set[index] = value;			
			_size++;
		}
	}

	//! \brief removes the index from the set
	inline void removeIndex(int index)
	{
		if (_set[index] == 1) 
		{
			_set[index] = 0;			
			_size--;
		}
	}

	//! \brief adds the index to the set
	inline void addAll(int val = 1)
	{
		_size = DIM; 
		_set.setConstant(val);
	}

	//! \brief resets the index set empty
	inline void reset()
	{
		_size = 0;
		_set.setZero();
	}

	//! \brief returns the size of the index set
	inline unsigned int size() const 
	{
		return _size;
	}

	//! \brief returns whether the index belongs to the index set
	inline bool belong(int index, int value = 1) const 
	{
		return _set[index] == value;
	}
	
	//! \brief partitions the whole indices into two index sets
	inline void partitionIndexVector(IndexVecType & in, IndexVecType & out) const
	{
		in.resize(_size);
		out.resize(DIM - _size);

		for (int k = 0, i = 0, j = 0; k < DIM; k++)
		{
			if (belong(k))
				in(i++) = k;
			else
				out(j++) = k;				
		}
	}		

	/*
	template <typename Derived, typename Derived1, typename Derived2>
	inline void colPartition(Derived const & m, Derived1 & min, Derived2 & mout) const
	{
		for (int k = 0, in = 0, out = 0; k < DIM; k++)
		{
			if (belong(k))
			{
				min.col(in++) = m.col(k);
			}
			else
			{
				mout.col(out++) = m.col(k);
			}
		}
	}

	template <typename Derived, typename Derived1, typename Derived2>
	inline void rowPartition(Derived const & m, Derived1 & min, Derived2 & mout) const
	{
		for (int k = 0, in = 0, out = 0; k < DIM; k++)
		{
			if (belong(k))
			{
				min.row(in++) = m.row(k);
			}
			else
			{
				mout.row(out++) = m.row(k);
			}
		}
	}
	
	inline void getIndexVector(DerivedVecType & in) const
	{
		in.resize(_size);
			
		for (int k = 0, i = 0; k < DIM; k++)
		{
			if (belong(k))
				in(i++) = k;				
		}
	}

	inline DerivedVecType getComplementIndexVector(DerivedVecType & out) const
	{
		out.resize(DIM - _size);

		for (int k = 0, i = 0; k < DIM; k++)
		{
			if (!belong(k))
				out(i++) = k;				
		}
	}	
	*/
		
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//! \brief the whole index vector (of dimension DIM)
	Type _set; 

	//! \brief the size of the index set
	unsigned int _size;
};

}

enum KinematicUpdateMode 
{
	POS_AND_VEL = 0,
	POS_ONLY,
	VEL_ONLY,
};

template <typename T>
static int signum(T x)
{
	return (T(0) < x) - (x < T(0));
}

/*
template <typename T> inline 
static int signum(T x, std::false_type is_signed) 
{
	return T(0) < x;
}

template <typename T> inline 
static int signum(T x, std::true_type is_signed) 
{
	return (T(0) < x) - (x < T(0));
}

template <typename T> inline 
static int signum(T x) 
{
	return signum(x, std::is_signed<T>());
}
*/

} // namespace NRMKFoundation

#endif
