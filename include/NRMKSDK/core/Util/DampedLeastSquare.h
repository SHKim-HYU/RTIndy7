//  ---------------------- Doxygen info ----------------------
//! \file DampedLeastSqure.h
//!
//! \brief
//! Header file for the class DynamicAnalysis (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements an articulated multibody system class
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
//! \date December 2013
//! 
//! \version 1.5
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#include "Eigen/Eigen"

namespace NRMKFoundation
{
//  ---------------------- Doxygen info ----------------------
//! \class DLSSolver
//!
//! \brief
//! This class provides a damped-least square solver 
//! which minimized \f$ \| \dot{p} - J \dot{q} \| \f$
//! subject to \f$ | v_k | <= Q_k \f$
//! where \f$ v_k = V^T \dot{q} \f$ for the right singular matrix \f$ V \f$
//  ----------------------------------------------------------
template<typename _MatrixType>	
class DLSSolver
{
public:
	typedef _MatrixType MatrixType;
	typedef typename MatrixType::Scalar Scalar;
	typedef typename Eigen::NumTraits<typename MatrixType::Scalar>::Real RealScalar;
	typedef typename MatrixType::Index Index;
	enum {
		RowsAtCompileTime = MatrixType::RowsAtCompileTime,
		ColsAtCompileTime = MatrixType::ColsAtCompileTime,
		DiagSizeAtCompileTime = EIGEN_SIZE_MIN_PREFER_DYNAMIC(RowsAtCompileTime,ColsAtCompileTime),
		MaxRowsAtCompileTime = MatrixType::MaxRowsAtCompileTime,
		MaxColsAtCompileTime = MatrixType::MaxColsAtCompileTime,
		MaxDiagSizeAtCompileTime = EIGEN_SIZE_MIN_PREFER_FIXED(MaxRowsAtCompileTime,MaxColsAtCompileTime),
		MatrixOptions = MatrixType::Options,
		DIM = RowsAtCompileTime,
		DOF = ColsAtCompileTime
	};

	typedef Eigen::Matrix<Scalar, DIM, 1, Eigen::ColMajor, MaxRowsAtCompileTime, 1> TaskVec;
	typedef Eigen::Matrix<Scalar, DOF, 1, Eigen::ColMajor, MaxColsAtCompileTime, 1> JointVec;

	DLSSolver(MatrixType const & J) 
		: _svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV)
	{
		_rank = _svd.nonzeroSingularValues();
		_Q.setConstant(std::numeric_limits<Scalar>::infinity());
	}

	void setBound(JointVec const & Q)
	{
#if 0
		_Q.noalias() = _svd.matrixV().transpose()*Q;
#else
		_Q = Q;
#endif
	}

	template<typename Derived>
	JointVec solve(Eigen::MatrixBase<Derived> const & pdot)
	{
		TaskVec u = _svd.matrixU().transpose()*pdot;

		TaskVec v;
		for (int k = 0; k < _rank; k++)
		{
#if 0
			v[k] = min(::fabs(u[k])/_svd.singularValues()[k], _Q[k]);
			if (u[k] < 0)
					v[k] *= -1;
		}

		return _svd.matrixV().leftCols(_rank)*v.head(_rank); 
#else
			v[k] = u[k]/_svd.singularValues()[k];
		}

		JointVec qdot = _svd.matrixV().leftCols(_rank)*v.head(_rank); 

		for (int k = 0; k < DOF;  k++)
		{
			if (::fabs(qdot[k]) > _Q[k])
				qdot[k] = (qdot[k] > 0) ? _Q[k] : -_Q[k];
		}

		return qdot; 
#endif
	}

	template<typename Derived>
	JointVec solve(Eigen::MatrixBase<Derived> const & pdot, JointVec const & Q)
	{
		setBound(Q);

		return solve(pdot); 
	}
	
private:
	Eigen::JacobiSVD<MatrixType> _svd;
	int _rank;

	JointVec _Q;
};

} // namespace NRMKFoundation
