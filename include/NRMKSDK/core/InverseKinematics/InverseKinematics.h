//  ---------------------- Doxygen info ----------------------
//! \file InverseKinematics.h
//!
//! \brief
//! Header file for the class InverseKinematics (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements an inverse kinematics class
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
//! \date March 2016
//! 
//! \version 1.9.4
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note 
//!  - v1.9.4(20160313) : Template parameter 'JacobianSolverType' is removed from InverseKinematicsBase
//!		Now class NumericInverseKinematics includes two algorithms, JacobianSolverQR and JacobianSolverTP by default.
//!	 - v1.9.4(20160312) : JacobianSolverTP implemented 
//!           
//!
//!	 Copyright (C) 2013-2016 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

//#include <iostream>

#include <Eigen/Eigen>

#include "../NRMKCommon.h"

#include <stdexcept>

namespace NRMKFoundation
{
namespace internal
{
}

/// ADDED @20160312
//  ---------------------- Doxygen info ----------------------
//! \brief
//! Compute null space bases matrix with its inverse (or pseudoinverse)
//!
//! \param J Jacobian
//! \param Z Null space bases matrix 
//! \param Jinv (pseudo-)inverse of J
//  ----------------------------------------------------------
template<typename TaskJac, typename NullJac, typename TaskJacInv>
inline void ComputeNullSpaceBases(TaskJac const & J, NullJac & Z, TaskJacInv & Jinv)
{
	if (J.rows() > 0)
	{
		if (J.rows() < J.cols())
		{
#if 1
			// Construct SVD of J
			Eigen::JacobiSVD<TaskJac> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
		
			// Construct null space bases matrix
			Z = svd.matrixV().rightCols(J.cols() - J.rows()).transpose();

			// Jacobian pseudoinverse
			typename Eigen::JacobiSVD<TaskJac>::Index nonzeroSingVals = svd.nonzeroSingularValues();		
			Jinv = svd.matrixV().leftCols(nonzeroSingVals); 
			Jinv *= svd.singularValues().head(nonzeroSingVals).asDiagonal().inverse() * svd.matrixU().leftCols(nonzeroSingVals).transpose();		
#else
			Eigen::ColPivHouseholderQR<TaskJacInv> qr(J.transpose());
			
			typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, TaskJac::MaxColsAtCompileTime, TaskJac::MaxColsAtCompileTime> JointMat;

			JointMat Q = qr.householderQ();
 			Z.transpose() = Q.rightCols(J.cols() - J.rows());
 
 			// Jacobian pseudoinverse
 			JointMat Id(J.cols(), J.cols());
			Id.setIdentity();

 			Jinv.transpose() = qr.solve(Id);
#endif
		}
		else if (J.rows() == J.cols())
		{
			Eigen::ColPivHouseholderQR<TaskJac> qr(J);

			Z.resize(0, J.cols());
			Jinv = qr.inverse();
		}
		else
		{
			assert(false);
		}
	}
	else 
	{
		Z.resize(J.cols(), J.cols());
		Z.setIdentity(J.cols(), J.cols());

		Jinv.resize(J.cols(), 0);
	}
}

/// ADDED @20160312
//  ---------------------- Doxygen info ----------------------
//! \brief
//! Compute null space bases matrix with its inverse (or pseudoinverse)
//!
//! \param J Jacobian
//! \param Z Null space bases matrix
//! \param Jinv (pseudo-)inverse of J
//! \param smax maximum singular value of J
//  ----------------------------------------------------------
template<typename TaskJac, typename NullJac, typename TaskJacInv>
inline void ComputeNullSpaceBases(TaskJac const & J, NullJac & Z, TaskJacInv & Jinv, double& smax)
{
	if (J.rows() > 0)
	{
		if (J.rows() < J.cols())
		{
#if 1
			// Construct SVD of J
			Eigen::JacobiSVD<TaskJac> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);

			// Construct null space bases matrix
			Z = svd.matrixV().rightCols(J.cols() - J.rows()).transpose();

			// Jacobian pseudoinverse
			typename Eigen::JacobiSVD<TaskJac>::Index nonzeroSingVals = svd.nonzeroSingularValues();
			Jinv = svd.matrixV().leftCols(nonzeroSingVals);
			Jinv *= svd.singularValues().head(nonzeroSingVals).asDiagonal().inverse() * svd.matrixU().leftCols(nonzeroSingVals).transpose();
			smax = 0;
			for(int i = 0 ; i < J.rows() ; i++)
				if(smax > fabs(svd.singularValues()[i])) smax = fabs(svd.singularValues()[i]);
#else
			Eigen::ColPivHouseholderQR<TaskJacInv> qr(J.transpose());

			typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, TaskJac::MaxColsAtCompileTime, TaskJac::MaxColsAtCompileTime> JointMat;

			JointMat Q = qr.householderQ();
 			Z.transpose() = Q.rightCols(J.cols() - J.rows());

 			// Jacobian pseudoinverse
 			JointMat Id(J.cols(), J.cols());
			Id.setIdentity();

 			Jinv.transpose() = qr.solve(Id);
#endif
		}
		else if (J.rows() == J.cols())
		{
			Eigen::ColPivHouseholderQR<TaskJac> qr(J);

			Z.resize(0, J.cols());
			Jinv = qr.inverse();
			smax = qr.absDeterminant();
		}
		else
		{
			assert(false);
		}
	}
	else
	{
		Z.resize(J.cols(), J.cols());
		Z.setIdentity(J.cols(), J.cols());

		Jinv.resize(J.cols(), 0);
	}
}

//  ---------------------- Doxygen info ----------------------
//! \class JacobianSolverBase
//!
//! \brief
//! This implements a Jacobian solver class.
//!
//! \details
//! Class JacobianSolverBase models linear solver algorithm for a systems of linear equations modeled by Jacobian matrix. 
//!
//! In order to enforce the static polymorphism the derived class should inherit the class by the CRTP pattern. 
//! See http://en.wikipedia.org/wiki/Curiously_recurring_template_pattern.
//!  
//! \tparam DIM Task dimension
//! \tparam DOF Joint dof 
//! \tparam JacobianSolverType Derived class, i.e. concrete or user-defined class
//!
//! \sa JacobianSolverQR
//  ----------------------------------------------------------
template<int DIM, int DOF, typename JacobianSolverType>
class JacobianSolverBase
{
public:
	/// DELETED @20160312
	//typedef Eigen::Matrix<double, DIM, DOF> TaskJac;
	//typedef Eigen::Matrix<double, DIM, 1> TaskVec;
	//typedef Eigen::Matrix<double, DOF, 1> JointVec;

	//! \returns Reference to the derived object 
	inline JacobianSolverType& derived() { return *static_cast<JacobianSolverType*>(this); }
	//! \returns Constant reference to the derived object 
	inline const JacobianSolverType& derived() const { return *static_cast<const JacobianSolverType*>(this); }

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the jacobian solution algorithm
	//!
	//! \details
	//! It solves the equtaion vel = J*qdot for qdot given vel. 
	//! 
	//! \param J Task Jacobian
	//! \param vel Task velocity desired
	//! \param qdot Joint vector
	//  ----------------------------------------------------------
	template <typename DerivedTaskJac, typename DerivedTaskVec, typename DerivedJointVec>
	inline int solve(DerivedTaskJac const & J, DerivedTaskVec const & vel, DerivedJointVec & qdot)
	{
		return derived().solve(J, vel, qdot); 
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the jacobian solution algorithm using already computed decomposition
	//!
	//! \details
	//! It solves the equtaion vel = _J*qdot for qdot given vel. 
	//! 
	//! \param vel Task velocity desired
	//! \param qdot Joint vector
	//  ----------------------------------------------------------
	template <typename DerivedTaskVec, typename DerivedJointVec>
	inline int solveAgain(DerivedTaskVec const & vel, DerivedJointVec & qdot)
	{
		return derived().solveAgain(vel, qdot); 
	}
};

//  ---------------------- Doxygen info ----------------------
//! \class JacobianSolverInverse
//!
//! \brief
//! This implements a Jacobian solver class by analytic inverse
//!
//! \tparam DIM Task dimension
//! \tparam DOF Joint dof 
//!
//! \sa JacobianSolverBase
//  ----------------------------------------------------------
template<int DIM, int DOF>
class JacobianSolverInverse : public JacobianSolverBase<DIM, DOF, JacobianSolverInverse<DIM, DOF> >
{
public:
	typedef Eigen::Matrix<double, DOF, DIM> JacInv;  //!< Typedef of task Jacobian pseudoinverse matrix
	
public:
	//inline JacobianSolverInverse() : JacobianSolverBase<DIM, DOF, JacobianSolverInverse<DIM, DOF> >()
	//{
	//}

	//! \brief sets the analytic inverse 
	inline void setJinv(JacInv const & Jinv) 
	{
		_Jinv = Jinv;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! solves the system by QR decomposition of the jacobian
	//!
	//! \detail
	//! it returns '0' in case of successful solution, '1' for possible singular situation.
	//!
	//! \param J Task Jacobian
	//! \param vel Task velocity desired
	//! \param qdot Joint vector
	//  ----------------------------------------------------------
	template <typename DerivedTaskJac, typename DerivedTaskVec, typename DerivedJointVec>
	inline int solve(DerivedTaskJac const & J, DerivedTaskVec const & vel, DerivedJointVec & qdot)
	{
		return solveAgain(vel, qdot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! solves the system by QR decomposition of the jacobian, already computed
	//!
	//! \detail
	//! it returns '0' in case of successful solution, '1' for possible singular situation.
	//!
	//! \param vel Task velocity desired
	//! \param qdot Joint vector
	//  ----------------------------------------------------------
	template <typename DerivedTaskVec, typename DerivedJointVec>
	inline int solveAgain(DerivedTaskVec const & vel, DerivedJointVec & qdot)	
	{
		qdot = _Jinv*vel;

		return 0;
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	JacInv _Jinv;
};

//  ---------------------- Doxygen info ----------------------
//! \class JacobianSolverQR
//!
//! \brief
//! This implements a Jacobian solver class by QR decomposition method
//!
//! \tparam DIM Task dimension
//! \tparam DOF Joint dof 
//!
//! \sa JacobianSolverBase
//  ----------------------------------------------------------
template<int DIM, int DOF>
class JacobianSolverQR : public JacobianSolverBase<DIM, DOF, JacobianSolverQR<DIM, DOF> >
{
public:
	inline JacobianSolverQR() : JacobianSolverBase<DIM, DOF, JacobianSolverQR<DIM, DOF> >()
	, _singularErr("SingularPositionError")
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! solves the system by QR decomposition of the jacobian
	//!
	//! \detail
	//! it returns '0' in case of successful solution, '1' for possible singular situation.
	//!
	//! \param J Task Jacobian
	//! \param vel Task velocity desired
	//! \param qdot Joint vector
	//  ----------------------------------------------------------
	template <typename DerivedTaskJac, typename DerivedTaskVec, typename DerivedJointVec>
	inline int solve(DerivedTaskJac const & J, DerivedTaskVec const & vel, DerivedJointVec & qdot)
	{
		_qr.compute(J);

		if (_qr.absDeterminant() < 0.004)
		{
//			throw _singularErr;
			return 1;
		}

		return solveAgain(vel, qdot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! solves the system by QR decomposition of the jacobian, already computed
	//!
	//! \detail
	//! it returns '0' in case of successful solution, '1' for possible singular situation.
	//!
	//! \param vel Task velocity desired
	//! \param qdot Joint vector
	//  ----------------------------------------------------------
	template <typename DerivedTaskVec, typename DerivedJointVec>
	inline int solveAgain(DerivedTaskVec const & vel, DerivedJointVec & qdot)	
	{
//		assert(_qr.rank() == DIM);

		if (_qr.rank() != DIM)
		{
//			throw _singularErr;
			return 1;
		}

		qdot = _qr.solve(vel);
		return 0;
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	Eigen::ColPivHouseholderQR<Eigen::Matrix<double, DIM, DOF> > _qr;
	std::runtime_error _singularErr;
};

//  ---------------------- Doxygen info ----------------------
//! \class JacobianSolverTP
//!
//! \brief
//! This implements a Jacobian solver class by task-priority method.
//!
//! \note 
//! After creating an instance, setPrimaryIndex() should be called to specify the primary task.
//!
//! \code{.cpp}
//! NRMKFoundation::internal::IndexSet<TaskKinematics::DIM> pindex; 
//!	pindex.addIndex(3);
//!	pindex.addIndex(4);
//!	pindex.addIndex(5);
//! ...
//!	JacobianSolverTP<DIM, DOF> jacobianSolver;
//! jacobianSolver.setPrimaryIndex(pindex);
//! \endcode
//!
//! \tparam DIM Task Dimension (a whole sum)
//! \tparam DOF Joint dof 
//! \tparam PRIORITY_LEVEL Priority levels (two by default, i.e. primary and secondary task)
//!
//! \sa JacobianSolverBase
//  ----------------------------------------------------------
template<int DIM, int DOF, int PRIORITY_LEVEL = 2>
class JacobianSolverTP : public JacobianSolverBase<DIM, DOF, JacobianSolverTP<DIM, DOF> >
{
public:
	typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, DIM, 1> ComponentTaskVec;

	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, DIM, DOF> ComponentTaskJac;
	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, DOF, DIM> ComponentTaskJacInv;

public:
	//inline JacobianSolverTP() : JacobianSolverBase<DIM, DOF, JacobianSolverTP<DIM, DOF> >()
	//{
	//}
	
	/// ADDED @20160312
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the task priority index set and partitions the index set 
	//!
	//! \note
	//! it should be called before solve().
	//!
	//! \param idx Index set (of type derived from internal::IndexSet<DIM>
	//  ----------------------------------------------------------
	inline void setPriorityIndex(internal::IndexSet<DIM> const & idx)
	{
		idx.partitionIndexVector(_idx1, _idx2);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! solves the system by task priority method 
	//!
	//! \detail
	//! it returns '0' in case of successful solution, '1' for possible singular situation.
	//!
	//! \param J Task Jacobian
	//! \param vel Task velocity desired
	//! \param qdot Joint vector
	//  ----------------------------------------------------------
	template<typename TaskJac, typename TaskVec, typename JointVec>
	inline int solve(TaskJac const & J, TaskVec const & vel, JointVec & qdot) 
	{
		// partition Jacobian
		_partitionJacobian(J);
		
		//_computeNullSpaceBases();
//		ComputeNullSpaceBases(_J1, _Z1, _Jinv1);
//		ComputeNullSpaceBases(_J2, _Z2, _Jinv2);
		ComputeNullSpaceBases(_J1, _Z1, _Jinv1, _smax1);
		ComputeNullSpaceBases(_J2, _Z2, _Jinv2, _smax2);
		
		return solveAgain(vel, qdot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! solves the system again by task priority method 
	//!
	//! \detail
	//! it returns '0' in case of successful solution, '1' for possible singular situation.
	//!
	//! \param vel Task velocity desired
	//! \param qdot Joint vector
	//  ----------------------------------------------------------
	template<typename TaskVec, typename JointVec>
	inline int solveAgain(TaskVec const & vel, JointVec & qdot) 
	{
		// partition task vector
		ComponentTaskVec v1(_idx1.size());
		ComponentTaskVec v2(_idx2.size());

		// partition velocity
		for (int k = 0; k < _idx1.size(); k++)
			v1[k] = vel[_idx1[k]];

		for (int k = 0; k < _idx2.size(); k++)
			v2[k] = vel[_idx2[k]];

		// Main algorithm
		// For qdot2
		JointVec qdot2 = _Jinv2*v2; // + _Z2.transpose()*_Z2*nullvelocity; 

		// For v1		
		qdot = _Jinv1*v1 + _Z1.transpose()*(_Z1*qdot2);

		return 0;
	}

	inline double singularValueMax()
	{
		return (_smax1 > _smax2) ? _smax1 : _smax2;
	}

private:
	// partition Jacobian		
	template<typename TaskJac>
	inline void _partitionJacobian(TaskJac const & J)
	{
		_J1.resize(_idx1.size(), J.cols());
		_J2.resize(_idx2.size(), J.cols());

		for (int k = 0; k < _idx1.size(); k++)
			_J1.row(k) = J.row(_idx1[k]);

		for (int k = 0; k < _idx2.size(); k++)
			_J2.row(k) = J.row(_idx2[k]);
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
private:
	typename internal::IndexSet<DIM>::IndexVecType _idx1;
	typename internal::IndexSet<DIM>::IndexVecType _idx2;

	ComponentTaskJac _J1;
	ComponentTaskJac _J2;

	ComponentTaskJac _Z1;
	ComponentTaskJac _Z2;

	ComponentTaskJacInv _Jinv1;
	ComponentTaskJacInv _Jinv2;

	double _smax1;
	double _smax2;
};

//  ---------------------- Doxygen info ----------------------
//! \class InverseKinematicsBase
//!
//! \brief
//! This implements a base inverse kinematics class.
//!
//! \details
//! Class InverseKinematicsBase models inverse kinematics algorithm which computes the joint position or velocity solution 
//! corresponding to task position and/or velocity. 
//!
//! In order to enforce the static polymorphism the derived class should inherit the class by the CRTP pattern. 
//! See http://en.wikipedia.org/wiki/Curiously_recurring_template_pattern.
//!  
//! \tparam _SubsysType Type of the subsys
//! \tparam _KinematicsType Type of task kinematics 
//! \tparam InverseKinematicsType Derived class 
//!
//! \sa NumericInverseKinematics
//  ----------------------------------------------------------
template<typename _SubsysType, typename _KinematicsType, typename InverseKinematicsType>
class InverseKinematicsBase
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \enum
	//! 
	//! \brief Provides compile-time constants 
	//  ----------------------------------------------------------
	enum 
	{	
		JOINT_DOF = _SubsysType::JOINT_DOF, //!< Degrees-of-freedom of the total joints of the system (includeing an earthing joint) 
		DIM = _KinematicsType::DIM, //!< Task dimension
	};

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides Typedefs 
	//  ----------------------------------------------------------
	typedef _SubsysType SubsysType;
	typedef _KinematicsType KinematicsType;

	typedef typename SubsysType::JointVec JointVec; //!< Typedef of the type for joint vector
	typedef typename KinematicsType::PosType TaskPosition; //!< Typedef of the type for position variable
	typedef typename KinematicsType::VelType TaskVelocity; //!< Typedef of the type for velocity variable
	typedef Eigen::Matrix<double, DIM, 1> TaskVec; //!< Typedef of task vector
	typedef Eigen::Matrix<double, DIM, JOINT_DOF> TaskJac; //!< Typedef of task Jacobian matrix

	//! \returns Reference to the derived object 
	inline InverseKinematicsType& derived() { return *static_cast<InverseKinematicsType*>(this); }
	//! \returns Constant reference to the derived object 
	inline const InverseKinematicsType& derived() const { return *static_cast<const InverseKinematicsType*>(this); }

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint velocity solution 
	//!
	//! \details
	//! It solves the joint velocity of the subsys corresponding to desired task velocity by Jacobian solver
	//! 
	//! \param J Task Jacobian
	//! \param vel Task velocity desired
	//! \param qdot Joint vector
	//  ----------------------------------------------------------
	template<typename DerivedTaskJac, typename DerivedTaskVec, typename DerivedJointVec>
	void solveJacobian(DerivedTaskJac const & J, DerivedTaskVec const & vel, DerivedJointVec & qdot) 
	{
		derived().solveJacobian(J, vel, qdot);
	}


	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint velocity solution 
	//!
	//! \details
	//! It solves the joint velocity of the subsys corresponding to desired task velocity by Jacobian solver
	//! 
	//! \param vel Task velocity desired
	//! \param qdot Joint vector
	//  ----------------------------------------------------------
	template<typename DerivedTaskVec, typename DerivedJointVec>
	void solveJacobianAgain(DerivedTaskVec const & vel, DerivedJointVec & qdot) 
	{
		derived().solveJacobianAgain(vel, qdot);
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint position solution 
	//!
	//! \details
	//! It should update the joint position of the subsys corresponding to desired task position.
	//! It is mandatory that the system (of type SubsysType) be updated before calling this function.
	//!
	//! \note 
	//! The joint solution is reserved in Subsys, i.e. subsys.q(). Task controller is used to compute the task (position) error.
	//! 
	//! \tparam TaskControllerType Task controller type
	//! 
	//! \param subsys System object
	//! \param tkin Task kinematics object
	//! \param tctrl Task controller object
	//! \param pos Task position 
	//  ----------------------------------------------------------
	template<typename TaskControllerType>
	void solvePos(SubsysType & subsys, 
		KinematicsType const & tkin, TaskControllerType & tctrl, 
		TaskPosition const & pos) 
	{
		derived().solvePos(subsys, tkin, tctrl, pos);
	}
		
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint velocity solution (by RMRC)
	//!
	//! \details
	//! It should update the task position and velocity values for the system.
	//! It is mandatory that the system (of type SubsysType) be updated before calling this function.
	//! 
	//! \param subsys System object
	//! \param J Task Jacobian
	//! \param vel_ref Task reference velocity 
	//! \param qdotref Joint reference velocity
	//  ----------------------------------------------------------
	void solveVel(SubsysType & subsys, TaskJac const & J, TaskVec & vel_ref, JointVec & qdotref)
	{
		derived().solveVel(subsys, J, vel_ref, qdotref);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint acceleration solution (by RAC)
	//!
	//! \details
	//! It should update the task position and velocity values for the system.
	//! It is mandatory that the system (of type SubsysType) be updated before calling this function.
	//! 
	//! \param subsys System object
	//! \param J Task Jacobian
	//! \param Jdot Task Jacobian derivative
	//! \param acc_ref Task reference acceleration 
	//! \param qddotref Joint reference acceleration
	//  ----------------------------------------------------------
	void solveAcc(SubsysType & subsys, TaskJac const & J, TaskJac const & Jdot, TaskVec & acc_ref, JointVec & qddotref)
	{
		derived().solveAcc(subsys, J, Jdot, acc_ref, qddotref);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint acceleration solution (by RAC)
	//!
	//! \details
	//! It should update the task position and velocity values for the system.
	//! It is mandatory that the system (of type SubsysType) be updated before calling this function.
	//! 
	//! \param subsys System object
	//! \param J Task Jacobian
	//! \param Jdot Task Jacobian derivative
	//! \param vel_ref Task reference velocity 
	//! \param acc_ref Task reference acceleration 
	//! \param qdotref Joint reference velocity
	//! \param qddotref Joint reference acceleration
	//  ----------------------------------------------------------
	void solveAcc(SubsysType & subsys, TaskJac const & J, TaskJac const & Jdot, TaskVec & vel_ref, TaskVec & acc_ref,
			JointVec & qdotref, JointVec & qddotref)
	{
		derived().solveAcc(subsys, J, Jdot, vel_ref, acc_ref, qdotref, qddotref);
	}
};

//  ---------------------- Doxygen info ----------------------
//! \class NumericInverseKinematics
//!
//! \brief
//! This implements a default numerical inverse kinematics class.
//!
//! \details
//! Class NumericInverseKinematics implements inverse kinematics algorithm by numerical iterative algorithm. 
//!  
//! \tparam SubsysType Type of the subsys
//! \tparam KinematicsType Type of task kinematics 
//!
//! \sa InverseKinematicsBase
//  ----------------------------------------------------------
template<typename SubsysType, typename KinematicsType>
class NumericInverseKinematics : public InverseKinematicsBase<SubsysType, KinematicsType, NumericInverseKinematics<SubsysType, KinematicsType> >
{
public:
	typedef InverseKinematicsBase<SubsysType, KinematicsType, NumericInverseKinematics<SubsysType, KinematicsType> > InverseKinematicsType;

	//  ---------------------- Doxygen info ----------------------
	//! \enum
	//! 
	//! \brief Provides compile-time constants 
	//  ----------------------------------------------------------
	enum 
	{	
		JOINT_DOF = SubsysType::JOINT_DOF, //!< Degrees-of-freedom of the total joints of the system (includeing an earthing joint) 
		DIM = KinematicsType::DIM, //!< Task dimension
		MAX_NO_ITERATION = 1,		//!< maximum number of iteration
	};

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides Typedefs 
	//  ----------------------------------------------------------
	typedef typename SubsysType::JointVec JointVec; //!< Typedef of the type for joint vector
	typedef typename InverseKinematicsType::TaskPosition TaskPosition; //!< Typedef of the type for position variable
	typedef typename InverseKinematicsType::TaskVelocity TaskVelocity; //!< Typedef of the type for velocity variable
	typedef typename KinematicsType::TaskVec TaskVec; //!< Typedef of task vector
	typedef typename KinematicsType::TaskJac TaskJac; //!< Typedef of task Jacobian matrix

	typedef JacobianSolverQR<DIM, JOINT_DOF> SolverQR;  //!< Typedef of JacobianSolverQR
	typedef JacobianSolverTP<DIM, JOINT_DOF> SolverTP;  //!< Typedef of JacobianSolverTP

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint velocity solution 
	//!
	//! \details
	//! It solves the joint velocity of the subsys corresponding to desired task velocity by Jacobian solver
	//! 
	//! \param J Task Jacobian
	//! \param vel Task velocity desired
	//! \param qdot Joint vector
	//  ----------------------------------------------------------
	template<typename DerivedTaskJac, typename DerivedTaskVec, typename DerivedJointVec>
	int solveJacobian(DerivedTaskJac const & J, DerivedTaskVec const & vel, DerivedJointVec & qdot)
	{
		//if (subsys.kinematicJointSize() == 0)
		if (qdot.size() == JOINT_DOF && J.cols() == JOINT_DOF)
			return _solverQR.solve(J, vel, qdot);
		else
			return _solverTP.solve(J, vel, qdot);

	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint velocity solution 
	//!
	//! \details
	//! It solves the joint velocity of the subsys corresponding to desired task velocity by Jacobian solver
	//! 
	//! \param vel Task velocity desired
	//! \param qdot Joint vector
	//  ----------------------------------------------------------
	template<typename DerivedTaskVec, typename DerivedJointVec>
	void solveJacobianAgain(DerivedTaskVec const & vel, DerivedJointVec & qdot) 
	{
		//if (subsys.kinematicJointSize() == 0)
		if (qdot.size() == JOINT_DOF)
			_solverQR.solveAgain(vel, qdot);
		else
			_solverTP.solveAgain(vel, qdot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint velocity solution for redundant kinematics
	//!
	//! \details
	//! It solves the joint velocity of the subsys corresponding to desired task velocity by Jacobian solver.
	//! Specifically it solves the Jacobian inverse directly from the null velocity kinematics.
	//! 
	//! \param J Task Jacobian
	//! \param vel Task velocity desired
	//! \param qdot Joint vector
	//! \param null Null velocity kinematics
	//  ----------------------------------------------------------
	template<typename DerivedTaskJac, typename DerivedTaskVec, typename DerivedJointVec, typename NullKinematicsType>
	void solveJacobian(DerivedTaskJac const & J, DerivedTaskVec const & vel, DerivedJointVec & qdot, NullKinematicsType const & null) 
	{
		//if (subsys.kinematicJointSize() == 0)
		if (qdot.size() == JOINT_DOF && J.cols() == JOINT_DOF)
		{
			if (null.weighted())
				_solverQR.solve(J, vel, qdot);
			else
			{
				DerivedTaskJac Jinv;
				null.getKDJSDInverse(Jinv);

				qdot = Jinv*vel;
			}
		}
		else
			_solverTP.solve(J, vel, qdot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint velocity solution for redundant kinematics
	//!
	//! \details
	//! It solves the joint velocity of the subsys corresponding to desired task velocity by Jacobian solver.
	//! Specifically it solves the Jacobian inverse directly from the null velocity kinematics.
	//! 
	//! \param vel Task velocity desired
	//! \param qdot Joint vector
	//! \param null Null velocity kinematics
	//  ----------------------------------------------------------
	template<typename DerivedTaskVec, typename DerivedJointVec, typename NullKinematicsType>
	void solveJacobianAgain(DerivedTaskVec const & vel, DerivedJointVec & qdot, NullKinematicsType const & null) 
	{
		//if (subsys.kinematicJointSize() == 0)
		if (qdot.size() == JOINT_DOF) // && J.cols() == JOINT_DOF)
		{
			if (null.weighted())
				_solverQR.solveAgain(vel, qdot);
			else
			{
				TaskJac Jinv;
				null.getKDJSDInverse(Jinv);			

				qdot = Jinv*vel;
			}
		}
		else
			_solverTP.solveAgain(vel, qdot);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint position solution 
	//!
	//! \details
	//! It should update the task position and velocity values for the system.
	//! It is mandatory that the system (of type SubsysType) be updated before calling this function.
	//! 
	//! \tparam TaskControllerType Task controller type
	//!
	//! \param subsys System object
	//! \param tkin Task kinematics object
	//! \param tctrl Task controller object
	//! \param pos Task position desired
	//  ----------------------------------------------------------
	template<typename TaskControllerType>
	void solvePos(SubsysType & subsys, 
			KinematicsType const & tkin, TaskControllerType & tctrl, 
			TaskPosition const & pos) 
	{
		TaskPosition p;
		TaskJac J;

		static const double kappa = 1.0;

		int num_ik_steps = 0;
		while (num_ik_steps++ < MAX_NO_ITERATION)
		{
			tkin.jacobian(subsys, p, J);

			// task controller
			TaskVec e;
			tctrl.error(p, pos, e);

			/// ADDED @20160406	(for ExtendedTask)		
			//tctrl.postProcessError(e);
			
			if (e.squaredNorm() < 1e-8)
				break;

			JointVec dq;
			
			//if (subsys.kinematicJointSize() == 0)
			if (_useQR(subsys))
			{
				solveJacobian(J, e, dq); 
			}
			else
			{
				/// FIXME @20160313 (Considering kinematic joints)
				Eigen::Matrix<double, DIM, Eigen::Dynamic, Eigen::ColMajor, DIM, JOINT_DOF> Jtilde;
				Jtilde.resize(DIM, JOINT_DOF - subsys.kinematicJointSize());

				for (int j = 0, k = 0; j < SubsysType::NUM_TOTAL_JOINTS; j++)
				{
					if (!subsys.isKinematicJoint(j))
					{
						for (int i = 0; i < subsys.joint(j).dof(); i++)
						{
							int idx = subsys.joint(j).index() + i;
							Jtilde.col(k++) = J.col(idx);
						}
					}
				}
			
				Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, JOINT_DOF, 1> dq_tilde;
 				solveJacobian(Jtilde, e, dq_tilde);

				subsys.fillInKinematicJointVector(dq, 0, dq_tilde);
				/*
				for (int j = 0, k = 0; j < SubsysType::NUM_TOTAL_JOINTS; j++)
				{
					if (subsys.isKinematicJoint(j))
					{
						for (int i = 0; i < subsys.joint(j).dof(); i++)
						{
							int idx = subsys.joint(j).index() + i;
							dq[idx] = 0;
						}
					}
					else
					{
						for (int i = 0; i < subsys.joint(j).dof(); i++)
						{
							int idx = subsys.joint(j).index() + i;
							dq[idx] = dq_tilde[k++];
						}
					}
				}
				*/
				////
			}

			subsys.q() += kappa*dq;

// 			if (dq.squaredNorm() < 1e-8)
// 				break;

			subsys.update(true);
			subsys.updateJacobian(true);
		}
	}
	
		//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! ADDED @20181115
	//! implements the inverse kinematics to get a joint position solution 
	//!
	//! \details
	//! It should update the task position and velocity values for the system.
	//! It is mandatory that the system (of type SubsysType) be updated before calling this function.
	//! 
	//! \tparam TaskControllerType Task controller type
	//!
	//! \param subsys System object
	//! \param tkin Task kinematics object
	//! \param tctrl Task controller object
	//! \param pos Task position desired
	//! \param kappa gain for numerical solver
	//! \param maxiter number of max iteration
	//  ----------------------------------------------------------
	template<typename TaskControllerType>
	bool solvePos(SubsysType & subsys,
			KinematicsType const & tkin, TaskControllerType & tctrl,
			TaskPosition const & pos, const double kappa, const int maxiter)
	{
		TaskPosition p;
		TaskJac J;

		int num_ik_steps = 0;
		while (num_ik_steps++ < maxiter)
		{
			tkin.jacobian(subsys, p, J);

			// task controller
			TaskVec e;
			tctrl.error(p, pos, e);

			/// ADDED @20160406	(for ExtendedTask)
			//tctrl.postProcessError(e);

			if (e.squaredNorm() < 1e-8)
				return true;

			JointVec dq;

			//if (subsys.kinematicJointSize() == 0)
			if (_useQR(subsys))
			{
				solveJacobian(J, e, dq);
			}
			else
			{
				/// FIXME @20160313 (Considering kinematic joints)
				Eigen::Matrix<double, DIM, Eigen::Dynamic, Eigen::ColMajor, DIM, JOINT_DOF> Jtilde;
				Jtilde.resize(DIM, JOINT_DOF - subsys.kinematicJointSize());

				for (int j = 0, k = 0; j < SubsysType::NUM_TOTAL_JOINTS; j++)
				{
					if (!subsys.isKinematicJoint(j))
					{
						for (int i = 0; i < subsys.joint(j).dof(); i++)
						{
							int idx = subsys.joint(j).index() + i;
							Jtilde.col(k++) = J.col(idx);
						}
					}
				}

				Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, JOINT_DOF, 1> dq_tilde;
				solveJacobian(Jtilde, e, dq_tilde);

				subsys.fillInKinematicJointVector(dq, 0, dq_tilde);
				/*
				for (int j = 0, k = 0; j < SubsysType::NUM_TOTAL_JOINTS; j++)
				{
					if (subsys.isKinematicJoint(j))
					{
						for (int i = 0; i < subsys.joint(j).dof(); i++)
						{
							int idx = subsys.joint(j).index() + i;
							dq[idx] = 0;
						}
					}
					else
					{
						for (int i = 0; i < subsys.joint(j).dof(); i++)
						{
							int idx = subsys.joint(j).index() + i;
							dq[idx] = dq_tilde[k++];
						}
					}
				}
				*/
				////
			}

			subsys.q() += kappa*dq;

// 			if (dq.squaredNorm() < 1e-8)
// 				break;

			subsys.update(true);
			subsys.updateJacobian(true);
		}
		return false;
	}

	/// ADDED @20160406
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint position solution 
	//!
	//! \details
	//! It should update the task position and velocity values for the system.
	//! It is mandatory that the system (of type SubsysType) be updated before calling this function.
	//! 
	//! \tparam TaskControllerType Task controller type
	//!
	//! \param subsys System object
	//! \param tkin Task kinematics object
	//! \param tctrl Task controller object
	//! \param pos Task position desired
	//  ----------------------------------------------------------
	template<typename TaskControllerType, typename NullKinematicsType, typename PerformanceMeasureType>
	void solvePos(SubsysType & subsys, 
			KinematicsType const & tkin, TaskControllerType & tctrl, 
			TaskPosition const & pos, 
			NullKinematicsType const & null,
			PerformanceMeasureType const & measure) 
	{
		TaskPosition p;
		TaskJac J;

		static const double kappa = 1.0;

		int num_ik_steps = 0;
		while (num_ik_steps++ < MAX_NO_ITERATION)
		{
			tkin.jacobian(subsys, p, J);

			// task controller
			TaskVec e;
			tctrl.error(p, pos, e);

			/// ADDED @20160406	(for ExtendedTask)	
			typename NullKinematicsType::NullVec nu;
			null.desMotion(subsys, measure, nu);

			e.template tail<NullKinematicsType::DIM>() = nu;
			///////////////////////////////////////////////////////
			
			if (e.squaredNorm() < 1e-8)
				break;

			JointVec dq;
			
			//if (subsys.kinematicJointSize() == 0)
			if (_useQR(subsys))
			{
				solveJacobian(J, e, dq); 
			}
			else
			{
				/// FIXME @20160313 (Considering kinematic joints)
				Eigen::Matrix<double, DIM, Eigen::Dynamic, Eigen::ColMajor, DIM, JOINT_DOF> Jtilde;
				Jtilde.resize(DIM, JOINT_DOF - subsys.kinematicJointSize());

				for (int j = 0, k = 0; j < SubsysType::NUM_TOTAL_JOINTS; j++)
				{
					if (!subsys.isKinematicJoint(j))
					{
						for (int i = 0; i < subsys.joint(j).dof(); i++)
						{
							int idx = subsys.joint(j).index() + i;
							Jtilde.col(k++) = J.col(idx);
						}
					}
				}
			
				Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, JOINT_DOF, 1> dq_tilde;
 				solveJacobian(Jtilde, e, dq_tilde);

				subsys.fillInKinematicJointVector(dq, 0, dq_tilde);
				/*
				for (int j = 0, k = 0; j < SubsysType::NUM_TOTAL_JOINTS; j++)
				{
					if (subsys.isKinematicJoint(j))
					{
						for (int i = 0; i < subsys.joint(j).dof(); i++)
						{
							int idx = subsys.joint(j).index() + i;
							dq[idx] = 0;
						}
					}
					else
					{
						for (int i = 0; i < subsys.joint(j).dof(); i++)
						{
							int idx = subsys.joint(j).index() + i;
							dq[idx] = dq_tilde[k++];
						}
					}
				}
				*/
				////
			}

			subsys.q() += kappa*dq;

// 			if (dq.squaredNorm() < 1e-8)
// 				break;

			subsys.update(true);
			subsys.updateJacobian(true);
		}
	}

// 	template<typename TaskControllerType, typename JacobianSolverType>
// 	void solvePosCP(SubsysType & subsys, 
// 				KinematicsType const & tkin, TaskControllerType const & tctrl, JacobianSolverType const & dcmp, 
// 				TaskPosition const & pos, TaskVelocity const & vel, double dT) const
// 	{
// 		//derived().solvePosCP(subsys, tkin, tctrl, pos, vel);
// 		solveVel(subsys, tkin, tctrl, dcmp, pos, vel);
// 		subsys.q() += subsys.qdot()*dT;
// 	}
		
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint velocity solution (by RMRC)
	//!
	//! \details
	//! It should update the task position and velocity values for the system.
	//! It is mandatory that the system (of type SubsysType) be updated before calling this function.
	//! 
	//! \param subsys System object
	//! \param J Task Jacobian
	//! \param vel_ref Task reference velocity 
	//! \param qdotref Joint reference velocity
	//  ----------------------------------------------------------
	void solveVel(SubsysType & subsys, TaskJac const & J, TaskVec & vel_ref, JointVec & qdotref)
	{
		//if (subsys.kinematicJointSize() == 0)
		if (_useQR(subsys))
		{
			solveJacobian(J, vel_ref, qdotref);
		}
		else
		{
			Eigen::Matrix<double, DIM, Eigen::Dynamic, Eigen::ColMajor, DIM, JOINT_DOF> Jtilde;
			Jtilde.resize(DIM, JOINT_DOF - subsys.kinematicJointSize());

			for (int j = 0, k = 0; j < SubsysType::NUM_TOTAL_JOINTS; j++)
			{
				if (subsys.isKinematicJoint(j))
				{
					for (int i = 0; i < subsys.joint(j).dof(); i++)
					{
						int idx = subsys.joint(j).index() + i;
						vel_ref -= J.col(idx)*subsys.qdot()[idx];
					}
				}
				else
				{
					for (int i = 0; i < subsys.joint(j).dof(); i++)
					{
						int idx = subsys.joint(j).index() + i;
						Jtilde.col(k++) = J.col(idx);
					}
				}
			}
			
			Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, JOINT_DOF, 1> qdotref_tilde;
 			solveJacobian(Jtilde, vel_ref, qdotref_tilde);

			subsys.fillOutKinematicJointVector(subsys.qdot(), qdotref_tilde);
			/*
			for (int j = 0, k = 0; j < SubsysType::NUM_TOTAL_JOINTS; j++)
			{
				if (!subsys.isKinematicJoint(j))
				{
					for (int i = 0; i < subsys.joint(j).dof(); i++)
					{
						int idx = subsys.joint(j).index() + i;
						subsys.qdot()[idx] = qdotref_tilde[k++];
					}
				}
			}
			*/
		}
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint acceleration solution (by RAC)
	//!
	//! \details
	//! It should update the task position and velocity values for the system.
	//! It is mandatory that the system (of type SubsysType) be updated before calling this function.
	//! 
	//! \param subsys System object
	//! \param J Task Jacobian
	//! \param Jdot Task Jacobian derivative
	//! \param acc_ref Task reference acceleration 
	//! \param qddotref Joint reference acceleration
	//  ----------------------------------------------------------
	int solveAcc(SubsysType & subsys, TaskJac const & J, TaskJac const & Jdot, TaskVec & acc_ref, JointVec & qddotref)
	{
		if (_useQR(subsys))
		{
			if(solveJacobian(J, acc_ref - Jdot*subsys.qdot(), qddotref))
				return 1;
		}
		else
		{
			Eigen::Matrix<double, DIM, Eigen::Dynamic, Eigen::ColMajor, DIM, JOINT_DOF> Jtilde;
			Jtilde.resize(DIM, JOINT_DOF - subsys.kinematicJointSize());

			for (int j = 0, k = 0; j < SubsysType::NUM_TOTAL_JOINTS; j++)
			{
				if (subsys.isKinematicJoint(j))
				{
					for (int i = 0; i < subsys.joint(j).dof(); i++)
					{
						int idx = subsys.joint(j).index() + i;
						acc_ref -= J.col(idx)*subsys.qddot()[idx];
					}
				}
				else
				{
					for (int i = 0; i < subsys.joint(j).dof(); i++)
					{
						int idx = subsys.joint(j).index() + i;
						Jtilde.col(k++) = J.col(idx);
					}
				}
			}
			
			Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, JOINT_DOF, 1> qddotref_tilde;
 			solveJacobian(Jtilde, acc_ref - Jdot*subsys.qdot(), qddotref_tilde);

			subsys.fillInKinematicJointVector(qddotref, subsys.qddot(), qddotref_tilde);
			/*
			for (int j = 0, k = 0; j < SubsysType::NUM_TOTAL_JOINTS; j++)
			{
				if (subsys.isKinematicJoint(j))
				{
					for (int i = 0; i < subsys.joint(j).dof(); i++)
					{
						int idx = subsys.joint(j).index() + i;
						qddotref[idx] = subsys.qddot()[idx];
					}
				}
				else
				{
					for (int i = 0; i < subsys.joint(j).dof(); i++)
					{
						int idx = subsys.joint(j).index() + i;
						qddotref[idx] = qddotref_tilde[k++];
					}
				}
			}
			*/
		}

		return 0;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint acceleration solution (by RAC)
	//!
	//! \details
	//! It should update the task position and velocity values for the system.
	//! It is mandatory that the system (of type SubsysType) be updated before calling this function.
	//! 
	//! \param subsys System object
	//! \param J Task Jacobian
	//! \param Jdot Task Jacobian derivative
	//! \param vel_ref Task reference velocity 
	//! \param acc_ref Task reference acceleration 
	//! \param qdotref Joint reference velocity
	//! \param qddotref Joint reference acceleration
	//  ----------------------------------------------------------
	int solveAcc(SubsysType & subsys, TaskJac const & J, TaskJac const & Jdot, TaskVec & vel_ref, TaskVec & acc_ref,
			JointVec & qdotref, JointVec & qddotref)
	{
		if (_useQR(subsys))
		{
			if(solveJacobian(J, acc_ref - Jdot*subsys.qdot(), qddotref))
			{
				return 1;
			}
			solveJacobianAgain(vel_ref, qdotref);
		}
		else
		{
			Eigen::Matrix<double, DIM, Eigen::Dynamic, Eigen::ColMajor, DIM, JOINT_DOF> Jtilde;
			Jtilde.resize(DIM, JOINT_DOF - subsys.kinematicJointSize());

			for (int j = 0, k = 0; j < SubsysType::NUM_TOTAL_JOINTS; j++)
			{
				if (subsys.isKinematicJoint(j))
				{
					for (int i = 0; i < subsys.joint(j).dof(); i++)
					{
						int idx = subsys.joint(j).index() + i;
						vel_ref -= J.col(idx)*subsys.qdot()[idx];
						acc_ref -= J.col(idx)*subsys.qddot()[idx];
					}
				}
				else
				{
					for (int i = 0; i < subsys.joint(j).dof(); i++)
					{
						int idx = subsys.joint(j).index() + i;
						Jtilde.col(k++) = J.col(idx);
					}
				}
			}
			
			Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, JOINT_DOF, 1> qddotref_tilde;
			solveJacobian(Jtilde, acc_ref - Jdot*subsys.qdot(), qddotref_tilde);

			Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, JOINT_DOF, 1> qdotref_tilde;
			solveJacobianAgain(vel_ref, qdotref_tilde);

			subsys.fillInKinematicJointVector(qddotref, subsys.qddot(), qddotref_tilde);
			subsys.fillInKinematicJointVector(qdotref, subsys.qdot(), qdotref_tilde);
			/*
			for (int j = 0, k = 0; j < SubsysType::NUM_TOTAL_JOINTS; j++)
			{
				if (subsys.isKinematicJoint(j))
				{
					for (int i = 0; i < subsys.joint(j).dof(); i++)
					{
						int idx = subsys.joint(j).index() + i;
						qdotref[idx] = subsys.qdot()[idx];
						qddotref[idx] = subsys.qddot()[idx];
					}
				}
				else
				{
					for (int i = 0; i < subsys.joint(j).dof(); i++)
					{
						int idx = subsys.joint(j).index() + i;
						qdotref[idx] = qdotref_tilde[k++];
						qddotref[idx] = qddotref_tilde[k++];
					}
				}
			}
			*/
		}
		return 0;
	}

	/// ADDED @20160418
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint acceleration solution (by RAC) 
	//! for redundant manipulator (making advantage of previously computed null kinematics)
	//!
	//! \details
	//! It should update the task position and velocity values for the system.
	//! It is mandatory that the system (of type SubsysType) be updated before calling this function.
	//! 
	//! \param subsys System object
	//! \param J Task Jacobian
	//! \param Jdot Task Jacobian derivative
	//! \param acc_ref Task reference acceleration 
	//! \param qddotref Joint reference acceleration
	//! \param null null kinematics
	//  ----------------------------------------------------------
	template <typename NullKinematicsType>
	void solveAcc(SubsysType & subsys, TaskJac const & J, TaskJac const & Jdot, TaskVec & acc_ref, JointVec & qddotref, NullKinematicsType const & null)
	{
		if (_useQR(subsys))
		{
			solveJacobian(J, acc_ref - Jdot*subsys.qdot(), qddotref, null);
		}
		else
		{
			solveAcc(subsys, J, Jdot, acc_ref, qddotref);
		}
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! implements the inverse kinematics to get a joint acceleration solution (by RAC) 
	//! for redundant manipulator (making advantage of previously computed null kinematics)
	//!
	//! \details
	//! It should update the task position and velocity values for the system.
	//! It is mandatory that the system (of type SubsysType) be updated before calling this function.
	//! 
	//! \param subsys System object
	//! \param J Task Jacobian
	//! \param Jdot Task Jacobian derivative
	//! \param vel_ref Task reference velocity 
	//! \param acc_ref Task reference acceleration 
	//! \param qdotref Joint reference velocity
	//! \param qddotref Joint reference acceleration
	//! \param null null kinematics
	//  ----------------------------------------------------------
	template <typename NullKinematicsType>
	void solveAcc(SubsysType & subsys, TaskJac const & J, TaskJac const & Jdot, TaskVec & vel_ref, TaskVec & acc_ref,
			JointVec & qdotref, JointVec & qddotref, NullKinematicsType const & null)
	{
		if (_useQR(subsys))
		{
			solveJacobian(J, acc_ref - Jdot*subsys.qdot(), qddotref, null);
			solveJacobianAgain(vel_ref, qdotref, null);
		}
		else
		{
			solveAcc(subsys, J, Jdot, vel_ref, acc_ref, qdotref, qddotref);
		}
	}

	/*
	/// ADDED @20160312
	SolverQR & jacobianSolver() 
	{
		return _solverQR;
	}

	SolverTP & jacobianSolverTP() 
	{
		return _solverTP;
	}
	*/

	/// ADDED @20160312
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the task priority index set
	//!
	//! \note
	//! it should be called before solve().
	//!
	//! \param idx Index set (of type derived from internal::IndexSet<DIM>
	//  ----------------------------------------------------------
	inline void setPriorityIndex(internal::IndexSet<DIM> const & idx)
	{
		_solverTP.setPriorityIndex(idx);
	}

private:
	inline bool _useQR(SubsysType const & subsys) const
	{
		return (JOINT_DOF >= DIM) && (subsys.kinematicJointSize() == 0);
	}

private:
	SolverQR _solverQR;
	SolverTP _solverTP;
};

} // namespace NRMKFoundation
