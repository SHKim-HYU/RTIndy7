//! \file ForwardDynamics.h
//!
//! \brief Header file for the class ForwardDynamics (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a forward dynamics algorithm class
//! to be used for the interface of NRMKFoundation library
//! \n
//! \copydetails Neuromeka Foundation Library
//! \n
//! Neuromeka \n
//! South Korea\n
//! \n
//! http://www.neuromeka.com\n
//!
//! \date January 2017
//! 
//! \version 2.0.0
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013--2017 Neuromeka

#pragma once

#include "LieGroup/LieGroup.h"

#include "Body.h"
#include "Joint.h"
#include "FlexibleJoint.h"

//#include "CustomDynamics.h"

//#include "Constraint.h"

//#include "Loop.h"
//#include "Subsys.h"

//#include "Integrator.h"

//#include "LCP.h"

namespace NRMKFoundation
{
namespace internal
{
}

//  ---------------------- Doxygen info ----------------------
//! \class ForwardDynamics
//!
//! \brief
//! This implements a forward dynamics class.
//!
//! \details
//! Class ForwardDynamics models any type of forward dynamics algorithm for the given system. 
//! Since the system may be either floating or fixed, 
//! one has to deal with both cases in the derived concrete integrator class. 
//! In addition, it is supposed that after the member function update() has been executed, 
//! all body external wrench should be cleared as well as the control body wrench and the joint torque vector. 
//!
//! In order to enforce the static polymorphism the derived class should inherit the class by the CRTP pattern. 
//! See http://en.wikipedia.org/wiki/Curiously_recurring_template_pattern.
//!  
//! \tparam SubsysType Type of the subsys
//! \tparam ForwardDynamicsType Derived class 
//!
//! \sa ABMethod
//  ----------------------------------------------------------
template <typename _SubsysType, typename ForwardDynamicsType>
class ForwardDynamics
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides Typedefs 
	//  ----------------------------------------------------------
	typedef _SubsysType SubsysType;	//!< Typedef of the subsys
	typedef typename SubsysType::JointVec JointVec; //!< Typedef of the vector having the dimension of the total joint dof

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides compile-time constants of the subsys
	//  ----------------------------------------------------------
	enum 
	{	
		NUM_BODIES = SubsysType::NUM_BODIES,  //!< Number of bodies
		JOINT_DOF = SubsysType::JOINT_DOF, //!< Joint degrees-of-freedom
		EARTH_DOF = 0, //!< Degrees-of-freedom of the earthing joint
	};

	//! \returns Reference to the derived object 
	ForwardDynamicsType& derived() { return *static_cast<ForwardDynamicsType*>(this); }
	//! \returns Constant reference to the derived object 
	const ForwardDynamicsType& derived() const { return *static_cast<const ForwardDynamicsType*>(this); }

	LieGroup::Vector3D const & gacc() const { return _gacc; }

	SubsysType & subsys() { return _subsys; }

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the forward dynamics object
	//!
	//! \param subsys System object (of type SubsysType)
	//! \param gacc Gravitation acceleration vector (represented with respect to the global reference frame)
	//  ----------------------------------------------------------
	inline ForwardDynamics(SubsysType & subsys, LieGroup::Vector3D const & gacc)
		: _subsys(subsys), _gacc(gacc), _delT(0)
	{
	}

// 	// FIXME @20130114: This should move to SubsysBase
// 	//  ---------------------- Doxygen info ----------------------
// 	//! \brief
// 	//! updates the system custom dynamics
// 	//  ----------------------------------------------------------
// 	inline void updateCustomDynamics(double t = 0)
// 	{
// 		derived().updateCustomDynamics(t);
// 	}

// 	//  ---------------------- Doxygen info ----------------------
// 	//! \brief
// 	//! updates the system acceleration
// 	//  ----------------------------------------------------------
// 	inline void update(double t = 0)
// 	{
// 		derived().update(t);
// 	}

	inline void solve()
	{
		derived().solve();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the effective dynamics of a body 
	//!
	//! \detail 
	//! Effective dynamics of a body is expressed as \f$ \dot{V}_k = A_k F_k + b_k \f$ where \f$ F_k \f$ and \f$ \dot{V}_k \f$ denote 
	//! body wrench and body acceleration of body of index \f$ k \f$. The coefficient matrix \f$ A_k \f$ and the bias vector \f$ b_k \f$
	//! are called the effective inertia matrix inverse and the effective bias acceleration, respectively.
	//! 
	//! \param index Index of the body belonging to _subsys
	//! \param A Effective inertia matrix inverse 
	//! \param b Effective bias acceleration 
	//  ----------------------------------------------------------
	inline void effectiveDynamics(int index, Matrix6d & A, Vector6d & b) 
	{
		derived().effectiveDynamics(index, A, b);
	}

	template<typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
	inline void effectiveDynamics(Eigen::MatrixBase<Derived1> const & J, Eigen::MatrixBase<Derived2> const & Jdot, 
		Eigen::MatrixBase<Derived3> const & A, Eigen::MatrixBase<Derived4> const & b,
		Eigen::MatrixBase<Derived5> const & sol) 
	{
		derived().effectiveDynamics(J, Jdot, A, b, sol);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the system integration period
	//  ----------------------------------------------------------
	inline void setPeriod(double dT)
	{
		_delT = dT;
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
protected:
	//  ---------------------- Doxygen info ----------------------
	//! \brief System object
	//  ----------------------------------------------------------
	SubsysType & _subsys;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Gravitational acceleration vector
	//  ----------------------------------------------------------
	LieGroup::Vector3D _gacc;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Numerical integration period
	//  ----------------------------------------------------------
	double _delT;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Default numerical integrator
	//  ----------------------------------------------------------
	//EulerCromerMethod<SubsysType> _integrator;
};

//! \class ABMethod
//!
//! \brief
//! This implements a articulated body method.
//!  
//! \tparam SubsysType Type of the subsys
//!
//! \sa ForwardDynamics
//  ----------------------------------------------------------
template<typename _SubsysType>
class ABMethod : public ForwardDynamics<_SubsysType, ABMethod<_SubsysType> >
{
public:
	typedef typename ForwardDynamics<_SubsysType, ABMethod<_SubsysType> >::SubsysType SubsysType;

	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides compile-time constants of the subsys
	//  ----------------------------------------------------------
	enum 
	{	
		NUM_BODIES = SubsysType::NUM_BODIES,  //!< Number of bodies
		JOINT_DOF = SubsysType::JOINT_DOF, //!< Joint degrees-of-freedom
		EARTH_DOF = 0, //!< Degrees-of-freedom of the earthing joint
	};
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides Typedefs 
	//  ----------------------------------------------------------
	/// FIXED @20150426
	typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, 6, 1> Vector_jdof; //!< Typedef of the vector having the maximal dimension of six
	typedef Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::ColMajor, 6, 6> Matrix_6_jdof;  //!< Typedef of the matrix having the maximal number of columns six
	typedef Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::ColMajor, 6, 6> Matrix_jdof_6; //!< Typedef of the matrix having the maximal number of rows six
	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, 6, 6> SqMatrix_jdof;//!< Typedef of the square having the maximal dimension of six
	
	typedef typename SubsysType::JointVec JointVec; //!< Typedef of the vector having the dimension of the total joint dof

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the object
	//!
	//! \param subsys System object (of type SubsysType)
	//! \param gacc Gravitation acceleration vector (represented with respect to the global reference frame)
	//  ----------------------------------------------------------
	inline ABMethod(SubsysType & subsys, LieGroup::Vector3D const & gacc)
		: ForwardDynamics<SubsysType, ABMethod<SubsysType> >(subsys, gacc)
	{
		_U[0].resize(0, 6);
		_D[0].resize(0, 0);
		_u[0].resize(0);

		for (int k = 1; k < NUM_BODIES; ++k)
		{
			Joint const & j = _subsys.joint(k);	

			_U[k].resize(j.dof(), 6);
			_D[k].resize(j.dof(), j.dof());
			_u[k].resize(j.dof());
		}
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the effective dynamics of a body 
	//!
	//! \detail 
	//! Effective dynamics of a body is expressed as \f$ \dot{V}_k = A_k F_k + b_k \f$ where \f$ F_k \f$ and \f$ \dot{V}_k \f$ denote 
	//! body wrench and body acceleration of body of index \f$ k \f$. The coefficient matrix \f$ A_k \f$ and the bias vector \f$ b_k \f$
	//! are called the effective inertia matrix inverse and the effective bias acceleration, respectively.
	//! 
	//! \param index Index of the body belonging to _subsys
	//! \param A Effective inertia matrix inverse 
	//! \param b Effective bias acceleration 
	//  ----------------------------------------------------------
	inline void effectiveDynamics(int index, Matrix6d & A, Vector6d & b) 
	{
		// In order to deal with clearExtWrench() in update()
		// FIXME @20131002 : need to improve
		//LieGroup::Wrench F = _subsys.F();
		//typename SubsysType::JointVec tau = _subsys.tau();

		// FIXME @20140114: Fext[] is reused below, which should not be.
		//LieGroup::Wrench Fext[SubsysType::NUM_BODIES];
		//for (int k = 0; k < SubsysType::NUM_BODIES; k++)
		//	Fext[k] = _subsys.body(k).Fext();

		LieGroup::Wrench Findex = _subsys.body(index).Fext();

		// computation begins...
		// for bias
		solve();
		b = _subsys.Vdot(index);

		//_subsys.body(index).clearWrench();
		
		for (int k = 0; k < 6; k++)
		{
			// recover the external wrench
			//_subsys.tau() = tau;
			//_subsys.F() = F;

			//Fext[index][k] += 1;
			//for (int j = 0; j < SubsysType::NUM_BODIES; j++)
			//	_subsys.body(j).addExtWrench(Fext[j]);

			LieGroup::Wrench Ftest(Findex);
			Ftest[k] += 1;

			_subsys.body(index).setExtWrench(Ftest);

			solve();
			A.col(k) = _subsys.Vdot(index) - b;
		}

		// recover the external wrench for future use
		//_subsys.tau() = tau;
		//_subsys.F() = F;

		//for (int k = 0; k < SubsysType::NUM_BODIES; k++)
		//	_subsys.body(k).setExtWrench(Fext[k]);

		_subsys.body(index).setExtWrench(Findex);
	}
	
	template<typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
	inline void effectiveDynamics(Eigen::MatrixBase<Derived1> const & J, Eigen::MatrixBase<Derived2> const & Jdot, 
		Eigen::MatrixBase<Derived3> const & A_, Eigen::MatrixBase<Derived4> const & b_,
		Eigen::MatrixBase<Derived5> const & sol_) 
	{
		LieGroup::Wrench F = _subsys.F();
		typename SubsysType::JointVec tau = _subsys.tau();

		// resize
		Eigen::MatrixBase<Derived3> & A = const_cast<Eigen::MatrixBase<Derived3> &>(A_);
		Eigen::MatrixBase<Derived4> & b = const_cast<Eigen::MatrixBase<Derived4> &>(b_);

		A.derived().resize(J.rows(), J.rows());
		b.derived().resize(J.rows(), 1);

		// temp
		Eigen::MatrixBase<Derived5> & sol = const_cast<Eigen::MatrixBase<Derived5> &>(sol_);
		sol.derived().resize(SubsysType::SYSTEM_DOF, 1 + J.rows());

		// computation begins...
		// for bias
		solve();
		_subsys.kinematics(J, Jdot, b);

		// temp
		sol.col(0) << _subsys.Vdot(), _subsys.qddot();
		
		for (int k = 0; k < J.rows(); k++)
		{
#ifdef __GNUC__
			_subsys.F() += J.row(k).template head<6>();

			if (SubsysType::JOINT_DOF > 0)
				_subsys.tau() += J.row(k).template tail<SubsysType::JOINT_DOF>();
#else
			_subsys.F() += J.row(k).head<6>();

			if (SubsysType::JOINT_DOF > 0)
				_subsys.tau() += J.row(k).tail<SubsysType::JOINT_DOF>();
#endif
			
			solve();

			_subsys.kinematics(J, Jdot, A.col(k));
			A.col(k) -= b;

			// temp
			sol.col(k + 1) << _subsys.Vdot(), _subsys.qddot();
			sol.col(k + 1) -= sol.col(0);

#ifdef __GNUC__
			_subsys.F() -= J.row(k).template head<6>();
			if (SubsysType::JOINT_DOF > 0)
				_subsys.tau() -= J.row(k).template tail<SubsysType::JOINT_DOF>();
#else
			
			_subsys.F() -= J.row(k).head<6>();
			if (SubsysType::JOINT_DOF > 0)
				_subsys.tau() -= J.row(k).tail<SubsysType::JOINT_DOF>();
#endif
		}
	}

	/*
	template<typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
	inline void incrementEffectiveDynamics(int cdim, int idim, Eigen::MatrixBase<Derived1> const & J, Eigen::MatrixBase<Derived2> const & Jdot, 
		Eigen::MatrixBase<Derived3> const & A_, Eigen::MatrixBase<Derived4> const & b_,
		Eigen::MatrixBase<Derived5> const & sol_) 
	{
		LieGroup::Wrench F = _subsys.F();
		typename SubsysType::JointVec tau = _subsys.tau();

		// resize
		Eigen::MatrixBase<Derived3> & A = const_cast<Eigen::MatrixBase<Derived3> &>(A_);
		Eigen::MatrixBase<Derived4> & b = const_cast<Eigen::MatrixBase<Derived4> &>(b_);

		//A.derived().resize(J.rows(), J.rows());
		//b.derived().resize(J.rows(), 1);
		A.derived().conservativeResize(cdim + idim, cdim + idim);
		b.derived().conservativeResize(cdim + idim);

		// temp
		Eigen::MatrixBase<Derived5> & sol = const_cast<Eigen::MatrixBase<Derived5> &>(sol_);
		//sol.derived().resize(SubsysType::SYSTEM_DOF, 1 + J.rows());
		sol.derived().conservativeResize(Eigen::NoChange, 1 + cdim + idim);

		// computation begins...
		// for bias
		//_update();
		//_subsys.kinematics(J, Jdot, b);

		_subsys.Vdot() = sol.col(0).head<6>();
		if (SubsysType::JOINT_DOF > 0)
			_subsys.qddot() = sol.col(0).tail<SubsysType::JOINT_DOF>();

		_subsys.kinematics(J.bottomRows(idim), Jdot.bottomRows(idim), b.tail(idim));

		// temp
		//sol.col(0) << _subsys.Vdot(), _subsys.qddot();

// 		for (int k = 0; k < cdim; k++)
// 		{
// 			// 			LieGroup::Wrench Ftest(F);
// 			// 			typename SubsysType::JointVec tautest(tau);
// 
// 			//_subsys.F() += J.row(k).head<6>();
// 			//if (SubsysType::JOINT_DOF > 0)
// 			//	_subsys.tau() += J.row(k).tail<SubsysType::JOINT_DOF>();
// 			//
// 			//_update();
// 			//_subsys.kinematics(J, Jdot, A.col(k));
// 			// A.col(k) -= b;
// 
// 			_subsys.Vdot() = sol.col(k + 1).head<6>();
// 			if (SubsysType::JOINT_DOF > 0)
// 				_subsys.qddot() = sol.col(k + 1).tail<SubsysType::JOINT_DOF>();
// 
// 			_subsys.kinematics(J.bottomRows(idim), Jdot.bottomRows(idim), A.col(k).tail(idim));
// 			A.col(k).tail(idim) -= b.tail(idim);
// 
// 			//_subsys.F() -= J.row(k).head<6>();
// 			//if (SubsysType::JOINT_DOF > 0)
// 			//	_subsys.tau() -= J.row(k).tail<SubsysType::JOINT_DOF>();
// 		}

		for (int k = cdim; k < cdim + idim; k++)
		{
			// 			LieGroup::Wrench Ftest(F);
			// 			typename SubsysType::JointVec tautest(tau);

			_subsys.F() += J.row(k).head<6>();
			if (SubsysType::JOINT_DOF > 0)
				_subsys.tau() += J.row(k).tail<SubsysType::JOINT_DOF>();

			_update();

			_subsys.kinematics(J, Jdot, A.col(k));
			A.col(k) -= b;

			// temp
			sol.col(k + 1) << _subsys.Vdot(), _subsys.qddot();
			sol.col(k + 1) -= sol.col(0);

			_subsys.F() -= J.row(k).head<6>();
			if (SubsysType::JOINT_DOF > 0)
				_subsys.tau() -= J.row(k).tail<SubsysType::JOINT_DOF>();
		}

		A.bottomLeftCorner(idim, cdim) = A.topRightCorner(cdim, idim).transpose();
	}
	*/

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! updates the system acceleration
	//  ----------------------------------------------------------
	inline void solve()
	{
		double const * const q = _subsys.q().data();
		double const * const qdot = _subsys.qdot().data();
		/// FIXME@20150915
		double * const torque = _subsys.tau().data();
		double * const qddot = _subsys.qddot().data();

		//LieGroup::Wrench const & F = _subsys.F();
		//LieGroup::Twist & Vdot = _subsys.Vdot();

		int i = 0;
		// initialize AB inertia and bias
		for (volatile int k = 0; k < NUM_BODIES; k++)
		{
			Body & b = _subsys.body(k);
			b.dynamics(_ABinertia[k], _ABbias[k]);

			Joint const & j = _subsys.joint(k);

			/// FIXME@20150915
			if (j.kinematic())
				_c[k] = j.biasAcc(b.V()) + j.adjAcc(qdot + i, qddot + i);
			else
				_c[k] = j.biasAcc(b.V()) + j.adjAcc(qdot + i, NULL);

			i += j.dof();
		}

		// modified @20131002
		i = JOINT_DOF; // SubsysType::TOTAL_JOINT_DOF;
		for (volatile int k = NUM_BODIES - 1; k >= 0; --k)
		{
			//Body const & b = _subsys.body(k);
			Joint const & j = _subsys.joint(k);


			/// FIXME@20150915
			if (j.kinematic())
			{
				i -= j.dof();

				if (_subsys.hasParent(k))
				{
					Matrix6d Aa = _ABinertia[k];
					LieGroup::Wrench ba = _ABbias[k] + Aa*_c[k];

					int p = _subsys.parent(k);
					_ABinertia[p] += j.Tadj().congruence(Aa);
					_ABbias[p] += j.Tadj().transform(ba);
				}
			}
			//////////////////////////////////////////////////////////////////////////
			else
			{
				//_U[k].resize(j.jdof(), 6);
				SqMatrix_jdof Lambda(j.dof(), j.dof());
				j.project(_ABinertia[k], _U[k].data(), Lambda.data());

				//_D[k].resize(j.dof(), j.dof());
				// FIXME Use a fixed-size matrix for _D[k]
				switch (j.dof())
				{
				case 1:
					Eigen::internal::compute_inverse<SqMatrix_jdof, SqMatrix_jdof, 1>::run(Lambda, _D[k]);
					break;

				case 2:
					Eigen::internal::compute_inverse<SqMatrix_jdof, SqMatrix_jdof, 2>::run(Lambda, _D[k]);
					break;

				case 3:
					Eigen::internal::compute_inverse<SqMatrix_jdof, SqMatrix_jdof, 3>::run(Lambda, _D[k]);
					break;

				case 4:
					Eigen::internal::compute_inverse<SqMatrix_jdof, SqMatrix_jdof, 4>::run(Lambda, _D[k]);
					break;

				case 5:
				case 6:
					_D[k].setIdentity();
					Lambda.llt().solveInPlace(_D[k]);
					break;
				}

				//_u[k].resize(j.dof());
				j.idyn(_ABbias[k], _u[k].data());

				i -= j.dof();
				Eigen::Map<const Vector_jdof> tau_k(torque + i, j.dof());
				_u[k] = tau_k - _u[k];

				if (_subsys.hasParent(k))
				{
					Matrix6d Aa = _ABinertia[k];
					Aa -= _U[k].transpose()*_D[k].template selfadjointView<Eigen::Upper>()*_U[k];

					LieGroup::Wrench ba = _ABbias[k] + Aa*_c[k] + _U[k].transpose()*_D[k]*_u[k];

					int parent = _subsys.parent(k);
					_ABinertia[parent] += j.Tadj().congruence(Aa);
					_ABbias[parent] += j.Tadj().transform(ba);
				}

				// FIXME @20130608: Added joint damping/stiffness algorithm
				// The reason why this inversion is repeated again here is that
				// the joint inertia induced by the joint damping should not be
				// propagated to the proximal joint.
				// In order not to engage this algorithm, just set _delT = 0,
				// or do not call setPeriod().
				if (_delT > 0)
				{
					Lambda.diagonal() += _delT*Eigen::Map<const Vector_jdof>(j.damping(), j.dof());
					/// FIXME	@20150807
					if (j.flexibleJoint())
					{
						for (int p = 0; p < j.dof(); p++)
							Lambda.diagonal()[p] += j.flexibleJoint()[p].addedInertia();
					}

					switch (j.dof())	
					{
					case 1:
						Eigen::internal::compute_inverse<SqMatrix_jdof, SqMatrix_jdof, 1>::run(Lambda, _D[k]);
						break;

					case 2:
						Eigen::internal::compute_inverse<SqMatrix_jdof, SqMatrix_jdof, 2>::run(Lambda, _D[k]);
						break;

					case 3:
						Eigen::internal::compute_inverse<SqMatrix_jdof, SqMatrix_jdof, 3>::run(Lambda, _D[k]);
						break;

					case 4:
						Eigen::internal::compute_inverse<SqMatrix_jdof, SqMatrix_jdof, 4>::run(Lambda, _D[k]);
						break;

					case 5:
					case 6:
						_D[k].setIdentity();
						Lambda.llt().solveInPlace(_D[k]);
						break;
					}
				}
			}
		}

		/////////////////////
		Joint const & j = _subsys.joint(0);

		_Vdot[0] = _c[0];
		_Vdot[0].v() -= j.Tadj().R().transpose()*_gacc;

		Eigen::Map<Eigen::Matrix<double, 0, 1> > qddot_0(qddot);
		qddot_0 = _D[0]*(_u[0] - _U[0]*_Vdot[0]);

		_Vdot[0] += j.adjAcc(NULL, qddot);

		i = 0;
		//////

		for (volatile int k = 1; k < NUM_BODIES; k++)
		{
			Joint const & j = _subsys.joint(k);

			/// FIXME@20150915
			if (j.kinematic())
			{
				_Vdot[k] = j.Tadj().itransform(_Vdot[_subsys.parent(k)]) + _c[k];

				LieGroup::Wrench Fk = _ABinertia[k]*_Vdot[k] + _ABbias[k];
				j.idyn(Fk, torque + i);
			}
			//////////////////////////////////////////////////////////////////////////
			else
			{
				Eigen::Map<const Vector_jdof> q_k(q + i, j.dof());
				Eigen::Map<const Vector_jdof> qdot_k(qdot + i, j.dof());
				Eigen::Map<const Vector_jdof> tau_k(torque + i, j.dof());
				Eigen::Map<Vector_jdof> qddot_k(qddot + i, j.dof());

				_Vdot[k] = j.Tadj().itransform(_Vdot[_subsys.parent(k)]) + _c[k];

				// FIXME @20130608: Added joint damping/stiffness algorithm
				if (_delT > 0)
				{
					Eigen::Map<const Vector_jdof> q0(j.neutralPos(), j.dof());
					Vector_jdof delq = q0 - q_k;

					Eigen::Map<const Vector_jdof> K(j.stiffness(), j.dof());
					Eigen::Map<const Vector_jdof> D(j.damping(), j.dof());
					Vector_jdof Dtilde = D + _delT*K;

					/// FIXME @20150807
					//qddot_k = _D[k]*(_u[k] - _U[k]*_Vdot[k] - Dtilde.cwiseProduct(qdot_k) + K.cwiseProduct(delq));

					qddot_k = _u[k] - _U[k]*_Vdot[k] - Dtilde.cwiseProduct(qdot_k) + K.cwiseProduct(delq);

				}
				else
				{
					/// FIXME @20150807
					//qddot_k = _D[k]*(_u[k] - _U[k]*_Vdot[k]);
					qddot_k = _u[k] - _U[k]*_Vdot[k];
				}

				if (j.flexibleJoint())
				{
					for (int p = 0; p < j.dof(); p++)
						// For flexible joint, tau_k stands for the motor torque, not the joint torque
						qddot_k[p] += j.flexibleJoint()[p].addedTorque(q_k[p], qdot_k[p], tau_k[p]) - tau_k[p];  // -tau_k to compensate the term in u_k
				}

				/// FIXME @20151022
				// Added joint friction
				Vector_jdof tau_fric(j.dof());
				/// FIXME @20151107
				tau_fric.setZero();

				j.frictionTorque(q + i, qdot + i, torque + i, tau_fric.data());

				qddot_k -= tau_fric;
				//////////////////////////////////////////////////////////////////////////

				qddot_k = _D[k]*qddot_k;
				//////////////////////////////////////////////////////////////////////////

				_Vdot[k] += j.adjAcc(NULL, qddot + i);
			}

			i += j.dof();
		}

		_subsys.Vdot() = _Vdot[0];
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	using ForwardDynamics<SubsysType, ABMethod<SubsysType> >::_subsys;
	using ForwardDynamics<SubsysType, ABMethod<SubsysType> >::_delT;
	using ForwardDynamics<SubsysType, ABMethod<SubsysType> >::_gacc;
	
	Matrix6d _ABinertia[NUM_BODIES];
	LieGroup::Wrench _ABbias[NUM_BODIES];

	Matrix_jdof_6 _U[NUM_BODIES];	// U_k = E_k^T * ABinertia[k]
	SqMatrix_jdof _D[NUM_BODIES];	// D_k = inv(E^T A E)
	Vector_jdof _u[NUM_BODIES];		// u_k = tau_k - E^T*b[k]
	Vector6d _c[NUM_BODIES];

	LieGroup::Twist _Vdot[NUM_BODIES] ;	
};

} // namespace NRMKFoundation
