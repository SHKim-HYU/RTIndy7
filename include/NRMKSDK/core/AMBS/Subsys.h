//! \file Subsys.h
//! \brief Header file for the class Subsys (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements an articulated multibody system class
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
//! \note 
//!  - v2.0.0(20170117) : initial implementation
//!
//! \note Copyright (C) 2013--2017 Neuromeka

#pragma once

#include "LieGroup/LieGroup.h"

#include "Joint.h"
#include "Body.h"

namespace NRMKFoundation
{

namespace internal
{
	//! \class Topology
	//!
	//! \brief
	//! This implements a topology class to describe the connectivity of the system.
	struct Topology
	{
		enum {NUM_BODIES = 7};

		enum TOPOLOGY_MODE { UNINITIALIZED = -1, SERIAL_CHAIN = 0 };

		inline Topology(TOPOLOGY_MODE mode = UNINITIALIZED)
		{
			if (mode == UNINITIALIZED)
			{
				memset(_pre, -1, NUM_BODIES*sizeof(int));
				memset(_suc, 0, NUM_BODIES*sizeof(int));
				memset(_lambda, -1, NUM_BODIES*sizeof(int));
			}
			else if (mode == SERIAL_CHAIN)
			{
				_pre[0] = -1; 
				_suc[0] = 0;
				_lambda[0] = -1;

				for (int j = 1; j < NUM_BODIES; ++j)
				{
					_pre[j] = j - 1; 
					_suc[j] = j;
					_lambda[j] = _pre[j]; 
				}
			}
			else 
			{
				assert(false && "Invalid topology mode: Use 0 or 1...");
			}
		}

		inline bool connect(int p, int s) 
		{
			_pre[s] = p;
			_suc[s] = s;
			if (s < NUM_BODIES)
				_lambda[s] = _pre[s];

			return true;
		}

		inline int predecessor(int j) const { return _pre[j]; }
		inline int successor(int j) const { return _suc[j]; }

		// FIXME Implement the following function
		bool isValid() const { return true; }

	private:
		// FIXED @20140222
		int _pre[NUM_BODIES];		// The body of index '-1' stands for the ground.
		int _suc[NUM_BODIES];		// The joint of index '-1' stands for the free joint for the base body.
		// FIXED @20131003
		int _lambda[NUM_BODIES];	// IMPORTANT: If this goes ahead of _pre, the code does not work in ARM GCC Compiler.
	};
}

//! \class SubsysBase
//!
//! \tparam _NUM_BODIES Number of bodies 
//! \tparam _NUM_JOINTS Number of joints (excluding the joint to connect the base body) 
//! \tparam _JOINT_DOF Degrees-of-freedom of the joints (excluding the joint to connect the base body)
//!
//! \brief
//! This implements an articulated multibody system (AMBS) class.
//! It consists of a set of bodies, another set of joints, and 
//! the connectivity describing Topology object. Since the system is described by
//! tree-structured graph, it always has a unique base body which is the root of the tree. 
//! 
//! \details 
//! The state of an AMBS consists of the joint position vector \f$ q \f$ and the velocity vector \f$ \dot{q} \f$
//! as well as the base transformation \f$ T \f$ and the base twist \f$ V \f$. The base transformation and base twist 
//! described the base body. In presence of any earthing joint (which connects the base body to the ground), 
//! the system is called grounded. For grounded systems
//! the base transformation and twist are no more states, since they are described by the earthing joint.
//! Then the system state consists of the joint position vector \f$ q \f$ and the velocity vector \f$ \dot{q} \f$
//! where the earthing joint variable always comes first in these vectors.
//!
//! One can affect the system dynamics by applying the joint torques \f$ \tau \f$ and the base wrench \f$ F \f$.
//! Base wrench is the wrench applied to the base body. As a result, the system's acceleration changes consisting 
//! of the joint acceleration vector \f$ \ddot{q} \f$ and the base acceleration \f$ \dot{V} \f$. The latter is the 
//! body acceleration of the base body.
//!
//! \sa Topology
template<int _NUM_BODIES, int _NUM_JOINTS = _NUM_BODIES - 1, int _JOINT_DOF = _NUM_JOINTS>
class SubsysBase
{
public:
	//! \brief Provides compile-time constants of the subsys
	enum
	{
		NUM_BODIES = _NUM_BODIES, //!< Number of bodies
		NUM_JOINTS = _NUM_JOINTS, //!< Number of joints except the earthing joint
		NUM_TOTAL_JOINTS = NUM_JOINTS + 1, //!< Number of total joints including the earthing joint
		JOINT_DOF = _JOINT_DOF, //!< Total degrees-of-freedom of the system including the earthing joint
		// added @20131002
		SYSTEM_DOF = _JOINT_DOF, //!< System degrees-of-freedom including the earthing joint dof
	};

	//! \brief Provides Typedefs 
	//typedef RigidJoint EarthJoint; //!< Typedef of the earth joint
	typedef Eigen::Matrix<double, JOINT_DOF, 1> JointVec; //!< Typedef of the vector having the dimension of the total joint dof
	typedef Eigen::Matrix<double, JOINT_DOF, JOINT_DOF> JointMat; //!< Typedef of the square matrix having the dimension of the total joint dof

	// added @20131002
	//typedef Eigen::Matrix<double, SYSTEM_DOF, 1> SystemVec; //!< Typedef of the vector having the dimension of the system dof
	//typedef Eigen::Matrix<double, SYSTEM_DOF, SYSTEM_DOF> SystemMat; //!< Typedef of the square matrix having the dimension of the system dof
	
	// modified @20131002
	typedef Eigen::Matrix<double, 6, SYSTEM_DOF> JointJac; //!< Typedef of the jacobian matrix having the dimension of six-by-system dof

	/// ADDED @20160311
	typedef typename internal::IndexSet<JOINT_DOF>::Type  JointIndex;	//!< Joint index vector
	typedef Eigen::Matrix<double, Eigen::Dynamic, 1, JOINT_DOF, 1> VarJointVec; //!< Typedef of the vector having the dimension of the total joint dof
	
public:
	//! \brief constructs the subsys
	inline SubsysBase() : _kinJointIndex() 
	{

#ifdef __GNUC__
		memset(_bodies, 0, NUM_BODIES*sizeof(Body *));			// FIXED by THACHDO 20170804
		memset(_joints, 0, (NUM_JOINTS + 1)*sizeof(Joint *));	// FIXED by THACHDO 20170804
#else
		memset(_bodies, NULL, NUM_BODIES*sizeof(Body *));
		memset(_joints, NULL, (NUM_JOINTS + 1)*sizeof(Joint *));
#endif

		_q.setZero();
		_qdot.setZero();
		_qddot.setZero();
		_tau.setZero();
	}
	
	//! \brief adds the a body to the system
	//! \param k Body index (nonnegative integer smaller than NUM_BODIES)
	//! \param body Body object (Note that the body object should persist.)
	//!
	//! \details
	//! This adds the \f$ k \f$-th body to the system, where 0-th body is the base. 
	//!
	//! \sa Body
	inline bool addBody(int k, Body & body) 
	{
		_bodies[k] = &body;

		return true;
	}

	//! \brief adds the joint to the system
	//! \param j Joint index (nonnegative integer smaller than NUM_JOINTS + 1)
	//! \param joint Joint object (Note that the joint object should persist.)
	//!
	//! \details This adds the \f$ j \f$-th joint to the system.
	//! 
	//! \sa Joint
	inline bool addJoint(int j, Joint & joint) 
	{
		_joints[j] = &joint;
		
		/// ADDED @20160313
		_configureJointIndex();

		return true;
	}

	//! \brief
	//! connects the parent body (of index p) and the child body (of index s) by the joint
	//! \param p Parent body index (nonnegative integer smaller than the child body index s)
	//! \param s Child body index (nonnegative integer smaller than NUM_BODIES)
	//! \param joint Joint object (Note that the joint object should persist.)
	//!
	//! \details
	//! The joint has the same index as the child body's, i.e. s.
	//!
	//! \sa Joint
	inline bool connect(int p, int s, Joint & j)
	{
		addJoint(s, j);

		return _topology.connect(p, s);
	}

	//! \brief
	//! connects the parent body (of index p) and the child body (of index s) by the joint 
	//! with the joint offset frame specified by T
	//! \param p Parent body index (nonnegative integer smaller than the child body index s)
	//! \param s Child body index (nonnegative integer smaller than NUM_BODIES)
	//! \param joint Joint object (Note that the joint object should persist.)
	//! \param T Joint offset frame
	//!
	//! \details
	//! The joint has the index same as the child body's, i.e. s.
	//! 
	//! \sa Joint
	//! \sa LieGroup::HTransform
	inline bool connect(int p, int s, Joint & j, LieGroup::HTransform const & T)
	{
		addJoint(s, j);
		joint(s).setJointFrame(T);

		return _topology.connect(p, s);
	}

	//! \brief returns the parent body index of a body
	//! \param s Child body index
	//! \return Parent body index
	inline int parent(int s) const
	{
		return _topology.predecessor(s);
	}

	//! \brief determines whether a body has a parent or not
	//! \param k body index
	//! \return true when the body has parent
	//!
	//! \note Only the base body does not have the parent. 
	inline bool hasParent(int k) const
	{
		return parent(k) >= 0;
	}
	
	//! \brief returns the base body
	//! \return Reference to the base body
	inline Body & base() 
	{ 
		return *_bodies[_topology.successor(0)]; 
	}
	
	//! \brief returns the base body
	//! \return Constant reference to the base body
	inline Body const & base() const 
	{
		return *_bodies[_topology.successor(0)];
	}

	//! \brief returns the k-th body
	//! \param k Body index
	//! \return Reference to the k-th body
	inline Body & body(int k) 
	{ 
		return *_bodies[k];
	}

	//! \brief returns the k-th body
	//! \param k Body index
	//! \return Constant reference to the k-th body
	inline Body const & body(int k) const 
	{ 
		return *_bodies[k]; 
	}

	//! \brief returns the k-th joint
	//! \param k Joint index
	//! \return Reference to the k-th joint
	inline Joint & joint(int k)
	{ 
		return *_joints[k];
	}

	//! \brief returns the k-th joint
	//! \param k Joint index
	//! \return Constant reference to the k-th joint
	inline Joint const & joint(int k) const 
	{ 
		return *_joints[k]; 
	}

	//! \brief determines whether the system is floating
	//! \return true when the system is floating
	inline bool isFloating() const 
	{ 
		// FIXME: Check this code.. @20130402
		//return !hasJoint(0); 
		return (_joints[0] == NULL || (_joints[0] && joint(0).type() == FLOATING));
	}

	//! \brief returns the jacobian of the k-th body
	//! \param k Body index
	//! 
	//! \noteIt should be called after updateJacobian() has been called.
	inline JointJac const & J(int k) const
	{
		return _J[k];
	}

	//! \brief returns the jacobian of the k-th body for writing
	//! \param k Body index
	//! 
	//! \note It should be called after updateJacobian() has been called.
	inline JointJac & J(int k) 
	{
		return _J[k];
	}

	//! \brief returns the body and joint jacobian derivative of the k-th body
	//! \param k Body index
	//! 
	//! \note It should be called after updateJacobian() has been called.
	inline JointJac const & Jdot(int k) const
	{
		return _Jdot[k];
	}

	//! \brief returns the body and joint jacobian derivative of the k-th body for writing
	//! \param k Body index
	//! 
	//! \note
	//! It should be called after updateJacobian() has been called.
	inline JointJac & Jdot(int k) 
	{
		return _Jdot[k];
	}

	//! \brief returns the base transformation for reading
	//! \return Base transformation
	inline LieGroup::HTransform const & T() const
	{
		return base().T();
	}

	//! \brief returns the base transformation for writing
	//! \return Base transformation
	inline LieGroup::HTransform & T() 
	{	
		// In case that the base body (of index 0) has rigid-joint connection
		// it writes the adjoint transformation of the joint
		return (!isFloating() && joint(0).type() == RIGID) ? joint(0).Tadj() : base().T();
	}

	//! \brief returns the base twist for reading
	//! \return Base twist
	inline LieGroup::Twist const & V() const
	{
		return base().V();
	}

	//! \brief returns the base twist for writing
	//! \return Base twist
	inline LieGroup::Twist & V() 
	{
		return base().V();
	}

	//! \brief returns the joint position vector for reading
	//! \return Joint position vector
	inline JointVec const & q() const
	{
		return _q;
	}

	//! \brief returns the joint position vector for writing
	//! \return Joint position vector
	inline JointVec & q() 
	{
		return _q;
	}

	//! \brief returns the joint velocity vector for reading
	//! \return Joint velocity vector
	inline JointVec const & qdot() const
	{
		return _qdot;
	}

	//! \brief returns the joint velocity vector for writing
	//! \return Joint velocity vector
	inline JointVec & qdot() 
	{
		return _qdot;
	}

	//! \brief returns the joint torque vector for reading
	//! \return Joint torque vector
	inline JointVec const & tau() const
	{
		return _tau;
	}

	//! \brief return the joint torque vector for writing
	//! \return Joint torque vector
	inline JointVec & tau() 
	{
		return _tau;
	}

	//! \brief returns the base (control) wrench for reading
	//! \return Base wrench
	inline LieGroup::Wrench const & F() const
	{
		return _F;
	}

	//! \brief returns the base (control) wrench for writing
	//! \return Base wrench
	inline LieGroup::Wrench & F() 
	{
		return _F;
	}

	//! \brief returns the joint acceleration vector for reading
	//! \return Joint acceleration vector
	inline JointVec const & qddot() const
	{
		return _qddot;
	}

	//! \brief returns the joint acceleration vector for writing
	//! \return Joint acceleration vector
	inline JointVec & qddot() 
	{
		return _qddot;
	}

	//! \brief returns the base body acceleration for reading
	//! \return Base body acceleration
	inline LieGroup::Twist const & Vdot() const
	{
		return _Vdot;
	}

	//! \brief returns the base body acceleration for writing
	//! \return Base body  acceleration
	inline LieGroup::Twist & Vdot() 
	{
		return _Vdot;
	}
	
	// added @20121002
	//! \brief returns the body transformation for reading
	//! \param k Body index
	//! \return Body transformation
	inline LieGroup::HTransform const & T(int k) const
	{
		return body(k).T(); 
	}

	//! \brief returns the body twist for writing	
	//! \param k Body index
	//! \return Body twist
	inline LieGroup::HTransform & T(int k)
	{
		return body(k).T(); 
	}

	//! \brief returns the body twist for reading
	//! \param k Body index
	//! \return Body twist
	inline LieGroup::Twist const & V(int k) const
	{
		return body(k).V(); 
	}

	//! \brief returns the body twist for writing
	//! \param k Body index
	//! \return Body twist
	inline LieGroup::Twist & V(int k)
	{
		return body(k).V(); 
	}

	//! \brief clears all body wrenches
	inline void clearAllBodyWrenches()
	{
		for (int k = 0; k < NUM_BODIES; k++)
			body(k).clearExtWrench();
	}

	/// ADDED @20160311
	//! \brief sets a joint (of index k) kinematic or not
	//! \param k Joint index
	//! \param kinematic true or false
	inline void setKinematicJoint(int k, bool kinematic = true) 
	{
		joint(k).setKinematic(kinematic);

		if (kinematic)
			_kinJointIndex.addIndex(k);
		else
			_kinJointIndex.removeIndex(k);
	}

	//! \return whether a joint (of index k) is kinematic or not
	inline bool isKinematicJoint(int k) 
	{
		return joint(k).kinematic();
	}

	/*
	template<typename Derived>
	inline void getKinematicJointIndexVector(Derived & iv) const
	{
		_kinJointIndex.getIndexVector(iv);
	}

	template<typename Derived>
	inline void getNonKinematicJointIndexVector(Derived & iv) const
	{
		_kinJointIndex.getComplementIndexVector(iv);
	}
	*/
	//! \return the total number of kinematic joints
	inline unsigned int kinematicJointSize() const
	{
		return _kinJointIndex.size();
	}
	
	//! \brief fills in components of a vector ('whole') with values from a reference vector ('in') at kinematic joint indices
	inline void fillInKinematicJointVector(JointVec & whole, JointVec const & in) 
	{
		for (int j = 0, k = 0; j < NUM_TOTAL_JOINTS; j++)
		{
			if (isKinematicJoint(j))
			{
				for (int i = 0; i < joint(j).dof(); i++)
				{
					int idx = joint(j).index() + i;
					whole[idx] = in[idx];
				}
			}
		}
	}

	/*
	inline void fillInKinematicJointVector(JointVec & whole1, JointVec & whole2, JointVec const & in1, JointVec const & in2) 
	{
		for (int j = 0, k = 0; j < NUM_TOTAL_JOINTS; j++)
		{
			if (isKinematicJoint(j))
			{
				for (int i = 0; i < joint(j).dof(); i++)
				{
					int idx = joint(j).index() + i;
					whole1[idx] = in1[idx];
					whole2[idx] = in2[idx];
				}
			}
		}
	}
	*/

	//! \brief fills in components of a vector ('whole') with values from a reference vector ('out') 'NOT' at kinematic joint indices
	template<typename DerivedJointVec>
	inline void fillOutKinematicJointVector(JointVec & whole, DerivedJointVec const & out) 
	{
		for (int j = 0, k = 0; j < NUM_TOTAL_JOINTS; j++)
		{
			if (!isKinematicJoint(j))
			{
				for (int i = 0; i < joint(j).dof(); i++)
				{
					int idx = joint(j).index() + i;
					whole[idx] = out[k++];
				}
			}
		}
	}

	//! \brief fills in components of a vector ('whole') 
	//!  with value 'in' at kinematic joint indices, and
	//!  with values from a reference vector ('out') 'NOT' at kinematic joint indices
	template<typename DerivedJointVec>
	inline void fillInKinematicJointVector(JointVec & whole, double in, DerivedJointVec const & out) 
	{
		for (int j = 0, k = 0; j < NUM_TOTAL_JOINTS; j++)
		{
			if (isKinematicJoint(j))
			{
				for (int i = 0; i < joint(j).dof(); i++)
				{
					int idx = joint(j).index() + i;
					whole[idx] = in;
				}
			}
			else
			{
				for (int i = 0; i < joint(j).dof(); i++)
				{
					int idx = joint(j).index() + i;
					whole[idx] = out[k++];
				}
			}
		}
	}

	//! \brief fills in components of a vector ('whole') 
	//!  with values from a reference vector ('in') at kinematic joint indices, and
	//!  with values from a reference vector ('out') 'NOT' at kinematic joint indices
	template<typename DerivedJointVec>
	inline void fillInKinematicJointVector(JointVec & whole, JointVec const & in, DerivedJointVec const & out) 
	{
		for (int j = 0, k = 0; j < NUM_TOTAL_JOINTS; j++)
		{
			if (isKinematicJoint(j))
			{
				for (int i = 0; i < joint(j).dof(); i++)
				{
					int idx = joint(j).index() + i;
					whole[idx] = in[idx];
				}
			}
			else
			{
				for (int i = 0; i < joint(j).dof(); i++)
				{
					int idx = joint(j).index() + i;
					whole[idx] = out[k++];
				}
			}
		}
	}

private:
	/// ADDED @20160311
	inline void _configureJointIndex() 
	{
		int index = 0;
		for (int j = 0; j < NUM_TOTAL_JOINTS; j++)
		{
			if (_joints[j])
			{
				joint(j).setIndex(index);
				index += joint(j).dof();
			}
		}
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Array of the pointers to the bodies constituting the subsys 
	//!
	//! \note The body objects should persist.
	//  ----------------------------------------------------------
	Body * _bodies[NUM_BODIES];

	//  ---------------------- Doxygen info ----------------------
	//! \brief Array of the pointers to the joints constituting the subsys
	//!
	//! \note The joint objects should persist. The number of elements 
	//! is one more than the NUM_JOINTS, which is used for the base body.
	//  ----------------------------------------------------------
	Joint * _joints[NUM_JOINTS + 1];	// 1 for the base body

	//Constraint _constraints[NUM_LOOPS];

	//  ---------------------- Doxygen info ----------------------
	//! \brief Array of the body Jacobians
	//  ----------------------------------------------------------
	// FIXME When the subsys is floating, ...
	JointJac _J[NUM_BODIES];

	//  ---------------------- Doxygen info ----------------------
	//! \brief Array of the body Jacobian derivatives
	//  ----------------------------------------------------------
	JointJac _Jdot[NUM_BODIES];

	//  ---------------------- Doxygen info ----------------------
	//! \brief Joint position vector
	//  ----------------------------------------------------------
	JointVec _q;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Joint velocity vector
	//  ----------------------------------------------------------
	JointVec _qdot;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Joint torque vector
	//  ----------------------------------------------------------
	JointVec _tau;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Base wrench
	//  ----------------------------------------------------------
	LieGroup::Wrench _F;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Joint acceleration vector
	//  ----------------------------------------------------------
	JointVec _qddot;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Base acceleration
	//  ----------------------------------------------------------
	LieGroup::Twist _Vdot;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Topology information
	//  ----------------------------------------------------------
	//	internal::Topology<SubsysBase> _topology;		
	internal::Topology _topology;

	/// ADDED @20160311
	internal::IndexSet<NUM_TOTAL_JOINTS> _kinJointIndex;

	/// ADDED @20170117
//	SubsysAlgorithm *_algorithm;
};

template<>
class SubsysBase<0>
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Provides compile-time constants of the subsys
	//  ----------------------------------------------------------
	enum
	{
		NUM_BODIES = 0, 
	};
};

} // namespace NRMKFoundation
