//  ---------------------- Doxygen info ----------------------
//! \file Body.h
//!
//! \brief
//! Header file for the class Body (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a rigid body class
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
//!	\author Jonghoon Park, <crossover69@gmail.com>
//!	
//!
//! \note Copyright (C) 2013 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#pragma once

#include "LieGroup/LieGroup.h"
#include "Inertia.h"

namespace NRMKFoundation
{
namespace internal
{
	
}

//  ---------------------- Doxygen info ----------------------
//! \class Body
//!
//! \brief
//! This implements a rigid body class
//! to be used for the interface of NRMKFoundation library
//! 
//! \note
//! Bodies have the (unique) body frame, whose homogeneous transformation and twist defines the state.
//! Additionally, external body wrench can move the body. 
//! Since dynamic body should have nonzero body inertia, one has to add body inertia after instancing a body object.
//! 
//! \sa Inertia
//  ----------------------------------------------------------
class Body
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! Mode to specify the applied wrench to the body
	//! 
	//! \details
	//! This determines how the external body wrench is specified. In particular,
	//! it is set as the last argument to addExtWrench(F, at, mode).
	//! 
	//! \sa void addExtWrench(LieGroup::Wrench const & F, LieGroup::HTransform & at, APPLY_WRENCH_REF_FRAME_MODE mode = BF) 
	//  ----------------------------------------------------------
	enum APPLY_WRENCH_REF_FRAME_MODE 
	{
		BF = 0,  //!< The specified wrench is a body wrench.
		GF = 1	 //!< The specified wrench is a spatial wrench.
	};

public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructor a body
	//!
	//! \details
	//! The body is stationary at the origin of the global coordinate frame and aligned with its axis. 
	//! It has zero inertia.
	//  ----------------------------------------------------------
	inline Body() : _HT(), _V(), _Fext(), _hasNonzeroWrench(false), _inertia() 
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a body with the initial body transform
	//!
	//! \details
	//! The body is stationary at the specified transform.
	//! It has zero inertia.
	//!
	//! \param T0  Initial body transform (relative to the global reference frame)
	//!
	//! \sa Liegroup::HTransform
	//  ----------------------------------------------------------
	inline Body(LieGroup::HTransform const & T0) : _HT(T0), _V(), _Fext(), _hasNonzeroWrench(false), _inertia() 
	{
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs a body with the initial body transform and twist
	//!
	//! \details
	//! It has zero inertia.
	//!
	//! \param T0 Initial body transform
	//! \param V0 Initial body twist
	//!
	//! \sa Liegroup::HTransform
	//! \sa Liegroup::Twist
	//  ----------------------------------------------------------
	inline Body(LieGroup::HTransform const & T0, LieGroup::Twist const & V0) 
		: _HT(T0), _V(V0), _Fext(), _hasNonzeroWrench(false), _inertia() 
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the current body transform for reading
	//! 
	//! \return Body transform
	//!
	//! \sa Liegroup::HTransform
	//  ----------------------------------------------------------
	inline LieGroup::HTransform const & T() const 
	{ 
		return _HT; 
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the current body twist for reading
	//! 
	//! \return Body twist
	//!
	//! \sa Liegroup::Twist
	//  ----------------------------------------------------------
	inline LieGroup::Twist const & V() const 
	{ 
		return _V; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the current body transform for writing
	//! 
	//! \return Body transform
	//!
	//! \sa Liegroup::HTransform
	//  ----------------------------------------------------------
	inline LieGroup::HTransform & T() 
	{ 
		return _HT; 
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the current body twist for writing
	//! 
	//! \return Body twist
	//!
	//! \sa Liegroup::Twist
	//  ----------------------------------------------------------
	inline LieGroup::Twist & V() 
	{ 
		return _V; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the current body wrench for reading
	//! 
	//! \note 
	//! In order to set the body wrench, use addExtWrench() or addExtWrench(). 
	//! There is no version of Fext() for writing.
	//!
	//! \return Body wrench
	//!
	//! \sa Liegroup::Wrench
	//  ----------------------------------------------------------
	inline LieGroup::Wrench const & Fext() const 
	{ 
		return _Fext; 
	}

	//inline LieGroup::Wrench & Fext() { return _Fext; }

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the flag whether nonzero wrench is applied to the body
	//!
	//! \return true if the body's wrench is nonzero
	//  ----------------------------------------------------------
	inline bool hasNonzeroWrench() const 
	{ 
		return _hasNonzeroWrench; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! clears the body wrench
	//  ----------------------------------------------------------
	inline void clearExtWrench() 
	{ 
		_Fext.setZero(); 
		_hasNonzeroWrench = false; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the external body wrench at the body frame
	//! 
	//! \param F External body wrench
	//  ----------------------------------------------------------
	inline void setExtWrench(LieGroup::Wrench const & F) 
	{ 
		_Fext = F; 
		_hasNonzeroWrench = true;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the external body wrench at the specified frame
	//!
	//! \param F External body wrench
	//! \param T Homogeneous transformation of the body frame (relative to the body frame) where the body wrench is applied
	//  ----------------------------------------------------------
	inline void setExtWrench(LieGroup::Wrench const & F, LieGroup::HTransform const & T) 
	{
		setExtWrench(F.applyAt(T));
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! adds the external body wrench at the body frame
	//! 
	//! \param F External body wrench
	//  ----------------------------------------------------------
	inline void addExtWrench(LieGroup::Wrench const & F) 
	{ 
		_Fext += F; 
		_hasNonzeroWrench = true;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! adds the external body wrench at the specified frame
	//!
	//! \param F External body wrench
	//! \param T Homogeneous transformation of the body frame (relative to the body frame) where the body wrench is applied
	//  ----------------------------------------------------------
	inline void addExtWrench(LieGroup::Wrench const & F, LieGroup::HTransform const & T) 
	{
		addExtWrench(F.applyAt(T));
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! adds the external body wrench at the specified frame
	//!
	//! \param F External body wrench
	//! \param r Displacement of the point (relative to the body frame) where the body wrench is applied
	//  ----------------------------------------------------------
	inline void addExtWrench(LieGroup::Wrench const & F, LieGroup::Displacement const & r) 
	{
		addExtWrench(F.applyAt(r));
	}

	/// ADDED @20160801
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the current inertial parameters 
	//!
	//! \sa Inertia
	//  ----------------------------------------------------------
	inline Inertia const & getInertia()
	{
		return _inertia; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! resets the body inertia 
	//! 
	//! \param inertia Instance of class inertia
	//!
	//! \sa Inertia
	//  ----------------------------------------------------------
	inline void setInertia(Inertia const & inertia)
	{
		_inertia.set(inertia);
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! adds the body inertia to the current one
	//! 
	//! \param inertia Instance of class inertia
	//!
	//! \sa Inertia
	//  ----------------------------------------------------------
	inline void addInertia(Inertia const & inertia)
	{
		_inertia.addInertia(inertia);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! adds the body inertia at the relative body frame to the current one 
	//!
	//! \param inertia Instance of the class inertia
	//! \param T Homogeneous transformation of the added inertia frame (relative to the body frame)
	//!
	//! \sa Inertia
	//! \sa LieGroup::HTransform
	//  ----------------------------------------------------------
	inline void addInertia(Inertia const & inertia, LieGroup::HTransform const & T)
	{
		_inertia.addInertia(inertia, T);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the inverse dynamics of the body to realize the specified body acceleration
	//!
	//! \details
	//! This computes the inverse dynamics of the body to realize the specified body acceleration
	//! and stores the wrench to the second argument F.
	//! 
	//! \param Vdot Body acceleration
	//! \param F Inverse dynamics wrench
	//!
	//! \sa LieGroup::Twist
	//! \sa LieGroup::Wrench
	//  ----------------------------------------------------------
	inline void idyn(LieGroup::Twist const & Vdot, LieGroup::Wrench & F) const
	{
		F = inertia()*Vdot + bias(); // = A*Vdot - V().adjoint().transpose()*A*V();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the passivity-based inverse dynamics of the body 
	//! to realize the specified body acceleration and the reference body twist
	//!
	//! \details
	//! This computes the inverse dynamics of the body to realize the specified body acceleration
	//! and stores the wrench to the last argument F.
	//! 
	//! \param V Body reference velocity
	//! \param Vdot Body acceleration
	//! \param F Inverse dynamics wrench
	//!
	//! \sa LieGroup::Twist
	//! \sa LieGroup::Wrench
	//  ----------------------------------------------------------
	inline void idyn_passive(LieGroup::Twist const & V, LieGroup::Twist const & Vdot, LieGroup::Wrench & F) const
	{
		F = inertia()*Vdot + bias_passive(V); // = A*Vdot - V().adjoint().transpose()*A*V();
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes the dynamics 
	//!
	//! \param A body inertia
	//! \param b body bias including external wrench
	//!
	//! \sa LieGroup::Wrench
	//  ----------------------------------------------------------
	inline void dynamics(Matrix6d & A, LieGroup::Wrench & b) const
	{
		A = inertia();

		b =  V().coadjoint(A*V()); 
		if (_hasNonzeroWrench)
			b -= _Fext; // _Fext;
		
		b += addedBias();		
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes and returns the passivity-based bias wrench except inertial acceleration
	//!
	//! \param V Body reference velocity
	//!
	//! \return Inverse dynamics bias wrench
	//!
	//! \sa LieGroup::Wrench
	//  ----------------------------------------------------------
	inline void dynamics_passive(LieGroup::Twist const & V, Matrix6d & A, LieGroup::Wrench & b) const
	{
		A = inertia();
		b = A*_V.adjoint(V) + _V.coadjoint(A*V); 

		if (_hasNonzeroWrench)
			b -= _Fext; 

		b += addedBias();		
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes and returns the bias wrench except inertial acceleration
	//!
	//! \return Inverse dynamics bias wrench
	//!
	//! \note 
	//! Calling bias() evoke inertia() inside. 
	//! So the following code amounts to calling inertia() twice.
	//! When inertia() needs heavy computation, you had better call dynamics(). 
	//! The same applies to bias_passive().
	//!
	//! \code{.cpp}
	//! Body body; 
	//! ...
	//! Matrix6d A = body.inertia(); 
	//! LieGroup::Wrench b = body.bias(); 
	//! \endcode
	//!
	//! \sa LieGroup::Wrench
	//  ----------------------------------------------------------
	inline LieGroup::Wrench bias() const
	{
		//LieGroup::Wrench b = V().coadjoint(_inertia.bodyInertia()*V()); 
		LieGroup::Wrench b = V().coadjoint(inertia()*V()); 
		
		if (_hasNonzeroWrench)
			b -= _Fext; // _Fext;

		b += addedBias();		

		return b;	
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! computes and returns the passivity-based bias wrench except inertial acceleration
	//!
	//! \param V Body reference velocity
	//!
	//! \return Inverse dynamics bias wrench
	//!
	//! \sa LieGroup::Wrench
	//  ----------------------------------------------------------
	inline LieGroup::Wrench bias_passive(LieGroup::Twist const & V) const
	{
		LieGroup::Wrench b = inertia()*_V.adjoint(V) + _V.coadjoint(inertia()*V); 

		if (_hasNonzeroWrench)
			b -= _Fext; // _Fext;

		b += addedBias();		

		return b;	
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the body inertia matrix for reading
	//!
	//! \return Body inertia matrix of size 6-by-6
	//!
	//! \sa Matrix6d
	//! \sa Inertia::bodyInertia()
	//  ----------------------------------------------------------
	inline virtual Matrix6d inertia() const 
	{ 
		return _inertia.bodyInertia(); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the additional body bias
	//!
	//! \return Body bias wrench
	//  ----------------------------------------------------------
	inline virtual LieGroup::Wrench addedBias() const
	{
		return LieGroup::Wrench(); 
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! set the external wrench. By default it is cleared.
	//  ----------------------------------------------------------
	inline virtual void updateExtWrench()
	{	
		// If you call clearExtWrench here, every external wrench you set before ForwardDynamics is cleared.
		//	So don't call.
		// clearExtWrench();
	}

// 	//  ---------------------- Doxygen info ----------------------
// 	//! \brief
// 	//! set the external wrench. By default it is cleared.
// 	//  ----------------------------------------------------------
// 	inline virtual void updateExtWrenchByEffectiveDynamics(Matrix6d const & A, Vector6d const & b)
// 	{	
// 		// If you call clearExtWrench here, every external wrench you set before ForwardDynamics is cleared.
// 		//	So don't call.
// 		// clearExtWrench();
// 	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! set the custom dynamics.
	//  ----------------------------------------------------------
	inline virtual void updateCustomDynamics(double t) 
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief returns the body mass
	//  ----------------------------------------------------------
	double mass() const { return _inertia.m(); }

	//  ---------------------- Doxygen info ----------------------
	//! \brief returns the array containing the COM position
	//  ----------------------------------------------------------
	double const * const com() const { return _inertia.com(); }
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief returns the array containing the rotational inertia
	//  ----------------------------------------------------------
	double const * const rot_inertia() const { return _inertia.inertia(); }

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Current body transform
	//  ----------------------------------------------------------
	LieGroup::HTransform _HT;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Current body twist
	//  ----------------------------------------------------------
	LieGroup::Twist _V;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Current body wrench
	//  ----------------------------------------------------------
	LieGroup::Wrench _Fext;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Flag whether the body has nonzero wrench
	//  ----------------------------------------------------------
	bool _hasNonzeroWrench;

	//  ---------------------- Doxygen info ----------------------
	//! \brief Current body inertia object
	//  ----------------------------------------------------------
	Inertia _inertia;
};	// class Body

} // namespace NRMKFoundation