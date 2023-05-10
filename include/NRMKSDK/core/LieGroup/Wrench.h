//  ---------------------- Doxygen info ----------------------
//! \file Wrench.h
//!
//! \brief
//! Header file for the class Twist (API of the LieGroup Libraries)
//!
//! \details
//! This file implements a three-dimensional twist 
//! (6D vector comprised of linear and angular velocity) class
//! to be used for the interface of LieGroup library
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

#include "Eigen/Dense"

namespace LieGroup
{

class Vector3D;
class Rotation;
class HTransform;

//  ---------------------- Doxygen info ----------------------
//! \class Wrench
//!
//! \brief
//! This implements a three-dimensional wrench class which is a six-dimensional vector
//! cascading force and moment (both of type the three-dimensional vector).
//! 
//! \details
//! For two three-dimensional vector \f$ f \f$ and \f$ n \f$ representing 
//! the force and moment, respectively, the wrench is defined by 
//! \f$ F = \begin{bmatrix} f \\ n \end{bmatrix} \f$. 
//!
//! \note
//! Linear part always precedes angular part.
//! Compared to Vector6d, it is initialized properly.
//! Although it has same representation as the twist, two entities are physically different. 
//! 
//! \sa Vector6d
//  ----------------------------------------------------------
class Wrench : public Vector6d
{
public:
	RMATH_EIGEN_DERIVE(Vector6d, Wrench)

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the zero wrench
	//  ----------------------------------------------------------
	inline Wrench() : Vector6d(Vector6d::Zero()) 
	{
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the identity element
	//  ----------------------------------------------------------
	inline static Wrench const Zero() 
	{
		return Wrench();
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the wrench in terms of six elements by
	//! \f$ F = \begin{bmatrix} x \\ y \\ z \\ u \\ v \\ w \end{bmatrix} \f$.
	//!
	//! \param x x-component of the linear velocity
	//! \param y y-component of the linear velocity
	//! \param z z-component of the linear velocity
	//! \param u x-component of the angular velocity
	//! \param v y-component of the angular velocity
	//! \param w z-component of the angular velocity
	//  ----------------------------------------------------------
	inline Wrench(double x, double y, double z, double u, double v, double w) 
	{ 
		set(x, y, z, u, v, w); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the wrench in terms of force and moment vectors (both of type Vector3D)
	//! by \f$ F = \begin{bmatrix} f \\ n \end{bmatrix} \f$.
	//!
	//! \param f force vector
	//! \param n moment vector 
	//  ----------------------------------------------------------
	inline Wrench(Vector3D const & f, Vector3D const & n) 
	{ 
		set(f, n); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the wrench in terms of any three-dimensional vectors
	//! by \f$ V = \begin{bmatrix} f \\ n \end{bmatrix} \f$.
	//!
	//! \param f force vector
	//! \param n moment vector 
	//  ----------------------------------------------------------
	template<typename Derived1, typename Derived2>
	inline Wrench(Eigen::VectorBlock<Derived1, 3> const & f, Eigen::VectorBlock<Derived2, 3> const & n)
	{ 
		set(f, n);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the force vector block for writing
	//  ----------------------------------------------------------
	inline Eigen::VectorBlock<Vector6d, 3> f()
	{
		return head<3>(); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the force vector block for reading
	//  ----------------------------------------------------------
	inline const Eigen::VectorBlock<const Vector6d, 3> f() const 
	{ 
		return head<3>(); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the moment vector block for writing
	//  ----------------------------------------------------------
	inline Eigen::VectorBlock<Vector6d, 3> n() 
	{ 
		return tail<3>(); 
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the moment vector block for reading
	//  ----------------------------------------------------------
	inline const Eigen::VectorBlock<const Vector6d, 3> n() const
	{ 
		return tail<3>(); 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the wrench in terms of six elements by
	//! \f$ F = \begin{bmatrix} x \\ y \\ z \\ u \\ v \\ w \end{bmatrix} \f$.
	//!
	//! \param x x-component of the linear velocity
	//! \param y y-component of the linear velocity
	//! \param z z-component of the linear velocity
	//! \param u x-component of the angular velocity
	//! \param v y-component of the angular velocity
	//! \param w z-component of the angular velocity
	//  ----------------------------------------------------------
	inline void set(double x, double y, double z, double u, double v, double w) 
	{ 
		*this << x, y, z, u, v, w; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs the wrench in terms of force and moment vectors (both of type Vector3D)
	//! by \f$ F = \begin{bmatrix} f \\ n \end{bmatrix} \f$.
	//!
	//! \param f force vector
	//! \param n moment vector 
	//  ----------------------------------------------------------
	void set(Vector3D const & f, Vector3D const & n) 
	{ 
		head<3>() = f; 
		tail<3>() = n; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the wrench in terms of any three-dimensional vectors
	//! by \f$ V = \begin{bmatrix} f \\ n \end{bmatrix} \f$.
	//!
	//! \param f force vector
	//! \param n moment vector 
	//  ----------------------------------------------------------
	template<typename Derived1, typename Derived2>
	inline void set(Eigen::VectorBlock<Derived1, 3> const & f, Eigen::VectorBlock<Derived2, 3> const & n)
	{ 
		head<3>() = f; 
		tail<3>() = n; 
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the equivalent wrench applied at the coordinate frame specified by T 
	//! 
	//! \details
	//! It returns the wrench computed by \f$ {\rm Adj}_{T}^{-T} F \f$ for the wrench \f$ F \f$.
	//! The function is defined in HTransform.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Wrench F; 
	//! LieGroup::HTransform R;
	//! ...
	//! LieGroup::Wrench F2 = F.applyAt(T); // This sets F2 = T.tiadj()*[ f(); n() ]
	//! \endcode
	//!
	//! \param T The homogeneous transformation matrix
	//!
	//! \sa LieGroup::HTransform
	//  ----------------------------------------------------------
	Wrench applyAt(HTransform const & T) const;

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates the equivalent wrench applied at the point specified by r 
	//! 
	//! \details
	//! It returns the wrench computed by \f$ {\rm Adj}_{T}^{-T} F \f$ for the wrench \f$ F \f$,
	//! where \f$ T = \begin{bmatrix} I & r \\ 0 & 0 \end{bmatrix} \f$.
	//! The function is defined in HTransform.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Wrench F; 
	//! LieGroup::Displacement r;
	//! ...
	//! LieGroup::Wrench F2 = F.applyAt(r); // This sets F2 = [ I, r; 0, 0].tiadj()*[ f(); n() ]
	//! \endcode
	//!
	//! \param r The displacement vector
	//!
	//! \sa LieGroup::HTransform
	//  ----------------------------------------------------------
	Wrench applyAt(Displacement const & r) const;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! premultiplies the block-wise transpose of an lower block-triangular matrix and the wrench
	//! 
	//! \details
	//! The lower block-triangular matrix is defined by \f$ \begin{bmatrix} A & 0 \\ B & C \end{bmatrix} \f$ where each block is three-dimensiaonl 
	//! square matrix. Then, the block-wise transposed multiplication of this lower block-triangular matrix with the wrench is defined by 
	//! \f$ \begin{bmatrix} A^T f  \\  B^T f + C^T n \end{bmatrix} \f$ for \f$ V = \begin{bmatrix} f \\ n \end{bmatrix} \f$.
	//! This function is necessary to compute differential-of-exponential families for \f$ - \xi \f$.
	//! 
	//! \param LBT The six-dimensional lower block-triangular matrix
	//  ----------------------------------------------------------
	inline LieGroup::Wrench lowerBlockTriTransposedMult(Matrix6d const & LBT) const
	{
		LieGroup::Wrench res;

		res << LBT.topLeftCorner<3,3>().transpose()*f(),
			LBT.bottomLeftCorner<3,3>().transpose()*f() + LBT.bottomRightCorner<3,3>().transpose()*n();

		return res;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! premultiplies the block-wise relocated upper block-triangular matrix and the wrench
	//! 
	//! \details
	//! The upper block-triangular matrix is defined by \f$ \begin{bmatrix} A & B \\ 0 & C \end{bmatrix} \f$ where each block is three-dimensiaonl 
	//! square matrix. Its relocated matrix is defined by \f$ \begin{bmatrix} A & 0 \\ B & C \end{bmatrix} \f$. 
	//! Then, the block-wise multiplication of this relocated (i.e. lower block-triangular) matrix with the wrench is defined by 
	//! \f$ \begin{bmatrix} A f  \\  B f + C n \end{bmatrix} \f$ for \f$ V = \begin{bmatrix} f \\ n \end{bmatrix} \f$.
	//! This function is necessary to compute differential-of-exponential families for \f$ - \xi \f$.
	//! 
	//! \param UBT The six-dimensional lower block-triangular matrix
	//  ----------------------------------------------------------
	inline LieGroup::Wrench upperBlockTriRelocatedMult(Matrix6d const & UBT) const
	{
		LieGroup::Wrench res;

		res << UBT.topLeftCorner<3,3>()*f(),
			UBT.bottomLeftCorner<3,3>()*f() + UBT.bottomRightCorner<3,3>()*n();

		return res;
	}
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! rotates the wrench by the rotation matrix
	//! 
	//! \details
	//! The wrench is updated by \f$ \begin{bmatrix} R f \\ R n \end{bmatrix} \f$ 
	//!	for \f$ F = \begin{bmatrix} f \\ n \end{bmatrix} \f$.
	//! The function is defined in Rotation.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Wrench F; 
	//! LieGroup::Rotation R;
	//! ...
	//! F.rotate(R); // This sets F = [ R*f(); R*n() ]
	//!	\endcode
	//!
	//! \param R The three-dimensional rotation matrix
	//!
	//! \sa LieGroup::Rotation
	//  ----------------------------------------------------------
	inline void rotate(Rotation const & R);

	/**
	* update by inverse-rotation with the rotation matrix, i.e. self = [R^T 0; 0, R^T] *self
	* @param R the rotation matrix
	*/
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! rotates the wrench by the inverse of the rotation matrix
	//! 
	//! \details
	//! The wrench is updated by \f$ \begin{bmatrix} R^T f \\ R^T n \end{bmatrix} \f$ 
	//!	for \f$ F = \begin{bmatrix} f \\ n \end{bmatrix} \f$.
	//! The function is defined in Rotation.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Wrench F; 
	//! LieGroup::Rotation R;
	//! ...
	//! F.irotate(R); // This sets V = [ R.transpose()*f(); R.transpose()*n() ]
	//!	\endcode
	//!
	//! \param R The 3-dimensional rotation matrix
	//!
	//! \sa LieGroup::Rotation
	//  ----------------------------------------------------------
	inline void irotate(Rotation const & R);

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! transforms the wrench by the inverse-transposed adjoint transformation of the homogeneous transformation matrix
	//! 
	//! \details
	//! The wrench is updated by \f$ F = {\rm Adj}_T^{-T} F \f$.
	//! The function is defined in HTransform.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Wrench F; 
	//! LieGroup::HTransform T;
	//! ...
	//! F.transform(T); // This sets F = T.tiadj()*F
	//!	\endcode
	//!
	//! \param T The homogeneous transformation matrix
	//!
	//! \sa LieGroup::HTransform
	//! \sa LieGroup::HTransform::transform(Wrench const & v)
	//  ----------------------------------------------------------
	inline void transform(HTransform const & T);

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! transforms the wrench by the transposed adjoint transformation of the homogeneous transformation matrix
	//! 
	//! \details
	//! The wrench is updated by \f$ F = {\rm Adj}_T^{T} F \f$.
	//! The function is defined in HTransform.h.
	//! 
	//! \code{.cpp}
	//! LieGroup::Wrench F; 
	//! LieGroup::HTransform T;
	//! ...
	//! F.itransform(T); // This sets v = T.tadj()*F
	//!	\endcode
	//!
	//! \param T The homogeneous transformation matrix
	//!
	//! \sa LieGroup::HTransform
	//! \sa LieGroup::HTransform::itransform(Wrench const & v)
	//  ----------------------------------------------------------
	inline void itransform(HTransform const & T);
};

//! \sa LieGroup::Twist::coadjoint()
inline Wrench Twist::coadjoint(Wrench const & F) const
{
	return Wrench(w().cross(F.f()), v().cross(F.f()) + w().cross(F.n()));
}

}

