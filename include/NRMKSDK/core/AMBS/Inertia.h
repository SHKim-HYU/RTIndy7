//  ---------------------- Doxygen info ----------------------
//! \file Inertia.h
//!
//! \brief
//! Header file for the class Inertia (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a rigid body inertia class
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
//! \date April 2013
//! 
//! \version 0.1
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

#include <istream>

#include "LieGroup/LieGroup.h"

namespace NRMKFoundation
{
//  ---------------------- Doxygen info ----------------------
//! \class Inertia
//!
//! \brief
//! This implements a rigid body inertia class
//!
//! \details
//! It computes the body inertia and its inverse.
//! The parameters to express the inertia consists of mass, the position of the center-of-mass, and the rotational inertia.
//! Taking into account the rotational inertia being symmetric, they comprise ten parameters.
//! The displacement of the center of mass is measured with respect to the body frame, 
//! i.e. \f$ r = \begin{bmatrix} r_x \\ r_y \\ r_z \end{bmatrix} \f$.
//! The rotational inertia is measured at the com, and the coordinate frame has the same orientation as the body frame.
//! In particular, it is expressed as the three-dimensional symmetric matrix
//! \f$ \begin{bmatrix} I_{xx} & I_{xy} & I_{xz} \\ I_{xy} & I_{yy} & I_{yz} \\ I_{xz} & I_{yz} & I_{zz} \end{bmatrix} \f$.
//  ----------------------------------------------------------
class Inertia
{
public:
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs an inertia with the specified dynamic parameters
	//!
	//! \note
	//! When the first argument m is zero, all other parameters become zero.
	//! When the com is all zero, one can pass NULL.
	//! When the inertia is all zero, pass NULL.
	//!
	//! \param m Mass
	//! \param com Three-dimensional array for position of center of mass 
	//! \param inertia Six-dimensional array for rotational inertia 
	//!		in the order of \f$ Ixx \f$, \f$ Iyy \f$, \f$ Izz \f$, \f$ Ixy \f$, \f$ Iyz \f$, \f$ Izx \f$
	//  ----------------------------------------------------------
	inline Inertia(double m = 0.0, double const * const com = NULL, double const * const inertia = NULL) 
		: _updated_inertia(false), _updated_invInertia(false)
	{
		set(m, com, inertia);
	};

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! constructs an inertia with the specified dynamic parameters
	//!
	//! \note
	//! When the first argument m is zero, all other parameters become zero.
	//!
	//! \param m Mass
	//! \param rx X-coordinate value of the center of mass relative to the body frame, i.e. \f$ r_x \f$
	//! \param ry Y-coordinate value of the center of mass relative to the body frame, i.e. \f$ r_y \f$
	//! \param rz Z-coordinate value of the center of mass relative to the body frame, i.e. \f$ r_z \f$
	//! \param Ixx First diagonal element of the rotational inertia, i.e. \f$ I_{xx} \f$
	//! \param Iyy Second diagonal element of the rotational inertia, i.e. \f$ I_{yy} \f$
	//! \param Izz Third diagonal element of the rotational inertia, i.e. \f$ I_{zz} \f$
	//! \param Ixy Cross inertia between X and Y axis, i.e. \f$ I_{xy} \f$
	//! \param Iyz Cross inertia between Y and Z axis, i.e. \f$ I_{yz} \f$
	//! \param Ixz Cross inertia between X and Z axis, i.e. \f$ I_{xz} \f$
	//  ----------------------------------------------------------
	inline Inertia(double m, double rx, double ry, double rz, double Ixx = 0, double Iyy = 0, double Izz = 0, double Ixy = 0, double Iyz = 0, double Ixz = 0)
		: _updated_inertia(false), _updated_invInertia(false)
	{
		set(m, rx, ry, rz, Ixx, Iyy, Izz, Ixy, Iyz, Ixz);
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the inertia parameters
	//!
	//! \note
	//! When the first argument m is zero, all other parameters become zero.
	//!
	//! \param m Mass
	//! \param rx X-coordinate value of the center of mass relative to the body frame, i.e. \f$ r_x \f$
	//! \param ry Y-coordinate value of the center of mass relative to the body frame, i.e. \f$ r_y \f$
	//! \param rz Z-coordinate value of the center of mass relative to the body frame, i.e. \f$ r_z \f$
	//! \param Ixx First diagonal element of the rotational inertia, i.e. \f$ I_{xx} \f$
	//! \param Iyy Second diagonal element of the rotational inertia, i.e. \f$ I_{yy} \f$
	//! \param Izz Third diagonal element of the rotational inertia, i.e. \f$ I_{zz} \f$
	//! \param Ixy Cross inertia between X and Y axis, i.e. \f$ I_{xy} \f$
	//! \param Iyz Cross inertia between Y and Z axis, i.e. \f$ I_{yz} \f$
	//! \param Ixz Cross inertia between X and Z axis, i.e. \f$ I_{xz} \f$
	//  ----------------------------------------------------------
	inline void set(double m, double rx, double ry, double rz, double Ixx = 0, double Iyy = 0, double Izz = 0, double Ixy = 0, double Iyz = 0, double Ixz = 0)
	{
		_m = m;
		if (_m > 0)
		{	_com[0] = rx; 
			_com[1] = ry; 
			_com[2] = rz;

			_inertia[0] = Ixx;
			_inertia[1] = Iyy;
			_inertia[2] = Izz;
			_inertia[3] = Ixy;
			_inertia[4] = Iyz;
			_inertia[5] = Ixz;
		}
		else
		{
			set();
		}		

		_updated_inertia = false;
		_updated_invInertia = false;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the inertia parameters
	//!
	//! \note
	//! When the first argument m is zero, all other parameters become zero.
	//! When the com is all zero, one can pass NULL.
	//! When the inertia is all zero, pass NULL.
	//!
	//! \param m Mass
	//! \param com Three-dimensional array for position of center of mass 
	//! \param inertia Six-dimensional array for rotational inertia 
	//!		in the order of \f$ Ixx \f$, \f$ Iyy \f$, \f$ Izz \f$, \f$ Ixy \f$, \f$ Iyz \f$, \f$ Izx \f$
	//  ----------------------------------------------------------
	inline void set(double m = 0.0, double const * const com = NULL, double const * const inertia = NULL) 
	{
		_m = m;
		if (m > 0)
		{
			if (com) 
				memcpy(_com, com, 3*sizeof(double));
			else
				memset(_com, 0, 3*sizeof(double));

			if (inertia) 
				memcpy(_inertia, inertia, 6*sizeof(double));		
			else
				memset(_inertia, 0, 6*sizeof(double));	
		}
		else
		{
			memset(_com, 0, 3*sizeof(double));
			memset(_inertia, 0, 6*sizeof(double));	
		}

		_updated_inertia = false;
		_updated_invInertia = false;
	};

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the mass for reading
	//  ----------------------------------------------------------
	inline double m() const { return _m; }
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the com array for reading
	//  ----------------------------------------------------------
	inline double const * const com() const { return _com; }
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the inertia array for reading
	//  ----------------------------------------------------------
	inline double const * const inertia() const { return _inertia; }

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the mass for writing
	//  ----------------------------------------------------------
	inline double& m() { return _m; }
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the com array for writing
	//  ----------------------------------------------------------
	inline double* const com() { return _com; }

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the inertia array for writing
	//  ----------------------------------------------------------
	inline double* const inertia() { return _inertia; }

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the mass 
	//! 
	//! \param m Mass
	//  ----------------------------------------------------------
	inline void mass(double m) 
	{ 
		_m = m; 		

		_updated_inertia = false;
		_updated_invInertia = false;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the com 
	//! 
	//! \param y X-coordinate value of the center of mass relative to the body frame, i.e. \f$ r_x \f$
	//! \param y Y-coordinate value of the center of mass relative to the body frame, i.e. \f$ r_y \f$
	//! \param z Z-coordinate value of the center of mass relative to the body frame, i.e. \f$ r_z \f$
	//  ----------------------------------------------------------
	inline void com(double x, double y, double z) 
	{ 
		_com[0] = x; _com[1] = y; _com[2] = z; 		

		_updated_inertia = false;
		_updated_invInertia = false;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the inertia 
	//! 
	//! \param Ixx First diagonal element of the rotational inertia, i.e. \f$ I_{xx} \f$
	//! \param Iyy Second diagonal element of the rotational inertia, i.e. \f$ I_{yy} \f$
	//! \param Izz Third diagonal element of the rotational inertia, i.e. \f$ I_{zz} \f$
	//! \param Ixy Cross inertia between X and Y axis, i.e. \f$ I_{xy} \f$
	//! \param Iyz Cross inertia between Y and Z axis, i.e. \f$ I_{yz} \f$
	//! \param Ixz Cross inertia between X and Z axis, i.e. \f$ I_{xz} \f$
	//  ----------------------------------------------------------
	inline void inertia(double Ixx, double Iyy, double Izz, double Ixy = 0, double Iyz = 0, double Ixz = 0) 
	{ 
		_inertia[0] = Ixx;
		_inertia[1] = Iyy;
		_inertia[2] = Izz;
		_inertia[3] = Ixy;
		_inertia[4] = Iyz;
		_inertia[5] = Ixz;

		_updated_inertia = false;
		_updated_invInertia = false;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! sets the rotational inertia parameters from the given inertia parameters measured at the rotated frame
	//! 
	//! \details 
	//! The inertia is rotated by 
	//! \f$ R \begin{bmatrix} I_{xx} & I_{xy} & I_{xz} \\ I_{xy} & I_{yy} & I_{yz} \\ I_{xz} & I_{yz} & I_{zz} \end{bmatrix} R^T \f$.
	//!
	//! \param R Rotation matrix of the frame to measure the rotational inertia (relative to the body frame)
	//! \param Ixx First diagonal element of the rotational inertia, i.e. \f$ I_{xx} \f$
	//! \param Iyy Second diagonal element of the rotational inertia, i.e. \f$ I_{yy} \f$
	//! \param Izz Third diagonal element of the rotational inertia, i.e. \f$ I_{zz} \f$
	//! \param Ixy Cross inertia between X and Y axis, i.e. \f$ I_{xy} \f$
	//! \param Iyz Cross inertia between Y and Z axis, i.e. \f$ I_{yz} \f$
	//! \param Ixz Cross inertia between X and Z axis, i.e. \f$ I_{xz} \f$
	//  ----------------------------------------------------------
	inline void inertia(LieGroup::Rotation const & R, double Ixx, double Iyy, double Izz, double Ixy = 0, double Iyz = 0, double Ixz = 0) 
	{ 
		Eigen::Matrix3d Icom;
		Icom << Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz;

		Icom = R*Icom.selfadjointView<Eigen::Upper>()*R.transpose();

		_inertia[0] = Icom(0, 0);
		_inertia[1] = Icom(1, 1);
		_inertia[2] = Icom(2, 2);
		_inertia[3] = Icom(0, 1);
		_inertia[4] = Icom(1, 2);
		_inertia[5] = Icom(0, 2);

		_updated_inertia = false;
		_updated_invInertia = false;
	}

	/// ADDED @20160801
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! resets the body inertia 
	//!
	//! \details
	//! Note that the given inertia has the same body frame.
	//!
	//! \param inertia Inertia object
	//  ----------------------------------------------------------
	inline void set(Inertia const & inertia)
	{
		_m = inertia.m();
		for (int i = 0; i < 3; i++)
			_com[i] = inertia.com()[i];
		
		for (int i = 0; i < 6; i++)
			_inertia[i] = inertia.inertia()[i];

		_updated_inertia = false;
		_updated_invInertia = false;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! composes the body inertia to the current body inertia
	//!
	//! \details
	//! Note that the given inertia has the same body frame.
	//!
	//! \param inertia Inertia object
	//  ----------------------------------------------------------
	inline void addInertia(Inertia const & inertia)
	{
		double m1 = _m;
		double m2 = inertia.m();
		_m += m2;

		double com1[3] = { _com[0], _com[1], _com[2] };

		double ratio = m2/_m;
		for (int i = 0; i < 3; i++)
			_com[i] = (1 - ratio)*com1[i] + ratio*inertia.com()[i];
		
		Eigen::Matrix3d I2;
		I2 << inertia.inertia()[0], inertia.inertia()[3], inertia.inertia()[5], 
			  inertia.inertia()[3], inertia.inertia()[1], inertia.inertia()[4],
			  inertia.inertia()[5], inertia.inertia()[4], inertia.inertia()[2];

		I2 -= m2*LieGroup::internal::_ceil_sqr_expression(inertia.com()[0] - _com[0], inertia.com()[1] - _com[1], inertia.com()[2] - _com[2]);

		Eigen::Matrix3d I1;
		I1 << _inertia[0], _inertia[3], _inertia[5], 
			_inertia[3], _inertia[1], _inertia[4],
			_inertia[5], _inertia[4], _inertia[2];

		/// FIXME @20160817
		I1 -= m1*LieGroup::internal::_ceil_sqr_expression(com1[0] - _com[0], com1[1] - _com[1], com1[2] - _com[2]);

		_inertia[0] = I1(0, 0) + I2(0, 0);
		_inertia[1] = I1(1, 1) + I2(1, 1);
		_inertia[2] = I1(2, 2) + I2(2, 2);
		_inertia[3] = I1(0, 1) + I2(0, 1);
		_inertia[4] = I1(1, 2) + I2(1, 2);
		_inertia[5] = I1(0, 2) + I2(0, 2);

		_updated_inertia = false;
		_updated_invInertia = false;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! composes the body inertia (expressed with respect to the different body frame) to the current body inertia
	//!
	//! \details
	//! This composes the body inertia object to the current body inertia instance. Note that 
	//! the given inertia is measured with respect to the other body frame relative to the body frame.
	//!
	//! \param inertia
	//! Inertia object
	//! \param T
	//! The inertia frame
	//  ----------------------------------------------------------
	inline void addInertia(Inertia const & inertia, LieGroup::HTransform const & T)
	{
		Inertia inertia2(inertia);

		double m1 = _m;
		double m2 = inertia2.m();
		_m += m2;

		LieGroup::Displacement com2(inertia2.com()[0], inertia2.com()[1], inertia2.com()[2]);
		com2 = T.transform(com2);

		LieGroup::Displacement com1(_com[0], _com[1], _com[2]);

		double ratio = m2/_m;
		for (int i = 0; i < 3; i++)
			_com[i] = (1 - ratio)*com1[i] + ratio*com2[i];

		inertia2.inertia(T.R(), inertia2.inertia()[0], inertia2.inertia()[1], inertia2.inertia()[2], 
			inertia2.inertia()[3], inertia2.inertia()[4], inertia2.inertia()[5]);

		Eigen::Matrix3d I2;
		I2 << inertia2.inertia()[0], inertia2.inertia()[3], inertia2.inertia()[5], 
			inertia2.inertia()[3], inertia2.inertia()[1], inertia2.inertia()[4],
			inertia2.inertia()[5], inertia2.inertia()[4], inertia2.inertia()[2];

		I2 -= m2*LieGroup::internal::_ceil_sqr_expression(com2[0] - _com[0],com2[1] - _com[1], com2[2] - _com[2]);
		
		Eigen::Matrix3d I1;
		I1 << _inertia[0], _inertia[3], _inertia[5], 
			  _inertia[3], _inertia[1], _inertia[4],
			  _inertia[5], _inertia[4], _inertia[2];

		/// FIXME @20160817
		I1 -= m1*LieGroup::internal::_ceil_sqr_expression(com1[0] - _com[0], com1[1] - _com[1], com1[2] - _com[2]);
		
		_inertia[0] = I1(0, 0) + I2(0, 0);
		_inertia[1] = I1(1, 1) + I2(1, 1);
		_inertia[2] = I1(2, 2) + I2(2, 2);
		_inertia[3] = I1(0, 1) + I2(0, 1);
		_inertia[4] = I1(1, 2) + I2(1, 2);
		_inertia[5] = I1(0, 2) + I2(0, 2);

		_updated_inertia = false;
		_updated_invInertia = false;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns whether the body mass is zero
	//  ----------------------------------------------------------
	inline bool hasZeroMass() const { return m() == 0; }

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates, if necessary, the body inertia matrix
	//!
	//! \details
	//! The body inertia is defined by 
	//! \f$ \begin{bmatrix} m I & -m \left \lceil r_{com} \right \rceil \\ m \left \lceil r_{com} \right \rceil & I_{com} - m \left \lceil r_{com} \right \rceil^2 \end{bmatrix} \f$,
	//! where \f$ r_{com} \f$ is the com position and \f$ I_{com} \f$ is the rotational inertia at the com. 
	//! 
	//! \return six-dimensional body inertia matrix
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d const & bodyInertia() const
	{
		if (!_updated_inertia)
		{
			_bodyInertia.topLeftCorner<3, 3>() << m(), 0, 0, 0, m(), 0, 0, 0, m();
			_bodyInertia.bottomLeftCorner<3, 3>() = m()*LieGroup::internal::_ceil_expression(_com[0], _com[1], _com[2]);
		
			Eigen::Matrix3d Icom;
			Icom << _inertia[0], _inertia[3], _inertia[5], 
					_inertia[3], _inertia[1], _inertia[4],
					_inertia[5], _inertia[4], _inertia[2];

			_bodyInertia.bottomRightCorner<3, 3>() = Icom - m()*LieGroup::internal::_ceil_sqr_expression(_com[0], _com[1], _com[2]);

			_bodyInertia = _bodyInertia.selfadjointView<Eigen::Lower>();

			_updated_inertia = true;
		}

		return _bodyInertia;
	}

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! generates, if necessary, the body inverse inertia matrix
	//!
	//! \details
	//! The body inverse inertia is defined by 
	//! \f$ \begin{bmatrix} (1/m) I + \left \lceil r_{com} \right \rceil I_{com}^{-1} \left \lceil r_{com} \right \rceil^T & \left \lceil r_{com} \right \rceil I_{com}^{-1} \\ - I_{com}^{-1} \left \lceil r_{com} \right \rceil & I_{com}^{-1} \end{bmatrix} \f$,
	//! where \f$ r_{com} \f$ is the com position and \f$ I_{com} \f$ is the rotational inertia at the com. 
	//! 
	//! \return six-dimensional body inverse inertia matrix
	//!
	//! \sa Matrix6d
	//  ----------------------------------------------------------
	inline Matrix6d const & bodyInvInertia() const
	{
		if (!_updated_invInertia)
		{
			Eigen::Matrix3d Icom;
			Icom << _inertia[0], _inertia[3], _inertia[5], 
				_inertia[3], _inertia[1], _inertia[4],
				_inertia[5], _inertia[4], _inertia[2];

			Eigen::Matrix3d Phi = Icom.inverse();

			Eigen::Map<const Eigen::Vector3d> r(_com);
			Eigen::Matrix3d rPhi;
			rPhi << r.cross(Phi.col(0)), r.cross(Phi.col(1)), r.cross(Phi.col(2));

			Eigen::Matrix3d rPhir;
			rPhir << r.cross(rPhi.row(0)), r.cross(rPhi.row(1)), r.cross(rPhi.row(2));

			rPhir.diagonal().array() += 1/m();
			
			_bodyInvInertia.topRightCorner<3,3>() = rPhir;
			_bodyInvInertia.topLeftCorner<3, 3>() = rPhi;
			_bodyInvInertia.bottomRightCorner<3, 3>() = Phi;

			_bodyInvInertia = _bodyInvInertia.selfadjointView<Eigen::Upper>();

			_updated_invInertia = true;
		}

		return _bodyInvInertia;
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	//  ---------------------- Doxygen info ----------------------
	//! \brief Body mass
	//  ----------------------------------------------------------
	double _m;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Array of the com position
	//  ----------------------------------------------------------
	double _com[3];		// rx, ry, rz, 
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Array of the rotational inertia parameters
	//  ----------------------------------------------------------
	double _inertia[6];	// Ixx, Iyy, Izz, Ixy, Iyz, Ixz

	//  ---------------------- Doxygen info ----------------------
	//! \brief Body inertia matrix
	//  ----------------------------------------------------------
	mutable Matrix6d _bodyInertia;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Body inertia inverse matrix
	//  ----------------------------------------------------------
	mutable Matrix6d _bodyInvInertia;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Flag determining whether the inertia matrix is updated
	//  ----------------------------------------------------------
	mutable bool _updated_inertia;
	
	//  ---------------------- Doxygen info ----------------------
	//! \brief Flag determining whether the inverse inertia matrix is updated
	//  ----------------------------------------------------------
	mutable bool _updated_invInertia;

};	// class Inertia

} // namespace NRMKFoundation