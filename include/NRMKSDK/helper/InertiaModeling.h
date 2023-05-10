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

#include "NRMKHelper.h"

namespace NRMKHelper
{
class NRMKFoundation::Inertia;

namespace internal
{
	//  ---------------------- Doxygen info ----------------------
	//! \class PolyhedronInertia
	//!
	//! \brief
	//! This implements an algorithm to compute the inertia of a general polyhedron
	//! 
	//! \note
	//! To generate a polyhedron object use file stream.
	//! 
	//! \sa Inertia
	//  ----------------------------------------------------------
	class NRMKHelper_API PolyhedronInertia
	{
	public:
		enum 
		{
			MAX_VERTS = 100, 
			MAX_FACES = 100, 
			MAX_POLYGON_SZ = 10
		};

		struct Polyhedron;

		struct Face 
		{
			int		numVerts;
			double	norm[3];
			double	w;
			int		verts[MAX_POLYGON_SZ];
			Polyhedron *poly;
		};

		struct Polyhedron 
		{
			int		numVerts;
			int		numFaces;
			double	verts[MAX_VERTS][3];
			Face	faces[MAX_FACES];
		};	

	public:
		//  ---------------------- Doxygen info ----------------------
		//! \brief
		//! constructs the inertia for a polyhedron object and a mass and 
		//! saves in the last argument
		//!
		//! \param mass Mass
		//! \param p Polyhedron object
		//! \param inertia NRMKFoundation::Inertia object to store the computed inertia
		//!
		//! \sa internal::PolyhedronInertia::Polyhedron
		//! \sa NRMKFoundation::Inertia
		//  ----------------------------------------------------------
		PolyhedronInertia(double mass, Polyhedron const & p, NRMKFoundation::Inertia & inertia);

		//  ---------------------- Doxygen info ----------------------
		//! \brief
		//! creates a polyhedron object from an input stream
		//!
		//! \param is Input stream
		//! \param p Polyhedron object
		//!
		//! \sa internal::PolyhedronInertia::Polyhedron
		//  ----------------------------------------------------------
		friend std::istream& operator>>(std::istream& is, Polyhedron& p);

	private:
		// compute various integrations over projection of face 
		void _compProjectionIntegrals(Face const & f);
		void _compFaceIntegrals(Face const & f);
		void _compVolumeIntegrals(Polyhedron const & p);

	private:
		int A;   // alpha 
		int B;   // beta 
		int C;   // gamma 

		// projection integrals 
		double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;

		// face integrals 
		double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;

		// volume integrals 
		double T0, T1[3], T2[3], TP[3];
	};

	std::istream& operator>>(std::istream& is, PolyhedronInertia::Polyhedron& p);
}

void NRMKHelper_API SphereInertia(double m, double r, NRMKFoundation::Inertia & inertia);
void NRMKHelper_API HollowSphereInertia(double m, double r, NRMKFoundation::Inertia & inertia);
// at center of mass (located from 3*r/8 from the base)
void NRMKHelper_API HemisphereInertia(double m, double r, int hdir, NRMKFoundation::Inertia & inertia);
void NRMKHelper_API EllipsoidInertia(double m, double x, double y, double z, NRMKFoundation::Inertia & inertia);
void NRMKHelper_API CylinderInertia(double m, double r, double h, int hdir, NRMKFoundation::Inertia & inertia);
void NRMKHelper_API DiskInertia(double m, double r, int hdir, NRMKFoundation::Inertia & inertia);
void NRMKHelper_API RodInertia(double m, double h, int hdir, NRMKFoundation::Inertia & inertia);
void NRMKHelper_API ThickCylinderInertia(double m, double r1, double r2, double h, int hdir, NRMKFoundation::Inertia & inertia);
void NRMKHelper_API RingInertia(double m, double r, int hdir, NRMKFoundation::Inertia & inertia);
void NRMKHelper_API BoxInertia(double m, double x, double y, double z, NRMKFoundation::Inertia & inertia);
void NRMKHelper_API PlateInertia(double m, double x, double y, int hdir, NRMKFoundation::Inertia & inertia);
// at center of mass (located from h/4 from the base)
void NRMKHelper_API ConeInertia(double m, double r, double h, int hdir, NRMKFoundation::Inertia & inertia);
void NRMKHelper_API CapsuleInertia(double m, double r, double h, int hdir, NRMKFoundation::Inertia & inertia);
void NRMKHelper_API PolyhedronInertia(double m, internal::PolyhedronInertia::Polyhedron &p, NRMKFoundation::Inertia & inertia);

} // namespace NRMKHelper