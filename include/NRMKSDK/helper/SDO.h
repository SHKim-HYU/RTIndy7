//  ---------------------- Doxygen info ----------------------
//! \file SDO.h
//!
//! \brief
//! Header file for constants for SDO
//! \n
//! \n
//! \n
//! \copy details Neuromeka Platform Library
//! \n
//! \n
//! \n
//! Neuromeka \n
//! South Korea\n
//! \n
//! http://www.neuromeka.com\n
//!
//! \date October 2014
//! 
//! \version 1.8.2
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013-2014 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

// This file is part of NRMKFoundation, a lightweight C++ template library
// for robot motion control.
//
// Copyright (C) 2013-2014 Neuromeka <coolcat@neuromeka.com>

#pragma once

#define	SDO_DEVICE_TYPE			0x1000, 0x00	//	R	UINT32
#define	SDO_DEVICE_ERROR		0x1001, 0x00	//	R	UINT8

#define	SDO_IDENTITY_OBJECT		0x1018, 0x00	//	R	UINT8
#define	SDO_VENDOR_ID			0x1018, 0x01	//	R	UINT32
#define	SDO_PRODUCT_CODE		0x1018, 0x02	//	R	UINT32
#define	SDO_REVISION_NUMBER		0x1018, 0x03	//	R	UINT32
#define	SDO_SERIAL_NUMBER		0x1018, 0x04	//	R	UINT32
