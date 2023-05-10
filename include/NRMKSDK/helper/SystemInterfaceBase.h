//  ---------------------- Doxygen info ----------------------
//! \file SystemInterface.h
//!
//! \brief
//! Header file for the class SystemInterface (API of the NRMKFoundation Libraries)
//!
//! \details
//! This file implements a system interface class
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
//! \date May 2015
//! 
//! \version 1.9.0
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!	
//!
//! \note Copyright (C) 2013-2015 Neuromeka
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

// This file is part of NRMKFoundation, a lightweight C++ template library
// for robot motion control.
//
// Copyright (C) 2013-2014 Neuromeka <coolcat@neuromeka.com>

#pragma once

namespace NRMKHelper
{

//  ---------------------- Doxygen info ----------------------
//! \class SystemInterfaceBase
//!
//! \brief
//! This implements a interface class for the customized SystemInterface classes.
//!
//! \details
//! Class SystemInterfaceBase should implement a device which should be used to 
//! interface the system, regardless of whether it is virtual simulated one or actual one. 
//! Conceptual description of such a device is as follows. 
//! It consists of three buffers: read-buffer, write-buffer, and monitor-buffer.  
//! The read-buffer is intended to provide sensor values to the controller, whereas
//! the write-buffer is intended to provide actuator values from the controller. 
//! The monitor-buffer is used to send the system's extra information. 
//! It should be noted that these buffers are not necessarily physical buffers. Depending on
//! situation they may be nothing or anything providing some values.
//!
//! One can easily see that a controller should obtain the current sensor values from the read-buffer, 
//! and computes the control input. When the computation is completed, it writes the value to the write-buffer. 
//! From the system's viewpoint it is then clear that the read-buffer should interface to the system's sensor devices
//! and the write-buffer to the actuator devices. If these devices are physical ones this behavior is usually executed by
//! their device drivers. 
//!	
//! \note 
//! The class consists of two groups of member functions. The first is to access the three (virtual) buffers, and the second 
//! is to describe the activity of the device drivers for each buffer. 
//  ----------------------------------------------------------
class SystemInterfaceBase
{
public:
	// device drivers
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! describes the activity which should be done when system exports sensor values (usually to the read-buffer),
	//! e.g. updates the read-buffer with the system's current state
	//  ----------------------------------------------------------
	virtual void setReadData() { }	// describe the activity which should be done when system exports sensor values (usually from the read-buffer). 
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! describes the activity which should be done when system imports actuator values (usually from the write-buffer),
	//! e.g. updates the system's torque with the values in the write-buffer
	//  ----------------------------------------------------------
	virtual void setWriteData() { }  // describe the activity which should be done when system imports actuator values (usually from the write-buffer). 
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! describes the activity which should be done when system exports monitor values (usually to the monitor-buffer),
	//! e.g. updates the monitor-buffer with the system's current monitored values
	//  ----------------------------------------------------------
	virtual void setMonitorData() { }	// controller exports

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! blocked and returns 'true' when the actuator data is available (from controller)
	//  ----------------------------------------------------------
	virtual bool waitForWriteDataReady() { return true; }
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! blocked and returns 'true' when the sensor data is available (to controller)
	//  ----------------------------------------------------------
	virtual bool waitForReadDataReady() { return true; }

	// device drivers
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! read the sensor values (usually from the read-buffer)
	//  ----------------------------------------------------------
	virtual void readData(float * const data)  { }	// describe the activity which should be done when controller reads sensor values.
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! write the actuator values (usually to the write-buffer)
	//  ----------------------------------------------------------
	virtual void writeData(float const * const data) { }  // describe the activity which should be done when controller writes actuator values. 
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! get/set monitor values (usually to/from the monitor-buffer)
	//  ----------------------------------------------------------
	virtual void monitorData(float * const data) { }	

	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! read the sensor values (usually from the read-buffer)
	//  ----------------------------------------------------------
	virtual void readByte(unsigned char * const data)  { }	// describe the activity which should be done when controller reads sensor values.
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! write the actuator values (usually to the write-buffer)
	//  ----------------------------------------------------------
	virtual void writeByte(unsigned char const * const data) { }  // describe the activity which should be done when controller writes actuator values.
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! get/set monitor values (usually to/from the monitor-buffer)
	//  ----------------------------------------------------------
	virtual void monitorByte(unsigned char * const data) { }

	// ADDED@20150514
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! read the sensor values (usually from the read-buffer) of user-defined format
	//!
	//! \note 
	//! User should cast data of the void pointer type to an appropriate type before reading.
	//!
	//! \param mode User-defined mode for category
	//! \param data User-defined buffer
	//  ----------------------------------------------------------
	virtual void readBuffer(int mode, void * const data)  { }	// describe the activity which should be done when controller reads sensor values.
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! write the actuator values (usually to the write-buffer) of user-defined format
	//!
	//! \note 
	//! User should cast data of the void pointer type to an appropriate type before writing.
	//!
	//! \param mode User-defined mode for category
	//! \param data User-defined buffer
	//  ----------------------------------------------------------
	virtual void writeBuffer(int mode, void const * const data) { }  // describe the activity which should be done when controller writes actuator values.
	
	// device buffers
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the read-buffer
	//  ----------------------------------------------------------
	//virtual unsigned char const * const rdbuf() const = 0; 
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the write-buffer
	//  ----------------------------------------------------------
	//virtual unsigned char * const wrbuf() const = 0; 
	//  ---------------------- Doxygen info ----------------------
	//! \brief
	//! returns the monitor-buffer
	//  ----------------------------------------------------------
	//virtual unsigned char const * const monbuf() const = 0; 
};

}
