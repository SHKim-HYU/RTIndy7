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
//! \date November 2013
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

// This file is part of NRMKFoundation, a lightweight C++ template library
// for robot motion control.
//
// Copyright (C) 2013-2013 Neuromeka <coolcat@neuromeka.com>

#pragma once

#include <rtapi.h>

#include "SystemInterfaceBase.h"

namespace NRMKHelper
{

//  ---------------------- Doxygen info ----------------------
//! \class SystemInterface_ShMem
//!
//! \brief
//! This implements a concrete SystemInterface class.
//!
//! \details
//! Class SystemInterface_Buffer should implement a virtual device for simulated system executing in the different process.
//! Unlike the previous one, it has a concrete buffers for data communication. 
//!  
//! \tparam SubsysType Type of the subsys
//! \tparam _NUM_BYTES_READ_BUF Size in bytes of the read-buffer
//! \tparam _NUM_BYTES_WRITE_BUF Size in bytes of the write-buffer
//! \tparam _NUM_BYTES_MONITOR_BUF Size in bytes of the monitor-buffer
//  ----------------------------------------------------------
template <typename SubsysType, int _NUM_BYTES_READ_BUF, int _NUM_BYTES_WRITE_BUF, int _NUM_BYTES_MONITOR_BUF = 0>
class SystemInterface_ShMem_RTX : public SystemInterfaceBase
{
public:
	enum
	{
		NUM_BYTES_READ_BUF = _NUM_BYTES_READ_BUF, 
		NUM_BYTES_WRITE_BUF = _NUM_BYTES_WRITE_BUF, 
		NUM_BYTES_MONITOR_BUF = _NUM_BYTES_MONITOR_BUF,
	};

public:
	SystemInterface_ShMem_RTX() 
		: _robot(NULL)
		, _mem(NULL)
		, _rdbuf(NULL), _wrbuf(NULL), _monbuf(NULL), _key(NULL)
	{
		_mem = RtOpenSharedMemory(
			SHM_MAP_WRITE,
			0,
			TEXT("NRMKFoundation::SharedMem_for_RTX"),
			(void**)&_rdbuf);

		if (_mem == NULL)
		{
			_mem = RtCreateSharedMemory(
				PAGE_READWRITE,
				0,
				NUM_BYTES_READ_BUF + NUM_BYTES_WRITE_BUF + NUM_BYTES_MONITOR_BUF + sizeof(char),
				TEXT("NRMKFoundation::SharedMem_for_RTX64"),
				(void**)&_rdbuf);			
		}

		_wrbuf = _rdbuf + NUM_BYTES_READ_BUF;
		_monbuf = _wrbuf + NUM_BYTES_WRITE_BUF;
		_key = _monbuf + NUM_BYTES_MONITOR_BUF;

		_rdbufReadyEvent = RtOpenEvent(
			NULL, 
			FALSE, // ignored
			TEXT("NRMKFoundation::RdBufReadyEvent_for_RTX64")
			);

		if (_rdbufReadyEvent == NULL)
		{
			_rdbufReadyEvent = RtCreateEvent(
				NULL, 
				TRUE, // manual reset
				FALSE, // initial state
				TEXT("NRMKFoundation::RdBufReadyEvent_for_RTX64")
				);			
		}

		_wrbufReadyEvent = RtOpenEvent(
			NULL, 
			FALSE, // ignored
			TEXT("NRMKFoundation::WrBufReadyEvent_for_RTX64")
			);

		if (_wrbufReadyEvent == NULL)
		{
			_wrbufReadyEvent = RtCreateEvent(
				NULL, 
				TRUE, // manual reset
				FALSE, // initial state
				TEXT("NRMKFoundation::WrBufReadyEvent_for_RTX64")
				);			
		}
	}

	~SystemInterface_ShMem_RTX()
	{
		if (_rdbuf)
			RtCloseHandle(_mem);

		if (_rdbufReadyEvent)
			RtCloseHandle(_rdbufReadyEvent);

		if (_wrbufReadyEvent)
			RtCloseHandle(_wrbufReadyEvent);
	}

	void setSystem(SubsysType * robot)
	{
		_robot = robot;

		setReadData();
	}

	int setKey(char key)
	{
		//_mutex.lock();
		memcpy(_key, &key, sizeof(char));
		//_mutex.unlock();

		return 0;
	}

	int getKey(char & key)
	{
		//_mutex.lock();
		memcpy(&key, _key, sizeof(char));
		_key[0] = 0;
		//_mutex.unlock();

		return 0;
	}

	virtual void setReadData()
	{
#if defined(_FORCE_SENSOR_FEEDBACK)
			// ADDED @ 20140106
		float state[2*SubsysType::JOINT_DOF + 6];
#else
		float state[2*SubsysType::JOINT_DOF];
#endif

		for (int k = 0; k < SubsysType::JOINT_DOF; k++)
		{
			state[k] = (float) _robot->q()[k];
			state[k + SubsysType::JOINT_DOF] = (float) _robot->qdot()[k];
		}

#if	defined(_FORCE_SENSOR_FEEDBACK)
		// ADDED @ 20140106
		LieGroup::Wrench const & Fsensor = _robot->body(SubsysType::NUM_BODIES - 1).Fext();
		for (int k = 0; k < 6; k++)
			state[k + 2*SubsysType::JOINT_DOF] = (float) Fsensor[k];
#endif

		memcpy(_rdbuf, state, NUM_BYTES_READ_BUF);
		
		RtSetEvent(_rdbufReadyEvent);
	}

	virtual void setWriteData()
	{
		float * tau = (float *) _wrbuf; 
		for (int k = 0; k < SubsysType::JOINT_DOF; k++)
			_robot->tau()[k] = tau[k];
		
		RtResetEvent(_wrbufReadyEvent);
	}

	virtual void setMonitorData()
	{
	}

	virtual bool waitForWriteDataReady()
	{
		if (RtWaitForSingleObject(_wrbufReadyEvent, INFINITE) == WAIT_OBJECT_0)
			return true;
		else
			return false;
	}

	virtual bool waitForReadDataReady()
	{
		if (RtWaitForSingleObject(_rdbufReadyEvent, INFINITE) == WAIT_OBJECT_0)
			return true;
		else
			return false;
	}

	// FIXME @ 20131115
	virtual void readData(float * const data)
	{
		memcpy(data, _rdbuf, NUM_BYTES_READ_BUF);

		RtResetEvent(_rdbufReadyEvent);
	}

	// FIXME @ 20131115
	virtual void writeData(float const * const data)
	{
		memcpy(_wrbuf, data, NUM_BYTES_WRITE_BUF);

		RtSetEvent(_wrbufReadyEvent);
	}

	// FIXME @ 20131115
	virtual void monitorData(float * const data)
	{
	}

	// FIXME @ 20131115
	unsigned char const * const rdbuf() const
	{
		return _rdbuf;
	} 

	unsigned char * const wrbuf() const 
	{
		return _wrbuf;
	} 

	unsigned char const * const monbuf() const
	{
		return _monbuf;
	} 

private:
	SubsysType * _robot;

	// device memory
	HANDLE _mem;

	HANDLE _rdbufReadyEvent;
	HANDLE _wrbufReadyEvent;

	unsigned char * _rdbuf; 
	unsigned char * _wrbuf; 
	unsigned char * _monbuf; 
	unsigned char * _key;

};

}