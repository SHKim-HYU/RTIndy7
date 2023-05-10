//  ---------------------- Doxygen info ----------------------
//! \file SystemInterface_ShMem.h
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

#include "SystemInterfaceBase.h"
#include "Poco/SharedMemory.h"
//#include "Poco/NamedMutex.h"
#include "Poco/NamedEvent.h"

//#include "NRMKSyncObject.h"

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
	class SystemInterface_ShMem : public SystemInterfaceBase
	{
	public:
		enum
		{
			NUM_BYTES_READ_BUF = _NUM_BYTES_READ_BUF, 
			NUM_BYTES_WRITE_BUF = _NUM_BYTES_WRITE_BUF, 
			NUM_BYTES_MONITOR_BUF = _NUM_BYTES_MONITOR_BUF,
		};

	public:
		SystemInterface_ShMem() 
			: _robot(NULL)
			, _mem("SysInterface_SharedMemory", NUM_BYTES_READ_BUF + NUM_BYTES_WRITE_BUF + NUM_BYTES_MONITOR_BUF, static_cast<Poco::SharedMemory::AccessMode>(Poco::SharedMemory::AM_READ | Poco::SharedMemory::AM_WRITE))
			, _rdbufReadyEvent("SysInterface_SharedMemory_RdbufReadyEvent")
			, _wrbufReadyEvent("SysInterface_SharedMemory_WrbufReadyEvent")
			, _rdbuf(reinterpret_cast<unsigned char*>(_mem.begin())), _wrbuf(_rdbuf + NUM_BYTES_READ_BUF), _monbuf(_wrbuf + NUM_BYTES_WRITE_BUF)
		{

		}

		void setSystem(SubsysType * robot)
		{
			_robot = robot;

			setReadData();
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

			_rdbufReadyEvent.set();
		}

		virtual void setWriteData()
		{
			float * tau = (float *) _wrbuf; 
			for (int k = 0; k < SubsysType::JOINT_DOF; k++)
				_robot->tau()[k] = tau[k];
			
			//_wrbufReadyEvent.reset();
		}

		virtual void setMonitorData()
		{
		}

		virtual bool waitForWriteDataReady()
		{
			_wrbufReadyEvent.wait();
			return true;
		}

		virtual bool waitForReadDataReady()
		{
			_rdbufReadyEvent.wait();
			return true;
		}

		// FIXME @ 20131115
		virtual void readData(float * const data)
		{
			// 		for (int k = 0; k < SubsysType::JOINT_DOF; k++)
			// 		{
			// 			data[k] = (float) _robot->q()[k];
			// 			data[k + SubsysType::JOINT_DOF] = (float) _robot->qdot()[k];
			// 		}
			memcpy(data, _rdbuf, NUM_BYTES_READ_BUF);

			//_rdbufReadyEvent.reset();
		}

		// FIXME @ 20131115
		virtual void writeData(float const * const data)
		{
			// 		for (int k = 0; k < SubsysType::JOINT_DOF; k++)
			// 			_robot->tau()[k] = data[k];
			memcpy(_wrbuf, data, NUM_BYTES_WRITE_BUF);

			_wrbufReadyEvent.set();
		}

		// FIXME @ 20131115
		virtual void monitorData(float * const data)
		{
			// 		_mutex.lock();
			// 		memcpy(data, _monbuf, NUM_BYTES_MONITOR_BUF);
			// 		_mutex.unlock();
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
		Poco::SharedMemory _mem;

		Poco::NamedEvent _rdbufReadyEvent;
		Poco::NamedEvent _wrbufReadyEvent;

		unsigned char * const _rdbuf; 
		unsigned char * const _wrbuf; 
		unsigned char * const _monbuf; 
	};

}