//  ---------------------- Doxygen info ----------------------
//! \file SystemInterface_Socket.h
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

namespace NRMKHelper
{

	//  ---------------------- Doxygen info ----------------------
	//! \class SystemInterface_Socket
	//!
	//! \brief
	//! This implements a concrete SystemInterface class.
	//!
	//! \details
	//! Class SystemInterface_Socket should implement a virtual device for simulated system executing in the separate process.
	//!  
	//! \tparam SubsysType Type of the subsys
	//! \tparam ControlSocketType Type of the control socket
	//! \tparam NUM_BYTES_READ_BUF Size in bytes of the read-buffer
	//! \tparam NUM_BYTES_WRITE_BUF Size in bytes of the write-buffer
	//! \tparam NUM_BYTES_MONITOR_BUF Size in bytes of the monitor-buffer
	//  ----------------------------------------------------------
	//template <typename SubsysType, int _NUM_BYTES_READ_BUF, int _NUM_BYTES_WRITE_BUF, int _NUM_BYTES_MONITOR_BUF = 0>
	template <typename SubsysType, typename ControlSocketType>
	class SystemInterface_Socket : public SystemInterfaceBase
	{
	public:
		enum
		{
			NUM_BYTES_READ_BUF = ControlSocketType::NUM_BYTES_READ_BUF, 
			NUM_BYTES_WRITE_BUF = ControlSocketType::NUM_BYTES_WRITE_BUF, 
			NUM_BYTES_MONITOR_BUF = ControlSocketType::NUM_BYTES_MONITOR_BUF
		};

	public:
		inline SystemInterface_Socket(ControlSocketType & socket) 
			: _socket(socket), _robot(NULL)
			, _rdbuf(_buf), _wrbuf(_rdbuf + NUM_BYTES_READ_BUF), _monbuf(_wrbuf + NUM_BYTES_WRITE_BUF)
		{		
		}

		// This function is to be called by ApplicationManager. 
		// This should be called after control socket has been established.
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

			// socket send
			_socket.sendReadData(_rdbuf);
		}

		virtual void setWriteData()
		{
			// socket receive 
			// This should always succeed because it is called after waitForWriteDataReady()
			_socket.receiveWriteData(_wrbuf);

			float * tau = (float *) _wrbuf; 

			for (int k = 0; k < SubsysType::JOINT_DOF; k++)
				_robot->tau()[k] = tau[k];		
		}

		virtual void setMonitorData()
		{
			memset(_monbuf, 0, NUM_BYTES_MONITOR_BUF);

			// socket send
			_socket.sendReadData(_monbuf);
		}

		virtual bool waitForWriteDataReady()
		{
			_socket.waitForWriteBufReady();
			return true;
		}

		virtual bool waitForReadDataReady()
		{
			_socket.waitForReadBufReady();
			return true;
		}

		// FIXME @ 20131115
		virtual void readData(float * const data)
		{
			_socket.receiveReadData(_rdbuf);
			memcpy(data, _rdbuf, NUM_BYTES_READ_BUF);
		}

		// FIXME @ 20131115
		virtual void writeData(float const * const data)
		{
			memcpy(_wrbuf, data, NUM_BYTES_WRITE_BUF);

			_socket.sendWriteData(_wrbuf);
		}

		// FIXME @ 20131115
		virtual void monitorData(float * const data)
		{
		}
		//////////////////////////////////////////////////////////////////////////

		// FIXME @ 20131115
		// 	virtual unsigned char const * const rdbuf() const
		// 	{
		// 		return _rdbuf;
		// 	} 
		// 
		// 	virtual unsigned char * const wrbuf() const 
		// 	{
		// 		return _wrbuf;
		// 	} 
		// 
		// 	virtual unsigned char const * const monbuf() const
		// 	{
		// 		return _monbuf;
		// 	} 
		// 	
		ControlSocketType & socket() { return _socket; }

	private:		
		// FIXED by THACHDO 20150717
		//NRMKHelper::NRMKControlSocket<NUM_BYTES_READ_BUF, NUM_BYTES_WRITE_BUF, NUM_BYTES_MONITOR_BUF> & _socket;
		ControlSocketType & _socket;
		SubsysType * _robot;

		// device memory
		unsigned char _buf[NUM_BYTES_READ_BUF + NUM_BYTES_WRITE_BUF + NUM_BYTES_MONITOR_BUF];
		unsigned char * const _rdbuf; 
		unsigned char * const _wrbuf; 
		unsigned char * const _monbuf; 
	};

}