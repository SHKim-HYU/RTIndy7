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
//! \version 1.9
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
// Copyright (C) 2013-2015 Neuromeka <coolcat@neuromeka.com>

#pragma once

#include "SystemInterfaceBase.h"

namespace NRMKHelper
{

	//  ---------------------- Doxygen info ----------------------
	//! \class SystemInterface_Direct
	//!
	//! \brief
	//! This implements a concrete SystemInterface class.
	//!
	//! \details
	//! Class SystemInterface_Direct should implement a virtual device for simulated system executing in the same process.
	//!  
	//! \tparam SubsysType Type of the subsys
	//! \tparam ControlAlgorithmType Type of the control algorithm
	//  ----------------------------------------------------------
	template <typename SubsysType>
	class SystemInterface_Direct : public SystemInterfaceBase
	{
	public:
		SystemInterface_Direct(SubsysType * robot) 
			: _robot(robot)
		{
			setReadData();
		}

		virtual void setReadData()
		{
		}

		virtual void setWriteData()
		{
		}

		virtual void setMonitorData()
		{

		}

		virtual bool waitForWriteDataReady()
		{
			return true;
		}

		virtual bool waitForReadDataReady()
		{
			return true;
		}

		virtual void readData(float * const data)
		{
			for (int k = 0; k < SubsysType::JOINT_DOF; k++)
			{
				data[k] = (float) _robot->q()[k];
				data[k + SubsysType::JOINT_DOF] = (float) _robot->qdot()[k];
			}

#ifdef _FORCE_SENSOR_FEEDBACK
			// ADDED @ 20131216
			LieGroup::Wrench const & Fsensor = _robot->body(SubsysType::NUM_BODIES - 1).Fext();
			for (int k = 0; k < 6; k++)
				data[k + 2*SubsysType::JOINT_DOF] = (float) Fsensor[k];
#endif
		}

		virtual void writeData(float const * const data)
		{
			for (int k = 0; k < SubsysType::JOINT_DOF; k++)
				_robot->tau()[k] = data[k];
		}

		virtual void monitorData(float * const data)
		{
		}

		// FIXME @ 20131115
		// 	virtual unsigned char const * const rdbuf() const
		// 	{
		// 		return NULL;
		// 	} 
		// 
		// 	virtual unsigned char * const wrbuf() const 
		// 	{
		// 		return NULL;
		// 	} 
		// 
		// 	virtual unsigned char const * const monbuf() const
		// 	{
		// 		return NULL;
		// 	} 

	private:
		SubsysType * _robot;
	};

	//  ---------------------- Doxygen info ----------------------
	//! \class SystemInterface_Buffer
	//!
	//! \brief
	//! This implements a concrete SystemInterface class.
	//!
	//! \details
	//! Class SystemInterface_Buffer should implement a virtual device for simulated system executing in the same process.
	//! Unlike the previous one, it has a concrete buffers for data communication. 
	//!  
	//! \tparam SubsysType Type of the subsys
	//! \tparam NUM_BYTES_READ_BUF Size in bytes of the read-buffer
	//! \tparam NUM_BYTES_WRITE_BUF Size in bytes of the write-buffer
	//! \tparam NUM_BYTES_MONITOR_BUF Size in bytes of the monitor-buffer
	//  ----------------------------------------------------------
	template <typename SubsysType, int NUM_BYTES_READ_BUF, int NUM_BYTES_WRITE_BUF, int NUM_BYTES_MONITOR_BUF = 0>
	class SystemInterface_Buffer : public SystemInterfaceBase
	{
	public:
		SystemInterface_Buffer(SubsysType * robot) 
			: _robot(robot)
			, _rdbuf(_buf), _wrbuf(_rdbuf + NUM_BYTES_READ_BUF), _monbuf(_wrbuf + NUM_BYTES_WRITE_BUF)
		{
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

			// FIXME @ 20131115
			//memcpy(_rdbuf, state, 2*SubsysType::JOINT_DOF*sizeof(float));
			memcpy(_rdbuf, state, NUM_BYTES_READ_BUF);
		}

		virtual void setWriteData()
		{
			float * tau = (float *) _wrbuf; 

			for (int k = 0; k < SubsysType::JOINT_DOF; k++)
				_robot->tau()[k] = tau[k];
		}

		virtual void setMonitorData()
		{
		}

		virtual bool waitForWriteDataReady()
		{
			return true;
		}

		virtual bool waitForReadDataReady()
		{
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
		}

		// FIXME @ 20131115
		virtual void writeData(float const * const data)
		{
			// 		for (int k = 0; k < SubsysType::JOINT_DOF; k++)
			// 			_robot->tau()[k] = data[k];

			memcpy(_wrbuf, data, NUM_BYTES_WRITE_BUF);
		}

		// FIXME @ 20131115
		virtual void monitorData(float * const data)
		{
		}

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

	private:
		SubsysType * _robot;

		// device memory
		unsigned char _buf[NUM_BYTES_READ_BUF + NUM_BYTES_WRITE_BUF + NUM_BYTES_MONITOR_BUF];
		unsigned char * const _rdbuf; 
		unsigned char * const _wrbuf; 
		unsigned char * const _monbuf; 
	};
}