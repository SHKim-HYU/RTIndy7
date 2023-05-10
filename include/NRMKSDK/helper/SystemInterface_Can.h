//  ---------------------- Doxygen info ----------------------
//! \file SystemInterface_Can.h
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
//! \copy details Neuromeka Foundation Library
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
#include "nrmk_hw_cpp.h"

#define CAN_BOUND(val, lbound, ubound)			((val) < (lbound) ? (lbound) : ((val) > (ubound) ? (ubound) : (val)))
#ifdef _USE_MAXON_DES_
#define torque_lbound	(-1000)
#define torque_ubound	(1000)
#define detime			(10000)
#define CAN_SUCCESS		(0)
#define CAN_INIT_ERR	(1)
#define DES_INIT_ERR	(2)

#define DES_SEND_ERR	(10)
#define DES_GET_ERR		(11)

#endif


#define SensorData_rq		0x01
#define ControlData_rq		0x02

#define BcastRxID		0x001	//Broadcasting Rx ID, controller send to simulation (request sensor data, used for simulation purpose only)
#define BcastTxID		0x002	//Broadcasting Tx ID, simulation send all sensor data to controller (used for simulation )

#define VKeyID			0x010	//virtual key Tx ID, simulator send to controller

#define RxIDBase		0x100 	//PDO ID for sending from controller->robot (tau, wrbuf)
#define TxIDBase		0x500 	//PDO ID for sending from robot->controller (q, qdot, rdbuf)


//#include "NRMKSyncObject.h"

namespace NRMKHelper
{
	//  ---------------------- Doxygen info ----------------------
	//! \class SystemInterface_Can
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
	class SystemInterface_Can : public SystemInterfaceBase
	{
	public:
		enum
		{
			NUM_BYTES_READ_BUF = _NUM_BYTES_READ_BUF, 
			NUM_BYTES_WRITE_BUF = _NUM_BYTES_WRITE_BUF, 
			NUM_BYTES_MONITOR_BUF = _NUM_BYTES_MONITOR_BUF,
		};

	public:
		SystemInterface_Can(int cansockfd)
			: _cansockfd(cansockfd)
			,_robot(NULL)
			,_wrdataready(false)
			,_rddataready(false)
			, can_state(-1)
			//, _rdbufReadyEvent("SysInterface_SharedMemory_RdbufReadyEvent")
			//, _wrbufReadyEvent("SysInterface_SharedMemory_WrbufReadyEvent")
			, _rdbuf(_buf), _wrbuf(_rdbuf + NUM_BYTES_READ_BUF), _monbuf(_wrbuf + NUM_BYTES_WRITE_BUF)
		{

		}

		inline void setRatio(float * const torqueratio, float * const gearratio){
			for(int i=0; i<SubsysType::JOINT_DOF; ++i)
			{
				_torqueratio[i]=torqueratio[i];
				_gearratio[i]=gearratio[i];
			}
		}

		inline int InitMotor(des_can_id desid, uint16_t sysConfig)
		{
			if ( !DES_PDOInit(_cansockfd, desid)){
				printf("Can't enable PDO mode\n");
				return PDOInitErr;
			}
			usleep(detime);
			if (!DES_CAN_SetTempParam(_cansockfd, desid, k_SysConfig , sysConfig)){ //system configuration variable
				printf("Can't configure DES device\n");
				return DESConfigErr;
			}
			usleep(detime);
			int timeout=10;
			WORD SysStatus=0;
			while (--timeout>0){
				if (!DES_CAN_Enable(_cansockfd, desid, 1, PDO_Mode)){
					printf("Can't enable DES device\n");
					return DESEnErr;
				}
				usleep(detime);
				DES_CAN_ReadSysStatus(_cansockfd, desid, &SysStatus, PDO_Mode);
				if ((SysStatus&Pwr_Stag_En&Soft_En)==(Pwr_Stag_En&Soft_En))
					break;
			}
			if (timeout==0){
				printf("Can't enable DES device\n");
				return DESEnErr;
			}
			usleep(detime);
			return 0;
		}

		void setCanfd(int cansockfd)
		{
			_cansockfd=cansockfd;
			for(int i=0; i<SubsysType::JOINT_DOF; ++i){
				DES_IDInit(&_desid[i], i+1);
				DES_CAN_Enable(_cansockfd, _desid[i], 0, SDO_Mode); //disable device
				can_state=InitMotor(_desid[i], 0x9004); //initialize and enable motors, current mode
				usleep(detime);

				_torqueratio[i]=0;
				_gearratio[i]=0;
				//DES_CAN_SetCurrent(_cansockfd, desid[i], 1, PDO_Mode);
			}

		}
		void startMotor(int motor_idx)
		{
			if (can_state==0)
				DES_CAN_SetCurrent(_cansockfd, _desid[motor_idx], 1, PDO_Mode);
		}
		void startAllMotor(void)
		{
			if (can_state==0){
				for(int i=0; i<SubsysType::JOINT_DOF; ++i){
					DES_CAN_SetCurrent(_cansockfd, _desid[i], 1, PDO_Mode);
				}
			}

		}
		void stopMotor(int motor_idx)
		{
			if (can_state==0)
				DES_CAN_SetCurrent(_cansockfd, _desid[motor_idx], 0, PDO_Mode);
		}
		void stopAllMotor(void)
		{
			if (can_state==0){
				for(int i=0; i<SubsysType::JOINT_DOF; ++i){
					DES_CAN_SetCurrent(_cansockfd, _desid[i], 0, PDO_Mode);
				}
			}

		}

		void setSystem(SubsysType * robot)
		{
			_robot = robot;

			setReadData();
		}

		virtual int writeTorque(float const * const data)
		{
			if (can_state)
				return -1;

			//directly send
			short ival=0;
			for (int i=0; i < SubsysType::JOINT_DOF; ++i)
			{
				ival = (short) (_torqueratio[i]*data[i]);
				if (!DES_CAN_SetCurrent(_cansockfd, _desid[i], CAN_BOUND(ival, torque_lbound, torque_ubound), PDO_Mode))
					return DES_SEND_ERR;
			}
			return 0;
		}

		virtual int readSensor(float * const q, float * const qdot)
		{
			if (can_state)
				return -1;

			float fval=0;
			short ival=0;
			short param1, param2;
			int lparam;
			for (int i=0; i < SubsysType::JOINT_DOF; ++i)
			{
				if (!DES_CAN_ReadAbsRotorPosition(_cansockfd, _desid[i], (DWORD*) &lparam))
					return DES_GET_ERR;
				fval=(float)lparam;
				q[i]=fval*_gearratio[i];

				if (!DES_CAN_ReadVelocityIsMust(_cansockfd, _desid[i], &param1, &param2, 1, PDO_Mode))
					return DES_GET_ERR;
				fval=(float)param1;
				qdot[i]=fval*_gearratio[i];

			}
			return 0;
		}



		virtual void setReadData()
		{
			float state[2*SubsysType::JOINT_DOF];

			for (int k = 0; k < SubsysType::JOINT_DOF; k++)
			{
				state[k] = (float) _robot->q()[k];
				state[k + SubsysType::JOINT_DOF] = (float) _robot->qdot()[k];
			}

			// FIXME @ 20131115
			//memcpy(_rdbuf, state, 2*SubsysType::JOINT_DOF*sizeof(float));
			memcpy(_rdbuf, state, NUM_BYTES_READ_BUF);

			// simulator send sensor data (_rdbuf)
			_canSendReadData();

			//_rdbufReadyEvent.set();
		}

		virtual void setWriteData()
		{
			//simulator get torque (_wrbuf)
			if (_canReceiveWriteData() <0) return;

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
			return true;
		}

		virtual bool waitForReadDataReady()
		{
			//_rdbufReadyEvent.wait();
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

			_canReceiveReadData();
			memcpy(data, _rdbuf, NUM_BYTES_READ_BUF);

			//_rdbufReadyEvent.reset();
		}

		bool readDatatest(float * const data)
		{
			// 		for (int k = 0; k < SubsysType::JOINT_DOF; k++)
			// 		{
			// 			data[k] = (float) _robot->q()[k];
			// 			data[k + SubsysType::JOINT_DOF] = (float) _robot->qdot()[k];
			// 		}

			if (_canReceiveReadData() !=-1){
				memcpy(data, _rdbuf, NUM_BYTES_READ_BUF);
				return true;
			}
			else
				return false;


			//_rdbufReadyEvent.reset();
		}




		// FIXME @ 20131115
		virtual void writeData(float const * const data)
		{
			memcpy(_wrbuf, data, NUM_BYTES_WRITE_BUF); //for future use only
			_canSendWriteData();

			//_wrbufReadyEvent.set();
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

		//private functions
	private:

		inline int _canSendWriteData() //send tau (controller side)
		{
			if (can_state)
				return -1;
			float fval=0;
			short ival=0;
			for (int i=0; i < SubsysType::JOINT_DOF; ++i)
			{
				memcpy(&fval, _wrbuf+i*4, 4);
				ival = (short) (_torqueratio[i]*fval);
				if (!DES_CAN_SetCurrent(_cansockfd, _desid[i], ival, PDO_Mode))
					return DES_SEND_ERR;
			}
			return 0;
		}

		inline int _canSendReadData() //send q, qdot (simulation side, send all at once)
		{

			return NUM_BYTES_READ_BUF;
		}

		inline int _canReceiveReadData(void) //get sensor data (controller side, _rdbuf) : broadcast
		{

			return NUM_BYTES_READ_BUF;
		}

		inline int _canReceiveWriteData(void) //get torque data (simulation side, _wrbuf)
		{
			return NUM_BYTES_READ_BUF;
		}

	private:
		SubsysType * _robot;
		bool _wrdataready, _rddataready;
		int _cansockfd;
		unsigned char _candata[10];
		int can_state;

#ifdef _USE_MAXON_DES_
		des_can_id _desid[SubsysType::JOINT_DOF]; //the pdo, sdo id set of maxon des motor
		float _torqueratio[SubsysType::JOINT_DOF];
		float _gearratio[SubsysType::JOINT_DOF];
#endif
		// device memory
		unsigned char _buf[NUM_BYTES_READ_BUF + NUM_BYTES_WRITE_BUF + NUM_BYTES_MONITOR_BUF];
		unsigned char * const _rdbuf; 
		unsigned char * const _wrbuf; 
		unsigned char * const _monbuf; 
	};

}
