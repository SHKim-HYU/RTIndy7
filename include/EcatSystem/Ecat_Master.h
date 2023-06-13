//! \file Ecat_Master.h
//!
//! \brief Automatically generated header file for the EtherCAT system interface
//!
// This file is part of NRMKPlatform SDK, Windows-based development tool and SDK
// for Real-time Linux Embedded EtherCAT master controller (STEP)
//
// Copyright (C) 2013-2016 Neuromeka <http://www.neuromeka.com>

#ifndef ECATSYSTEM_ECAT_MASTER_H_
#define ECATSYSTEM_ECAT_MASTER_H_

#pragma once 

//EtherCAT Library ******************************************************************
#include "ecrt.h"

#include "CoE.h"

#include "PDOConfig.h"
#include <stdio.h>
#include <memory>
#include <iostream>
#include <cstring>

#define FT_START_DEVICE 0x0B
#define FT_STOP_DEVICE 0x0C
#define FT_SET_OUTPUT_RATE 0x0F
#define FT_GET_OUTPUT_RATE 0x10
#define FT_SET_BIAS 0x11
#define FT_BIAS_SUB 0x01
#define FT_UNBIAS_SUB 0x00

class NRMK_Master
{ 
	public:
		enum
		{
			NUM_NRMK_DRIVE_AXES = 12,
				NUM_NRMK_IO_MODULE_AXES = 2,
				NUM_NRMK_INDY_TOOL_AXES = 2,
		};
		enum
			{
				NONE 	= 0x00,
				INIT	= 0x01,
				PRE_OP 	= 0x02,
				SAFE_OP = 0x04,
				OP 		= 0x08
			} MS_STATE;
		enum
		{
			NUM_MOTION_AXIS = NUM_NRMK_DRIVE_AXES + NUM_NRMK_IO_MODULE_AXES + NUM_NRMK_INDY_TOOL_AXES,
			MAX_TORQUE = 2000,
		};
								
		struct NRMK_DRIVE_IN
			{
				UINT16		Controlword; 	// 0x6040
				INT32		Targetposition; 	// 0x607a
				INT32		Targetvelocity; 	// 0x60ff
				INT16		Targettorque; 	// 0x6071
				INT8		Modesofoperation; 	// 0x6060				
			};
			struct NRMK_DRIVE_OUT
			{
				UINT16		Statusword; 	// 0x6041
				INT32		Positionactualvalue; 	// 0x6064
				INT32		Velocityactualvalue; 	// 0x606c
				INT16		Torqueactualvalue; 	// 0x6077
				INT8		Modesofoperationdisplay; 	// 0x6061				
			};
			struct NRMK_DRIVE
			{
				UINT32 Index;
				UINT32 Alias;
				UINT32 Position;
				SLAVE_CONFIG Config;
				
				NRMK_DRIVE_IN 	InParam;
				NRMK_DRIVE_OUT 	OutParam;
				
				UINT32 offControlword;
				UINT32 offTargetposition;
				UINT32 offTargetvelocity;
				UINT32 offTargettorque;
				UINT32 offModesofoperation;
				UINT32 offStatusword;
				UINT32 offPositionactualvalue;
				UINT32 offVelocityactualvalue;
				UINT32 offTorqueactualvalue;
				UINT32 offModesofoperationdisplay;
				UINT32 bitoffControlword;
				UINT32 bitoffTargetposition;
				UINT32 bitoffTargetvelocity;
				UINT32 bitoffTargettorque;
				UINT32 bitoffModesofoperation;
				UINT32 bitoffStatusword;
				UINT32 bitoffPositionactualvalue;
				UINT32 bitoffVelocityactualvalue;
				UINT32 bitoffTorqueactualvalue;
				UINT32 bitoffModesofoperationdisplay;								
				
			};

			struct NRMK_IO_MODULE_IN
			{
				UINT8		ControlCode; 	// 0x7100
				UINT8		DO5V; 	// 0x7100
				UINT8		TO; 	// 0x7100
				UINT8		DO; 	// 0x7100
				UINT16		AO1; 	// 0x7100
				UINT16		AO2; 	// 0x7100
				UINT32		FTConfigParam; 	// 0x7100
				UINT8		RS485ConfigParam; 	// 0x7100
				UINT8		RS485CMD; 	// 0x7100
				UINT8		RS485TxCnt; 	// 0x7100
				UINT8		RS485TxD0; 	// 0x7100
				UINT8		RS485TxD1; 	// 0x7100
				UINT8		RS485TxD2; 	// 0x7100
				UINT8		RS485TxD3; 	// 0x7100
				UINT8		RS485TxD4; 	// 0x7100
				UINT8		RS485TxD5; 	// 0x7100
				UINT8		RS485TxD6; 	// 0x7100
				UINT8		RS485TxD7; 	// 0x7100
				UINT8		RS485TxD8; 	// 0x7100
				UINT8		RS485TxD9; 	// 0x7100				
			};
			struct NRMK_IO_MODULE_OUT
			{
				UINT8		StatusCode; 	// 0x6100
				UINT8		DI5V; 	// 0x6100
				UINT8		DI1; 	// 0x6100
				UINT8		DI2; 	// 0x6100
				UINT16		AI1; 	// 0x6100
				UINT16		AI2; 	// 0x6100
				INT16		FTRawFx; 	// 0x6100
				INT16		FTRawFy; 	// 0x6100
				INT16		FTRawFz; 	// 0x6100
				INT16		FTRawTx; 	// 0x6100
				INT16		FTRawTy; 	// 0x6100
				INT16		FTRawTz; 	// 0x6100
				UINT8		FTOverloadStatus; 	// 0x6100
				UINT8		FTErrorFlag; 	// 0x6100
				UINT8		RS485RxCnt; 	// 0x6100
				UINT8		RS485RxD0; 	// 0x6100
				UINT8		RS485RxD1; 	// 0x6100
				UINT8		RS485RxD2; 	// 0x6100
				UINT8		RS485RxD3; 	// 0x6100
				UINT8		RS485RxD4; 	// 0x6100
				UINT8		RS485RxD5; 	// 0x6100
				UINT8		RS485RxD6; 	// 0x6100
				UINT8		RS485RxD7; 	// 0x6100
				UINT8		RS485RxD8; 	// 0x6100
				UINT8		RS485RxD9; 	// 0x6100				
			};
			struct NRMK_IO_MODULE
			{
				UINT32 Index;
				UINT32 Alias;
				UINT32 Position;
				SLAVE_CONFIG Config;
				
				NRMK_IO_MODULE_IN 	InParam;
				NRMK_IO_MODULE_OUT 	OutParam;
				
				UINT32 offControlCode;
				UINT32 offDO5V;
				UINT32 offTO;
				UINT32 offDO;
				UINT32 offAO1;
				UINT32 offAO2;
				UINT32 offFTConfigParam;
				UINT32 offRS485ConfigParam;
				UINT32 offRS485CMD;
				UINT32 offRS485TxCnt;
				UINT32 offRS485TxD0;
				UINT32 offRS485TxD1;
				UINT32 offRS485TxD2;
				UINT32 offRS485TxD3;
				UINT32 offRS485TxD4;
				UINT32 offRS485TxD5;
				UINT32 offRS485TxD6;
				UINT32 offRS485TxD7;
				UINT32 offRS485TxD8;
				UINT32 offRS485TxD9;
				UINT32 offStatusCode;
				UINT32 offDI5V;
				UINT32 offDI1;
				UINT32 offDI2;
				UINT32 offAI1;
				UINT32 offAI2;
				UINT32 offFTRawFx;
				UINT32 offFTRawFy;
				UINT32 offFTRawFz;
				UINT32 offFTRawTx;
				UINT32 offFTRawTy;
				UINT32 offFTRawTz;
				UINT32 offFTOverloadStatus;
				UINT32 offFTErrorFlag;
				UINT32 offRS485RxCnt;
				UINT32 offRS485RxD0;
				UINT32 offRS485RxD1;
				UINT32 offRS485RxD2;
				UINT32 offRS485RxD3;
				UINT32 offRS485RxD4;
				UINT32 offRS485RxD5;
				UINT32 offRS485RxD6;
				UINT32 offRS485RxD7;
				UINT32 offRS485RxD8;
				UINT32 offRS485RxD9;
				UINT32 bitoffControlCode;
				UINT32 bitoffDO5V;
				UINT32 bitoffTO;
				UINT32 bitoffDO;
				UINT32 bitoffAO1;
				UINT32 bitoffAO2;
				UINT32 bitoffFTConfigParam;
				UINT32 bitoffRS485ConfigParam;
				UINT32 bitoffRS485CMD;
				UINT32 bitoffRS485TxCnt;
				UINT32 bitoffRS485TxD0;
				UINT32 bitoffRS485TxD1;
				UINT32 bitoffRS485TxD2;
				UINT32 bitoffRS485TxD3;
				UINT32 bitoffRS485TxD4;
				UINT32 bitoffRS485TxD5;
				UINT32 bitoffRS485TxD6;
				UINT32 bitoffRS485TxD7;
				UINT32 bitoffRS485TxD8;
				UINT32 bitoffRS485TxD9;
				UINT32 bitoffStatusCode;
				UINT32 bitoffDI5V;
				UINT32 bitoffDI1;
				UINT32 bitoffDI2;
				UINT32 bitoffAI1;
				UINT32 bitoffAI2;
				UINT32 bitoffFTRawFx;
				UINT32 bitoffFTRawFy;
				UINT32 bitoffFTRawFz;
				UINT32 bitoffFTRawTx;
				UINT32 bitoffFTRawTy;
				UINT32 bitoffFTRawTz;
				UINT32 bitoffFTOverloadStatus;
				UINT32 bitoffFTErrorFlag;
				UINT32 bitoffRS485RxCnt;
				UINT32 bitoffRS485RxD0;
				UINT32 bitoffRS485RxD1;
				UINT32 bitoffRS485RxD2;
				UINT32 bitoffRS485RxD3;
				UINT32 bitoffRS485RxD4;
				UINT32 bitoffRS485RxD5;
				UINT32 bitoffRS485RxD6;
				UINT32 bitoffRS485RxD7;
				UINT32 bitoffRS485RxD8;
				UINT32 bitoffRS485RxD9;								
				
			};

			struct NRMK_INDY_TOOL_IN
			{
				UINT8		ILed; 	// 0x7000
				UINT8		IGripper; 	// 0x7000
				UINT32		FTConfigParam; 	// 0x7000
				UINT8		LEDMode; 	// 0x7000
				UINT8		LEDG; 	// 0x7000
				UINT8		LEDR; 	// 0x7000
				UINT8		LEDB; 	// 0x7000				
			};
			struct NRMK_INDY_TOOL_OUT
			{
				UINT8		IStatus; 	// 0x6000
				UINT8		IButton; 	// 0x6000
				INT16		FTRawFx; 	// 0x6000
				INT16		FTRawFy; 	// 0x6000
				INT16		FTRawFz; 	// 0x6000
				INT16		FTRawTx; 	// 0x6000
				INT16		FTRawTy; 	// 0x6000
				INT16		FTRawTz; 	// 0x6000
				UINT8		FTOverloadStatus; 	// 0x6000
				UINT8		FTErrorFlag; 	// 0x6000				
			};
			struct NRMK_INDY_TOOL
			{
				UINT32 Index;
				UINT32 Alias;
				UINT32 Position;
				SLAVE_CONFIG Config;
				
				NRMK_INDY_TOOL_IN 	InParam;
				NRMK_INDY_TOOL_OUT 	OutParam;
				
				UINT32 offILed;
				UINT32 offIGripper;
				UINT32 offFTConfigParam;
				UINT32 offLEDMode;
				UINT32 offLEDG;
				UINT32 offLEDR;
				UINT32 offLEDB;
				UINT32 offIStatus;
				UINT32 offIButton;
				UINT32 offFTRawFx;
				UINT32 offFTRawFy;
				UINT32 offFTRawFz;
				UINT32 offFTRawTx;
				UINT32 offFTRawTy;
				UINT32 offFTRawTz;
				UINT32 offFTOverloadStatus;
				UINT32 offFTErrorFlag;
				UINT32 bitoffILed;
				UINT32 bitoffIGripper;
				UINT32 bitoffFTConfigParam;
				UINT32 bitoffLEDMode;
				UINT32 bitoffLEDG;
				UINT32 bitoffLEDR;
				UINT32 bitoffLEDB;
				UINT32 bitoffIStatus;
				UINT32 bitoffIButton;
				UINT32 bitoffFTRawFx;
				UINT32 bitoffFTRawFy;
				UINT32 bitoffFTRawFz;
				UINT32 bitoffFTRawTx;
				UINT32 bitoffFTRawTy;
				UINT32 bitoffFTRawTz;
				UINT32 bitoffFTOverloadStatus;
				UINT32 bitoffFTErrorFlag;								
				
			};
			

		typedef std::auto_ptr<struct ecat_variables> EcatSystemVars;
		
	public: 
		NRMK_Master();
		~NRMK_Master();
		
		int init(INT8 ModeOp, UINT32 CycleNano)
		{
			_setMasterCycle(CycleNano);
				
			// TODO: Initiate Axes' parameters here
			
			for (int i=0; i<NUM_NRMK_DRIVE_AXES; i++)
			{
				_systemReady[_NRMK_Drive[i].Index]=0;
				_servoOn[_NRMK_Drive[i].Index] = false;
				// TODO: Init params here. 
				_NRMK_Drive[i].InParam.Modesofoperation = ModeOp;															
			}
			
		for (int i=0; i<NUM_NRMK_IO_MODULE_AXES; i++)
			{
				_systemReady[_NRMK_IO_Module[i].Index]=0;
				_servoOn[_NRMK_IO_Module[i].Index] = false;
				// TODO: Init params here. 														
			}
		

		for (int i=0; i<NUM_NRMK_INDY_TOOL_AXES; i++)
			{
				_systemReady[_NRMK_Indy_Tool[i].Index]=0;
				_servoOn[_NRMK_Indy_Tool[i].Index] = false;
				// TODO: Init params here. 														
			}
			
					
			if (_initMaster() < 0)
			{
				printf("Init Master Failed.\n");
				return -1;
			}					

			if (_initSlaves() == -1)
			{
				printf("Init Slaves Failed.\n");
				deinit();
				return -1;
			}
					
			if (_initDomains() == -1)
			{
				printf("Init Domains Failed.\n");
				deinit();
				return -1;
			}
					
			if (_activateMaster() == -1)
			{
				deinit();
				return -1;
			}
					

			return 0;
		}

		int deinit();
	
		void processTxDomain();
		void processRxDomain();
		
		void readBuffer(int EntryID, void * const data);
		void writeBuffer(int EntryID, void * const data);
		
		void syncEcatMaster();
		
		void readSDO(int EntryID, void * const data);
		void writeSDO(int EntryID, void * const data);
		
		int getRxDomainStatus();
		int getTxDomainStatus();
		int getMasterStatus(unsigned int & NumSlaves, unsigned int & State);
		int getAxisEcatStatus(unsigned int AxisIdx, unsigned int & State);
		
		
		void setServoOn(unsigned int AxisIdx)
		{
			_servoOn[AxisIdx] = true;
		}
		void setServoOff(unsigned int AxisIdx)
		{
			_servoOn[AxisIdx] = false;
		}
		bool isServoOn(unsigned int AxisIdx)
		{
			return _servoOn[AxisIdx];
		}
	
		bool isSystemReady()
		{
			for (int i=0; i<NUM_MOTION_AXIS; ++i)
				if (!_systemReady[i])
					return false;

			return true;
		}

	private:
		void _setMasterCycle(UINT32 DCCycle);
		int	_initMaster();
		int _initSlaves();
		int _initDomains();		
		int _activateMaster();
		UINT16 _statusword_buff[NUM_NRMK_DRIVE_AXES] = {0,};
		UINT16 _controlword_buff[NUM_NRMK_DRIVE_AXES] = {0,};
		
	private: 
		EcatSystemVars _systemVars;
		
		bool _servoOn[NUM_MOTION_AXIS];
		unsigned int _systemReady[NUM_MOTION_AXIS];	
		
		/* EtherCAT Slaves */
		NRMK_DRIVE _NRMK_Drive[NUM_NRMK_DRIVE_AXES];

		NRMK_IO_MODULE _NRMK_IO_Module[NUM_NRMK_IO_MODULE_AXES];

		NRMK_INDY_TOOL _NRMK_Indy_Tool[NUM_NRMK_INDY_TOOL_AXES];
						
}; 

#endif /* ECATSYSTEM_ECAT_MASTER_H_ */
