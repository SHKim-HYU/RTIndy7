//  ---------------------- Doxygen info ----------------------
//! \file CoE.h
//!
//! \brief
//! Header file for constants for COE
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

// Statusword bits
#define STATUSWORD_READY_TO_SWITCH_ON_BIT 		0
#define STATUSWORD_SWITCHED_ON_BIT 				1
#define STATUSWORD_OPERATION_ENABLE_BIT 		2
#define STATUSWORD_FAULT_BIT 					3
#define STATUSWORD_VOLTAGE_ENABLE_BIT 			4
#define STATUSWORD_QUICK_STOP_BIT 				5
#define STATUSWORD_SWITCH_ON_DISABLE_BIT 		6
#define STATUSWORD_NO_USED_WARNING_BIT 			7
#define STATUSWORD_ELMO_NOT_USED_BIT 			8
#define STATUSWORD_REMOTE_BIT 					9
#define STATUSWORD_TARGET_REACHED_BIT 			10
#define STATUSWORD_INTERNAL_LIMIT_ACTIVE_BIT	11

// COE OBJECTS
#define COE_STATUSWORD		0x6041, 0x00	// R, UINT16
#define COE_CONTROLWORD		0x6040, 0x00	// RW, UNIT16
#define COE_OPERATIONMODE	0x6060, 0x00	// RW, UNIT8

// Status
#define STATUS_NOT_SPECIFIED							0xffff
#define STATUS_NOT_READY_TO_SWITCH_ON 		0x0000
#define STATUS_SWITCH_ON_DISABLED 				0x0040
#define STATUS_READY_TO_SWITCH_ON 				0x0021
#define STATUS_SWITCHED_ON							0x0233
#define STATUS_OPERATION_ENABLED 				0x0237
#define STATUS_QUICK_STOP_ACTIVE 					0x0007
#define STATUS_FAULT_REACTION_ACTIVE 			0x000f
#define STATUS_FAULT 										0x0008

// Control CoE FSM(Finite State Machine)
#define CONTROL_COMMAND_DISABLE_VOLTAGE				0x0000
#define CONTROL_COMMAND_QUICK_STOP					0x0002
#define CONTROL_COMMAND_SHUTDOWN					0x0006
#define CONTROL_COMMAND_SWITCH_ON					0x0007
#define CONTROL_COMMAND_ENABLE_OPERATION			0x000f
#define CONTROL_COMMAND_SWITCH_ON_ENABLE_OPERATION	0x000f
#define CONTROL_COMMAND_DISABLE_OPERATION			0x0007
#define CONTROL_COMMAND_FAULT_RESET					0x0080

// Operation mode
#define OP_MODE_NO_MODE					0x00
#define OP_MODE_PROFILE_POSITION		0x01
#define OP_MODE_VELOCITY				0x02
#define OP_MODE_PROFILE_VELOCITY		0x03
#define OP_MODE_TORQUE_PROFILE			0x04
#define OP_MODE_HOMING					0x06
#define OP_MODE_INTERPOLATED_POSITION	0x07
#define OP_MODE_CYCLIC_SYNC_POSITION	0x08
#define OP_MODE_CYCLIC_SYNC_VELOCITY	0x09
#define OP_MODE_CYCLIC_SYNC_TORQUE		0x0a

#define SERVO_TIMEOUT	(50000)

#if 0
#define COE_POSITION	1
#define COE_VELOCITY	2
#define COE_TORQUE		3
#define DATA_IN			4
#define DATA_OUT		5
#endif

#define STATE_NONE 	 	0x00,
#define STATE_INIT	 	0x01,
#define STATE_PRE_OP 	0x02,
#define STATE_SAFE_OP  	0x04,
#define STATE_OP 		0x08

/*
enum __MS_STATE__
{
	NONE 	= 0x00,
	INIT	= 0x01,
	PRE_OP 	= 0x02,
	SAFE_OP = 0x04,
	OP 		= 0x08
} MS_STATE;
*/

typedef ec_sdo_request_t * 		SDO_REG;
typedef ec_slave_config_t *		SLAVE_CONFIG;

typedef uint64_t 		UINT64;
typedef int64_t 		INT64;
typedef unsigned int 	UINT32;
typedef int32_t 		INT32;
typedef int16_t 		INT16;
typedef uint16_t 		UINT16;
typedef uint8_t 		UINT8;
typedef int8_t 			INT8;
