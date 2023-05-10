//! \file Indy.h
//! \brief Header file for the class Indy (API of IndySDK)
//!
//! \details
//! This file implements an articulated multibody system class
//! to be used for the interface of NRMKFoundation library
//! \n
//! \copydetails Neuromeka Foundation Library
//! \n
//! Neuromeka \n
//! South Korea\n
//! \n
//! http://www.neuromeka.com\n
//!
//! \date January 2017
//! 
//! \version 2.0.0
//!
//!	\author Jonghoon Park, <coolcat@neuromeka.com>
//!
//! \note Copyright (C) 2013--2017 Neuromeka
//  ----------------------------------------------------------

#pragma once

#include "LieGroup/LieGroup.h"
#include "AMBS/Subsys.h"

#define INDY_MAX_NUM_BODIES		11
#define INDY_MAX_NUM_JOINTS		10
#define INDY_MAX_JOINT_DOF		10
#define INDY_NUM_TASK_AXES		6
#define INDY_NUM_TASK_POS		3
#define INDY_NUM_TASK_ROT		3
