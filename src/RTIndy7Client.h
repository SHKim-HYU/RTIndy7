#ifndef RTINDY7CLIENT_H_
#define RTINDY7CLIENT_H_

#include <stdio.h>
#include "iostream"
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <queue>
#include <sys/mman.h>

#include <sched.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <rtdm/ipc.h> 
#include "xddp_packet.h"

#include "CS_Indy7.h"

#include <iostream>

#include "Ecat_Master.h"
#include "ServoAxis_Core.h"
#include <PropertyDefinition.h>
#include <liegroup_robotics.h>

#define XDDP_PORT 0	/* [0..CONFIG-XENO_OPT_PIPE_NRDEV - 1] */

#define NSEC_PER_SEC 			1000000000
unsigned int cycle_ns = 1000000; // 1 ms
double period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit


// double ft_offset[6] = {27.90, -56.3, -46.7, -1.269, -0.099, 4.237};
double ft_offset[6] = {0.0,};
Twist F_tmp;

// For RT thread management
static int run = 1;

unsigned long periodCycle = 0, worstCycle = 0;
unsigned long periodLoop = 0, worstLoop = 0;
unsigned int overruns = 0;

double act_max[4] = {0.0,};

// Interface to physical axes
NRMKHelper::ServoAxis_Core Axis[NRMK_DRIVE_NUM];
#ifdef __RP__
const int 	 zeroPos_arm[NRMK_DRIVE_NUM] = {ZERO_POS_1,ZERO_POS_2,ZERO_POS_3,ZERO_POS_4,ZERO_POS_5,ZERO_POS_6,ZERO_POS_7};
const int 	 gearRatio_arm[NRMK_DRIVE_NUM] = {GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_101,GEAR_RATIO_101,GEAR_RATIO_101};
const int 	 TauADC_arm[NRMK_DRIVE_NUM] = {TORQUE_ADC_500,TORQUE_ADC_500,TORQUE_ADC_200,TORQUE_ADC_200,TORQUE_ADC_100,TORQUE_ADC_100,TORQUE_ADC_100};
const double TauK_arm[NRMK_DRIVE_NUM] = {TORQUE_CONST_500,TORQUE_CONST_500, TORQUE_CONST_200,TORQUE_CONST_200,TORQUE_CONST_100,TORQUE_CONST_100,TORQUE_CONST_100};
const int 	 dirQ_arm[NRMK_DRIVE_NUM] = {-1,-1,1,1,-1,-1,-1};
const int 	 dirTau_arm[NRMK_DRIVE_NUM] = {-1,-1,1,1,-1,-1,-1};
const double qdotLimit[NRMK_DRIVE_NUM] = {3*PI, 3*PI, 3*PI, 3*PI, 3*PI, 3*PI, 4*PI};
#else
const int 	 zeroPos_arm[NRMK_DRIVE_NUM] = {ZERO_POS_1,ZERO_POS_2,ZERO_POS_3,ZERO_POS_4,ZERO_POS_5,ZERO_POS_6};
const int 	 gearRatio_arm[NRMK_DRIVE_NUM] = {GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_121,GEAR_RATIO_101,GEAR_RATIO_101,GEAR_RATIO_101};
const int 	 TauADC_arm[NRMK_DRIVE_NUM] = {TORQUE_ADC_500,TORQUE_ADC_500,TORQUE_ADC_200,TORQUE_ADC_100,TORQUE_ADC_100,TORQUE_ADC_100};
const double TauK_arm[NRMK_DRIVE_NUM] = {TORQUE_CONST_500,TORQUE_CONST_500,TORQUE_CONST_200,TORQUE_CONST_100,TORQUE_CONST_100,TORQUE_CONST_100};
const int 	 dirQ_arm[NRMK_DRIVE_NUM] = {-1,-1,1,-1,-1,-1};
const int 	 dirTau_arm[NRMK_DRIVE_NUM] = {-1,-1,1,-1,-1,-1};
const double qdotLimit[NRMK_DRIVE_NUM] = {3*PI, 3*PI, 3*PI, 3*PI, 3*PI, 4*PI};
#endif

// Robotous FT EtherCAT
union DeviceConfig
{
	uint8_t u8Param[4];
	uint32_t u32Param;
};
double force_divider =50.0;
double torque_divider = 2000.0;

// EtherCAT System interface object
Master ecat_master;
EcatNRMK_Drive ecat_drive[NRMK_DRIVE_NUM];
EcatNRMK_Tool ecat_tool[NRMK_TOOL_NUM];

// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;

// Global time (beginning from zero)
double gt=0;
double gt_offset = 0;

// Trajectory parameers
double traj_time=0;
int motion=-1;
int modeControl = 0;
int motioncnt = 0;

Vector3d x_offset;
SO3 R_offset;

// Controller Gains
JVec NRIC_Kp;
JVec NRIC_Ki;
JVec NRIC_K;
JVec NRIC_gamma;

JVec Kp_n;
JVec Kd_n;
JVec K_n;

Twist Task_Kp;
Twist Task_Kv;
Twist Task_Ki;
JVec Task_K;

Matrix6d A_, D_, K_;

JVec des_int;

JVec tau_bd;
double alpha;
double manipulability;


#endif  // /* RTINDY7CLIENT_H_ */