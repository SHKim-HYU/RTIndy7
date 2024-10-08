#ifndef GLOBALVARS_H_
#define GLOBALVARS_H_

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
#include <PropertyDefinition.h>

extern double gt;
extern int system_ready ;
extern ROBOT_INFO info;
extern unsigned int cycle_ns;
extern unsigned long periodCycle;
extern unsigned long worstCycle;
extern unsigned long periodLoop;
extern unsigned long worstLoop;
extern unsigned int overruns;
extern NRMKHelper::ServoAxis_Core Axis[NRMK_DRIVE_NUM];
extern Master ecat_master;
extern EcatNRMK_Drive ecat_drive[NRMK_DRIVE_NUM];
extern EcatNRMK_Tool ecat_tool[NRMK_TOOL_NUM];
extern CS_Indy7 cs_indy7;
extern CS_Indy7 cs_nom_indy7;
extern Twist F_tmp;
extern double period;
extern double manipulability;
#endif
