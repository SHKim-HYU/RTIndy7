#ifndef MAIN_H  
#define MAIN_H
#include <LR/include/pinocchio_header.h>

//--------------__Define-----------------------


#ifdef __CB__
#define NUM_IO_MODULE 	1
#else
#define NUM_IO_MODULE 	0
#endif
#define NUM_TOOL 		1
#ifdef __RP__
#define NUM_AXIS		7
#else

#define NUM_AXIS		6
#endif
#define NUM_SLAVES (NUM_IO_MODULE+NUM_AXIS+NUM_TOOL)		//Modify this number to indicate the actual number of motor on the network

#ifndef PI
#define PI	(3.14159265359)
#define PI2	(6.28318530718)
#endif

#define WAKEUP_TIME				(5)	// wake up timeout before really run, in second
#define NSEC_PER_SEC 			1000000000
#define CYCLE_NS 1000000         /* 1 ms */
////////// LOGGING BUFFER ///////////////
#define MAX_BUFF_SIZE 		1000

#define FT_START_DEVICE 	0x0000000B
#define FT_STOP_DEVICE 		0x0000000C

#define FT_SET_FILTER_500 	0x00010108
#define FT_SET_FILTER_300 	0x00020108
#define FT_SET_FILTER_200 	0x00030108
#define FT_SET_FILTER_150 	0x00040108
#define FT_SET_FILTER_100 	0x00050108
#define FT_SET_FILTER_50 	0x00060108
#define FT_SET_FILTER_40 	0x00070108
#define FT_SET_FILTER_30 	0x00080108
#define FT_SET_FILTER_20 	0x00090108
#define FT_SET_FILTER_10 	0x000A0108
#define FT_SET_BIAS 		0x00000111
#define FT_UNSET_BIAS 		0x00000011





#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <malloc.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>

#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <rtdm/ipc.h> 
//#include <PCANDevice.h>
//#include <Robotous_FT.h>



#ifdef __QT__
#include <errno.h>
#include <sys/mman.h>

    #undef debug
#endif

#include "Ecat_Master.h"
#include "PropertyDefinition.h"
#include "ServoAxis.h"  // For Indy7 interface




typedef unsigned int UINT32;
typedef int32_t INT32;
typedef int16_t INT16;
typedef uint16_t UINT16;
typedef uint8_t UINT8;
typedef int8_t INT8;


//RT Thread 
#include "print/print_run.h"
#include "ecat/ecat_run.h"
#include "safety/safety_run.h"
#include "ft/ft_run.h"


typedef struct STATE{
	JVec q;
	JVec q_dot;
	JVec q_ddot;
	JVec tau;
	JVec tau_ext;
	JVec G;

	Vector6d x;                           //Task space
	Vector6d x_dot;
	Vector6d x_ddot;
	Vector6d F;
	Vector6d F_CB;

    double s_time;
}state;

typedef struct JOINT_INFO{
	int Position;
	int aq_inc[NUM_AXIS];
	int atoq_per[NUM_AXIS];
	short dtor_per[NUM_AXIS];
	int statusword[NUM_AXIS];

	JVec q_target;
	JVec qdot_target;
	JVec qddot_target;
	JVec traj_time;
	Vector6d F_sensor ;

	STATE act;
	STATE des;
	STATE nom;

}JointInfo;




////////////// TxPDO //////////////
// Drive
extern UINT16	StatusWord[NUM_SLAVES];
extern INT32 	ActualPos[NUM_SLAVES];
extern INT32 	ActualVel[NUM_SLAVES];
extern INT16 	ActualTor[NUM_SLAVES];
extern UINT32	DataIn[NUM_SLAVES];
extern UINT8	ModeOfOperationDisplay[NUM_SLAVES];

// Tool
extern UINT8	IStatus[NUM_SLAVES];
extern UINT8	IButton[NUM_SLAVES];
extern INT16	FTRawFx[NUM_SLAVES];
extern INT16	FTRawFy[NUM_SLAVES];
extern INT16	FTRawFz[NUM_SLAVES];
extern INT16	FTRawTx[NUM_SLAVES];
extern INT16	FTRawTy[NUM_SLAVES];
extern INT16	FTRawTz[NUM_SLAVES];
extern UINT8	FTOverloadStatus[NUM_SLAVES];
extern UINT8	FTErrorFlag[NUM_SLAVES];

// Tool
extern UINT8	ILed[NUM_SLAVES];
extern UINT8	IGripper[NUM_SLAVES];
extern UINT32	FTConfigParam[NUM_SLAVES];
extern UINT8	LEDMode[NUM_SLAVES];
extern UINT8	LEDG[NUM_SLAVES];
extern UINT8	LEDR[NUM_SLAVES];
extern UINT8	LEDB[NUM_SLAVES];

// IO
#ifdef __CB__

extern UINT8	StatusCode[NUM_SLAVES];
extern UINT8	DI5V[NUM_SLAVES];
extern UINT8	DI1[NUM_SLAVES];
extern UINT8	DI2[NUM_SLAVES];
extern UINT16	AI1[NUM_SLAVES];
extern UINT16	AI2[NUM_SLAVES];
extern INT16	FTRawFxCB[NUM_SLAVES];
extern INT16	FTRawFyCB[NUM_SLAVES];
extern INT16	FTRawFzCB[NUM_SLAVES];
extern INT16	FTRawTxCB[NUM_SLAVES];
extern INT16	FTRawTyCB[NUM_SLAVES];
extern INT16	FTRawTzCB[NUM_SLAVES];
extern UINT8	FTOverloadStatusCB[NUM_SLAVES];
extern UINT8	FTErrorFlagCB[NUM_SLAVES];
extern UINT8	RS485RxCnt[NUM_SLAVES];
extern UINT8	RS485RxD0[NUM_SLAVES];
extern UINT8	RS485RxD1[NUM_SLAVES];
extern UINT8	RS485RxD2[NUM_SLAVES];
extern UINT8	RS485RxD3[NUM_SLAVES];
extern UINT8	RS485RxD4[NUM_SLAVES];
extern UINT8	RS485RxD5[NUM_SLAVES];
extern UINT8	RS485RxD6[NUM_SLAVES];
extern UINT8	RS485RxD7[NUM_SLAVES];
extern UINT8	RS485RxD8[NUM_SLAVES];
extern UINT8	RS485RxD9[NUM_SLAVES];

extern UINT8	ControlCode[NUM_SLAVES];
extern UINT8	DO5V[NUM_SLAVES];
extern UINT8	TO[NUM_SLAVES];
extern UINT8	DO[NUM_SLAVES];
extern UINT16	AO1[NUM_SLAVES];
extern UINT16	AO2[NUM_SLAVES];
extern UINT32	FTConfigParamCB[NUM_SLAVES];
extern UINT8	RS485ConfigParam[NUM_SLAVES];
extern UINT8	RS485CMD[NUM_SLAVES];
extern UINT8	RS485TxCnt[NUM_SLAVES];
extern UINT8	RS485TxD0[NUM_SLAVES];
extern UINT8	RS485TxD1[NUM_SLAVES];
extern UINT8	RS485TxD2[NUM_SLAVES];
extern UINT8	RS485TxD3[NUM_SLAVES];
extern UINT8	RS485TxD4[NUM_SLAVES];
extern UINT8	RS485TxD5[NUM_SLAVES];
extern UINT8	RS485TxD6[NUM_SLAVES];
extern UINT8	RS485TxD7[NUM_SLAVES];
extern UINT8	RS485TxD8[NUM_SLAVES];
extern UINT8	RS485TxD9[NUM_SLAVES];
#endif


#ifdef __CASADI__
// #include <dlfcn.h>

// typedef long long int casadi_int;
// typedef int (*eval_t)(const double**, double**, casadi_int*, double*, int);
#include "control/control_run.h"
#include "CS_Indy7.h"
extern  unsigned long periodIndysim ;

extern CS_Indy7 cs_indy7;
extern CS_Indy7 cs_sim_indy7;

#endif
#ifdef __QT__
#include "qt/qt_run.h"
#endif

#ifdef __LR__
    #include "lr_control/lr_control_run.h"
	#include <LR/include/LR_Control.h>
	#include <LR/include/LR_Trajectory.h>
    extern LR_Control lr_control;

#endif
////////////// RxPDO //////////////
// Drive
extern INT32 	TargetPos[NUM_SLAVES];
extern INT32 	TargetVel[NUM_SLAVES];
extern INT16 	TargetTor[NUM_SLAVES];
extern UINT8 	ModeOfOperation[NUM_SLAVES];
extern UINT16	Controlword[NUM_SLAVES];


extern NRMK_Master nrmk_master;
extern NRMKHelper::ServoAxis Axis[NUM_AXIS];
extern bool system_ready;

extern double period;
extern double gt;

extern JOINT_INFO info;
extern bool run;

// For RT thread management

extern unsigned long periodCycle, worstCycle;
extern unsigned long periodCompute, worstCompute;
extern unsigned long periodEcat, worstEcat;
extern unsigned long periodBuffer, worstBuffer;
extern unsigned int overruns;

extern double a;
#endif  // MAIN_H

