
// Automatically generated realtime application source file for STEP platforms
//
// This file is part of NRMKPlatform SDK, Windows-based development tool and SDK
// for Real-time Linux Embedded EtherCAT master controller (STEP).
//
// Copyright (C) 2013-2015 Neuromeka <http://www.neuromeka.com>

//-system-/////////////////////////////////////////////////////////////////
#ifndef __XENO__
#define __XENO__
#endif

#include "RTRArmClient.h"

#define USE_DC_MODE


#define NUM_FT	 	1

hyuEcat::Master ecatmaster;

hyuEcat::EcatNRMK_Indy_Tool ecat_nrmk_indy_tool[NUM_FT];
hyuEcat::EcatNRMK_Drive ecat_nrmk_drive[NUM_AXIS];
hyuCtrl::Trajectory *traj5th_joint; //make an instance each of a joint
hyuCtrl::Trajectory *traj5th_task;
JointInfo info;
robot *cManipulator;
HYUControl::Controller *Control;
ROBOT *_robotNom;




////////// LOGGING BUFFER ///////////////
#define MAX_BUFF_SIZE 		1000

static int sampling_time 	= 5;	// Data is sampled every 5 cycles.
volatile int sampling_tick 	= 0;

struct LOGGING_PACK
{
	double Time;
	INT32 	ActualPos[NUM_AXIS];
	INT32 	ActualVel[NUM_AXIS];
};

unsigned int frontIdx = 0, rearIdx = 0;
LOGGING_PACK _loggingBuff[MAX_BUFF_SIZE];
/////////////////////////////////////////

// NRMKDataSocket for plotting axes data in Data Scope
EcatDataSocket datasocket;

// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;
int isElmoReady = 0;
int isFirstRun = 0;

// Global time (beginning from zero)
double gt=0;
double double_gt=0;

int InitFlag[NUM_AXIS] = {0,};
int TrajFlag_j[NUM_AXIS] = {1,1,1,1,1,1};
int TrajFlag_t[NUM_AXIS] = {0,};


// EtherCAT Data (in pulse)
INT32 	ZeroPos[NUM_AXIS] = {0,};
UINT16	StatusWord[NUM_AXIS] = {0,};
INT32 	ActualPos[NUM_AXIS] = {0,};
INT32	ActualPos_Old[NUM_AXIS] = {0,};
INT32 	ActualVel[NUM_AXIS] = {0,};
INT32	ActualAcc[NUM_AXIS] = {0,};
INT32	ActualVel_Old[NUM_AXIS] = {0,};
INT32	ActualAcc_Old[NUM_AXIS] = {0,};
INT16 	ActualTor[NUM_AXIS] = {0,};
UINT32	DataIn[NUM_AXIS] = {0,};
INT8	ModeOfOperationDisplay[NUM_AXIS] = {0,};
INT8	DeviceState[NUM_AXIS] = {0,};
UINT32	DigitalInput[NUM_AXIS] = {0,};
INT32   HomePos[NUM_AXIS]={0, 0, 0, -6553600, 0, 0};

INT32 	TargetPos[NUM_AXIS] = {0,};
INT32 	TargetVel[NUM_AXIS] = {0,};
INT32 	TargetAcc[NUM_AXIS] = {0,};
INT16 	TargetTor[NUM_AXIS] = {0,};

UINT32 	DataOut[NUM_AXIS] = {0,};
INT8 	ModeOfOperation[NUM_AXIS] = {0,};
UINT16	ControlWord[NUM_AXIS] = {0,};
INT32	VelocityOffset[NUM_AXIS] = {0,};
INT16	TorqueOffset[NUM_AXIS] = {0,};
UINT32	DigitalOutput[NUM_AXIS] = {0,};


INT8   iLed              = 0;       // write
INT8   iGripper          = 0; 		// write
INT32  FT_configparam    = 0; 		// write
INT8   LED_mode          = 0; 		// write (max torque (max current) = 1000)
INT8   LED_G             = 0; 		// write (use enum ModeOfOperation for convenience)
INT8   LED_R             = 0;       // write (use enum ModeOfOperation for convenience)
INT8   LED_B             = 0;       // write (use enum ModeOfOperation for convenience)

INT8   iStatus           = 0;       // read
INT32  iButton           = 0; 		// read
INT16  FT_Raw_Fx         = 0;       // read
INT16  FT_Raw_Fy         = 0;       // read
INT16  FT_Raw_Fz         = 0;       // read
INT16  FT_Raw_Tx         = 0;       // read
INT16  FT_Raw_Ty         = 0;       // read
INT16  FT_Raw_Tz         = 0; 		// read
INT8   FT_OverloadStatus = 0; 		// read
INT8   FT_ErrorFlag      = 0;       // read


double DF=50.0;
double DT=2000.0;
double Tx[NUM_FT]={0.0};
double Ty[NUM_FT]={0.0};
double Tz[NUM_FT]={0.0};
double Fx[NUM_FT]={0.0};
double Fy[NUM_FT]={0.0};
double Fz[NUM_FT]={0.0};
double ZMP_X[2]={0,};
double ZMP_Y[2]={0,};


//////////////////////////
/****************************************************************************/

// Xenomai RT tasks
RT_TASK RTRArm_task;
RT_TASK print_task;
RT_TASK plot_task;

// For RT thread management
static int run = 1;
unsigned long fault_count=0;
long ethercat_time=0, worst_time=0;
#define min_time	0
#define max_time	100000
#define hist_step	(100)
unsigned int histdata[hist_step+1];
unsigned int interval_size=350;

double ActualPos_zero[NUM_AXIS] = {ZERO_POS_1, ZERO_POS_2,ZERO_POS_3, ZERO_POS_4, ZERO_POS_5, ZERO_POS_6};
double ActualPos_Rad[NUM_AXIS] = {0.0,};
double ActualVel_Rad[NUM_AXIS] = {0.0,};
double ActualAcc_Rad[NUM_AXIS] = {0.0,};
double TargetPos_Rad[NUM_AXIS] = {0.0,};
double TargetVel_Rad[NUM_AXIS] = {0.0,};
double TargetAcc_Rad[NUM_AXIS] = {0.0,};
double TargetToq[NUM_AXIS] = {0.0,};
INT32   MotorDir[NUM_AXIS] = {1,1,1,1,1,1};
int   old_flag[NUM_AXIS]={0,};
int   acc_flag[NUM_AXIS]={0,};
Jointd gmat;
Matrixd mmat;
Matrixd cmat;
Matrix6n6nd adv;

Jaco b_jaco;
InvJaco b_inv;
LinJaco l_jaco, l_jaco_dot;
PinvLJaco DPI_l_jaco;
se3 wrench_test;
Jointd torq;
Vector3d EulerAng;
double time_buf;
int time_flag=0;
int limit_flag=0;


//For Trajectory management
//Joint
int NUM_MOTION=1;
int Motion=1;
double TargetTrajPos_Rad[NUM_AXIS]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};/////////////////////////must change/////////////////////////////////
//double TargetTrajPos_Rad[NUM_AXIS]={0.0, -1.57, 0.0, 1.57, 0.0};
double traj_time = 3;
int traj_changed = 0;
//Task
double TargetTrajPos_Meter[NUM_AXIS]={0,0,};
Vector3d xd, xd_dot, xd_ddot;
Vector3d x_buf;
Vector3d vive_pos;
Vector3d vive_vel;
Vector3d vive_buf;
int vive_flag=0;
double K_tracking_ = 1;
int task_time=0;

Matrix<double,NUM_AXIS,1> qd,qd_dot,qd_ddot ,qd_old, qd_dot_old;
double q_[3]={0.0,};
double traq[NUM_AXIS]={0.0, 0.0, 0.0, -1.5709, 0.0, 0.0};
double traq_d[NUM_AXIS]={0.0,};
double traq_dd[NUM_AXIS]={0.0,};
int cnt=0;
int flag=0;
int q_flag=0;

//For sEMG
int emg_state = 5;

//For Robo-Limb
int fd=-1;
int rtsercan_fd  = -1;
int ftsercan_fd  = -1;
int open_state;
int hand_flag = 0;
int finger_state[6]={0,};

// Signal handler for CTRL+C
void signal_handler(int signum);


void saveLogData()
{
/*	if (datasocket.hasConnection() && sampling_tick-- == 0)
	{
		sampling_tick = sampling_time - 1; // 'minus one' is necessary for intended operation

		if (rearIdx < MAX_BUFF_SIZE)
		{
			_loggingBuff[rearIdx].Time = gt;
			for (int i=0; i<NUM_AXIS; i++)
			{
				_loggingBuff[rearIdx].ActualPos[i] = ActualPos[i];
				_loggingBuff[rearIdx].ActualVel[i] = ActualVel[i];
			}
			rearIdx++;
		}
	}
*/
}

/****************************************************************************/
void EncToRad()
{
	
	for(int i=0; i<NUM_AXIS; i++)
	{
		if(old_flag[i]==0){
			ActualPos_Old[i]=ActualPos[i];
			old_flag[i]=1;
		}
		else if(old_flag[i]==1){

			ActualVel[i]=(2*0.01-0.001)/(2*0.01+0.001)*ActualVel_Old[i]+2/(2*0.01+0.001)*(ActualPos[i]-ActualPos_Old[i]);

			switch(i)
			{
			case 0:
			case 1:
				ActualPos_Rad[i]=(double)ActualPos[i]/(ENC_CORE_500/PI2*GEAR_RATIO_121); //262144/2PI
				ActualVel_Rad[i]=(double)ActualVel[i]/(ENC_CORE_500/PI2*GEAR_RATIO_121);
				break;
			case 2:
				ActualPos_Rad[i]=(double)ActualPos[i]/(ENC_CORE_200/PI2*GEAR_RATIO_121); //262144/2PI
				ActualVel_Rad[i]=(double)ActualVel[i]/(ENC_CORE_200/PI2*GEAR_RATIO_121);
				break;
			case 3:
			case 4:
			case 5:
				ActualPos_Rad[i]=(double)ActualPos[i]/(ENC_CORE_100/PI2*GEAR_RATIO_101); //262144/2PI
				ActualVel_Rad[i]=(double)ActualVel[i]/(ENC_CORE_100/PI2*GEAR_RATIO_101);
				break;
			}
			

			//for Kinematics & Dynamics
			info.act.q[i]=ActualPos_Rad[i];
			info.act.q_dot[i]=ActualVel_Rad[i];

			//buffer for calculate velocity
			ActualPos_Old[i]=ActualPos[i];
			ActualVel_Old[i]=ActualVel[i];
		}
	}
	info.act.j_q=Map<VectorXd>(info.act.q,NUM_AXIS);
	info.act.j_q_d=Map<VectorXd>(info.act.q_dot,NUM_AXIS);

	// for(int i=0; i<NUM_AXIS; i++)
	// {
	// 	switch(i)
	// 	{
	// 	case 0:
	// 	case 1:
	// 		ActualPos_Rad[i]=(double)ActualPos[i]/(ENC_CORE_500/PI2*GEAR_RATIO_121); //262144/2PI
	// 		ActualVel_Rad[i]=(double)ActualVel[i]/(ENC_CORE_500/PI2*GEAR_RATIO_121);
	// 		break;
	// 	case 2:
	// 		ActualPos_Rad[i]=(double)ActualPos[i]/(ENC_CORE_200/PI2*GEAR_RATIO_121); //262144/2PI
	// 		ActualVel_Rad[i]=(double)ActualVel[i]/(ENC_CORE_200/PI2*GEAR_RATIO_121);
	// 		break;
	// 	case 3:
	// 	case 4:
	// 	case 5:
	// 		ActualPos_Rad[i]=(double)ActualPos[i]/(ENC_CORE_100/PI2*GEAR_RATIO_101); //262144/2PI
	// 		ActualVel_Rad[i]=(double)ActualVel[i]/(ENC_CORE_100/PI2*GEAR_RATIO_101);
	// 		break;
	// 	}
		
	// 	//for Kinematics & Dynamics
	// 	info.act.q[i]=ActualPos_Rad[i];
	// 	info.act.q_dot[i]=ActualVel_Rad[i];

	// 	//buffer for calculate velocity
	// 	ActualPos_Old[i]=ActualPos[i];
	// 	ActualVel_Old[i]=ActualVel[i];

	// }
	// info.act.j_q=Map<VectorXd>(info.act.q,NUM_AXIS);
	// info.act.j_q_d=Map<VectorXd>(info.act.q_dot,NUM_AXIS);
}

void Robot_Limit()
{
	//Joint limit
	/*if(traq[0] <= -0.86 || traq[0] >= 2.61)
	{
		if(traq[0] <= -0.86){
			traq[0] =-0.86;
			//flag=1;
		}
		else{
			traq[0]=2.61;
			//flag=1;
		}
	}else{}
	if(traq[1] <= -1.89 || traq[1] >=0.3)
	{
		if(traq[1] <= -1.89){
			traq[1] =-1.89;
			//flag=1;
		}
		else{
			traq[1]=0.3;
			//flag=1;
		}
	}
	if(traq[2] <=-1.28  || traq[2] >=1.0)
	{
		if(traq[2] <= -1.28){
			traq[2] = -1.28;
			//flag=1;
		}
		else{
			traq[2]=1.0;
			//flag=1;
		}
	}
	if(traq[3] <= 0.3 || traq[3] >=2.35)
	{
		if(traq[3] <= 0.3){
			traq[3] =0.3;
			//flag=1;
		}
		else{
			traq[3]=2.35;
			//flag=1;
		}
	}
	if(traq[4] <= -1.35 || traq[4] >=1.25)
	{
		if(traq[4] <= -1.35){
			traq[4] =-1.35;
			//flag=1;
		}
		else{
			traq[4]=1.25;
			//flag=1;
		}
	}*/
	//Velocity & Acceleration limit
	for(int i=0;i<NUM_AXIS;i++)
	{
		if(abs(ActualVel_Rad[i])>4)
		{
			traq_d[i]=0;
			traq_dd[i]=0;
			limit_flag=1;
		}
	}
}
int isDriveInit(void)
{
	int elmo_count = 0;
	for(int i=0; i<NUM_AXIS; ++i)
	{
		if(ecat_nrmk_drive[i].initialized())
			elmo_count++;
	}
	if(elmo_count == NUM_AXIS)
		return 1;
	else
		return 0;
}




int compute()
{
	//FKin
	info.act.x.head(3)=cManipulator->pKin->ForwardKinematics();
	EulerAng = cManipulator->pKin->GetEulerAngle();
	b_jaco = cManipulator->pKin->BodyJacobian();
	l_jaco = cManipulator->pKin->LinearJacobian();
	l_jaco_dot = cManipulator->pKin->Jacobian_l_dot();
	DPI_l_jaco = cManipulator->pKin->DPI(l_jaco);
	info.act.x_dot.head(3) = l_jaco*info.act.j_q_d;

	//For InverseDynamics
	//mmat=cManipulator->pDyn->M_Matrix();
	//cmat=cManipulator->pDyn->C_Matrix();
	//adv=cManipulator->ad_V_Link(ActualVel_Rad);
	//gmat=cManipulator->pDyn->G_Matrix();

	for(int i=0;i<NUM_AXIS;i++)
	{
		ecat_nrmk_drive[i].mode_of_operation_ = ecat_nrmk_drive[i].MODE_CYCLIC_SYNC_TORQUE;
	}
	return 0;
}


// RTRArm_task
void RTRArm_run(void *arg)
{
	unsigned int runcount=0;
	RTIME now, previous;
	RTIME p1 = 0;
	RTIME p3 = 0;

	JointVec tauGrav, q_nom, qdot_nom;
	tauGrav.Zero(); q_nom.Zero(); qdot_nom.Zero();

	cManipulator->robot_update_R(); //update robot
	//ecatmaster.SyncEcatMaster(rt_timer_read());
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */

	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

	while (run)
	{

		runcount++;

		if (!run)
		{
			break;
		}

		previous = rt_timer_read();

		// [ToDo] Here is an error for PDO mapping
		ecatmaster.RxUpdate();
		if(system_ready)
		{
			//read the motor data
			for(int k=0; k<NUM_AXIS; ++k){

				DeviceState[k] = 			ecat_nrmk_drive[k].NRMK_Drive_DeviceState();
				StatusWord[k] = 			ecat_nrmk_drive[k].status_word_;
				ModeOfOperationDisplay[k] = ecat_nrmk_drive[k].mode_of_operation_display_;
				ControlWord[k] = 			ecat_nrmk_drive[k].control_word_;
				ActualPos[k] = 				ecat_nrmk_drive[k].position_-ActualPos_zero[k];
				ActualVel[k] = 				ecat_nrmk_drive[k].velocity_;
				ActualTor[k] = 				ecat_nrmk_drive[k].torque_;

			}

			// read FT
			iStatus = ecat_nrmk_indy_tool[0].iStatus_;
			iButton = ecat_nrmk_indy_tool[0].iButton_;
			FT_Raw_Fx = ecat_nrmk_indy_tool[0].FT_Raw_Fx_;
			FT_Raw_Fy = ecat_nrmk_indy_tool[0].FT_Raw_Fy_;
			FT_Raw_Fz = ecat_nrmk_indy_tool[0].FT_Raw_Fz_;
			FT_Raw_Tx = ecat_nrmk_indy_tool[0].FT_Raw_Tx_;
			FT_Raw_Ty = ecat_nrmk_indy_tool[0].FT_Raw_Ty_;
			FT_Raw_Tz = ecat_nrmk_indy_tool[0].FT_Raw_Tz_;
			FT_OverloadStatus = ecat_nrmk_indy_tool[0].FT_OverloadStatus_;
			FT_ErrorFlag = ecat_nrmk_indy_tool[0].FT_ErrorFlag_;


			EncToRad();
			cManipulator->pKin->Unflag_isInfoupdate();
			cManipulator->pKin->HTransMatrix(info.act.q);
			cManipulator->pDyn->Prepare_Dynamics(info.act.q, info.act.q_dot);

			_robotNom.idyn_gravity(LieGroup::Vector3D(0,0,-GRAV_ACC));

			compute();
			Robot_Limit();
			//cManipulator->pDyn->Prepare_Dynamics(traq, traq_d);

			/////////////Trajectory for Joint Space//////////////
            if(Motion==1 &&TrajFlag_j[0]==0)
            {
                TargetTrajPos_Rad[0]=1.5709; TargetTrajPos_Rad[1]=-0.7071; TargetTrajPos_Rad[2]= 0.7071; 
                TargetTrajPos_Rad[3]=1.57; TargetTrajPos_Rad[4]=1.5709; TargetTrajPos_Rad[5]=1.5709;
                Motion=1;
                traj_time = 5;
                for(int i=0;i<NUM_AXIS;i++)
                {
                    TrajFlag_j[i]=1;
                }
            }
            else if(Motion==2 &&TrajFlag_j[0]==0)
            {
                TargetTrajPos_Rad[0]=0.0; TargetTrajPos_Rad[1]=0.0; TargetTrajPos_Rad[2]= 0.0; 
                TargetTrajPos_Rad[3]=0.0; TargetTrajPos_Rad[4]=0.0; TargetTrajPos_Rad[5]=0.0;
                Motion++;
                traj_time = 5;
                for(int i=0;i<NUM_AXIS;i++)
                {
                    TrajFlag_j[i]=1;
                }
            }
            else if(Motion==3 &&TrajFlag_j[0]==0)
            {
                TargetTrajPos_Rad[0]=-1.5709; TargetTrajPos_Rad[1]=0.7071; TargetTrajPos_Rad[2]= -0.7071; 
                TargetTrajPos_Rad[3]=-1.57; TargetTrajPos_Rad[4]=-1.5709; TargetTrajPos_Rad[5]=-1.5709;
                Motion++;
                traj_time = 5;
                for(int i=0;i<NUM_AXIS;i++)
                {
                    TrajFlag_j[i]=1;
                }
            }
            else if(Motion==4 &&TrajFlag_j[0]==0)
            {
                TargetTrajPos_Rad[0]=0.0; TargetTrajPos_Rad[1]=0.0; TargetTrajPos_Rad[2]= 0.0; 
                TargetTrajPos_Rad[3]=0.0; TargetTrajPos_Rad[4]=0.0; TargetTrajPos_Rad[5]=0.0;
                Motion=1;
                traj_time = 5;
                for(int i=0;i<NUM_AXIS;i++)
                {
                    TrajFlag_j[i]=1;
                }
            }

			for(int i=0;i<NUM_AXIS;i++)
			{
				if(TrajFlag_j[i]==2)
				{
					traj5th_joint->Polynomial5th(i, double_gt, TrajFlag_j+i, q_);
					traq[i]=q_[0];
					traq_d[i]=q_[1];
					traq_dd[i]=q_[2];
				}
				else if(TrajFlag_j[i]==1)
				{
					traj5th_joint->SetPolynomial5th_j(i, &info.act, TargetTrajPos_Rad[i], double_gt, traj_time, q_);
					traq[i]=q_[0];
					traq_d[i]=q_[1];
					traq_dd[i]=q_[2];
					info.act.dq[i]=traq[i];
					info.act.dq_dot[i]=traq_d[i];
					info.act.dq_ddot[i]=traq_dd[i];

					TrajFlag_j[i]=2;
				}
			}
			Control->PD_Gravity(info.act.q, info.act.q_dot,traq, traq_d, TargetToq);

			//Gravity Controller
			//Control->Gravity(info.act.q, info.act.q_dot,TargetToq);

			//PD+Gravity Controller
/*traj*/		//Control->PD_Gravity(info.act.q, info.act.q_dot,traq, traq_d, TargetToq);
/*hold*/		//Control->PDController_gravity(ActualPos_Rad, ActualVel_Rad, TargetTrajPos_Rad, TargetVel_Rad, TargetToq, gmat);

			//Impedance Controller
			//Control->Impedance(info.act.j_q, info.act.j_q_d, info.act.j_q_dd, TargetPos_Rad, TargetVel_Rad, TargetAcc_Rad, TargetToq, mmat, gmat);
/*hold*/		//Control->Impedance(info.act.j_q, info.act.j_q_d, info.act.j_q_dd, TargetTrajPos_Rad, TargetVel_Rad, TargetAcc_Rad, TargetToq, mmat, gmat, cmat);
/*traj*/		//Control->Impedance(info.act.j_q, info.act.j_q_d, info.act.j_q_dd, traq, traq_d, traq_dd, TargetToq, mmat, gmat, cmat);
			//Control->Impedance(ActualPos_Rad, ActualVel_Rad, ActualAcc_Rad, TargetPos_Rad, TargetVel_Rad, TargetAcc_Rad, TargetToq, gmat, mmat, cmat);

			//IDC
/*hold*/		//Control->Inverse_Dynamics_Control(info.act.q, info.act.q_dot, TargetTrajPos_Rad, TargetVel_Rad, TargetAcc_Rad, TargetToq);
/*traj*/		//Control->Inverse_Dynamics_Control(info.act.q, info.act.q_dot, traq, traq_d, traq_dd, TargetToq);

			//CTC
/*hold*/		//Control->ComputedTorque(info.act.q, info.act.q_dot, TargetTrajPos_Rad, TargetVel_Rad, TargetAcc_Rad, TargetToq);
/*traj*/		//Control->ComputedTorque(info.act.q, info.act.q_dot, traq, traq_d, traq_dd, TargetToq);

			//VSD
			//Control->VSD(info.act.q, info.act.q_dot, xd, TargetToq);

			//Friction Identification
			//Control->FrictionIdentify(info.act.q, info.act.q_dot, traq, traq_d, traq_dd, TargetToq, double_gt);
//			}

			//else if(flag==1)
				//Control->FrictionIdentify(info.act.q, info.act.q_dot, traq, traq_d, traq_dd, TargetToq, double_gt);
				//Control->VSD(info.act.q, info.act.q_dot, xd, TargetToq);
				//Control->Gravity(info.act.q, info.act.q_dot,TargetToq);
				//Control->PD_Gravity(info.act.q, info.act.q_dot,traq, traq_d, TargetToq);
				//Control->ComputedTorque(info.act.q, info.act.q_dot, traq, traq_d, traq_dd, TargetToq);
			//else if(flag==2)//For Robot Limit
				//Control->Gravity(info.act.q, info.act.q_dot,TargetToq);



			Control->TorqueOutput(TargetToq, 1000, MotorDir);


			tauGrav = _robotNom->tau();

			//write the motor data
			for(int j=0; j<NUM_AXIS; ++j)
			{
				TargetTor[j] = round(TargetToq[j]);
				ecat_nrmk_drive[j].writeTorque(TargetTor[j]);

			}
			// ecat_nrmk_drive[0].writeTorque(TargetTor[0]);
			// ecat_nrmk_drive[1].writeTorque(TargetTor[1]);
			// ecat_nrmk_drive[2].writeTorque(TargetTor[2]);
			// ecat_nrmk_drive[3].writeTorque(TargetTor[3]);
			// ecat_nrmk_drive[5].writeTorque(TargetTor[5]);

		}

		ecatmaster.TxUpdate();
#if defined(USE_DC_MODE)
		ecatmaster.SyncEcatMaster(rt_timer_read());
#endif
		if (system_ready)
		{
			//saveLogData();
		}
		else
		{
			double_gt = 0;
			worst_time = 0;
			ethercat_time = 0;
		}
			

		// For EtherCAT performance statistics
		p1 = p3;
		p3 = rt_timer_read();
		now = rt_timer_read();
		double_gt += ((double)(long)(p3 - p1))*1e-9;
		ethercat_time = (long) now - previous;


		if ( isDriveInit() == 1 && (runcount > WAKEUP_TIME*(NSEC_PER_SEC/cycle_ns)))
		{
			system_ready=1;	//all drives have been done

			gt+= period;

			if (worst_time<ethercat_time)
				worst_time=ethercat_time;
			if(ethercat_time > (long)cycle_ns)
				++fault_count;
		}
		rt_task_wait_period(NULL); 	//wait for next cycle
	}
}

// Console cycle
// Note: You have to use rt_printf in Xenomai RT tasks
void print_run(void *arg)
{
	RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
	
	rt_printf("\e[31;1m \nPlease WAIT at least %i (s) until the system getting ready...\e[0m\n", WAKEUP_TIME);
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, 1e8);
	
	while (1)
	{
		if (++count==10)
		{
			++stick;
			count=0;
		}
		if (system_ready)
		{
			now = rt_timer_read();
			step=(unsigned long)(now - previous) / 1000000;
			itime+=step;
			previous=now;
			rt_printf("Time=%0.3fs \tFlag : %d\n", double_gt,flag);
			rt_printf("ethercat_dt= %lius, worst_dt= %lins, fault=%d\n", ethercat_time/1000, worst_time, fault_count);

			cout<<"Manipulability   : "<<cManipulator->pKin->Manipulability(l_jaco)<<endl;
			cout<<"Condition Number : "<<cManipulator->pKin->Condition_Number(l_jaco)<<endl;
			for(int j=0; j<NUM_AXIS; ++j){
				rt_printf("ID: %d", j+NUM_FT);
				rt_printf("\t CtrlWord: 0x%04X, ",		ControlWord[j]);
				rt_printf("\t StatWord: 0x%04X, \n",	StatusWord[j]);
			    rt_printf("\t DeviceState: %d, ",		DeviceState[j]);
				rt_printf("\t ModeOfOp: %d,	\n",		ModeOfOperationDisplay[j]);
				rt_printf("\t ActPos: %f, ActVel :%f \n",ActualPos_Rad[j], ActualVel_Rad[j]);
				//rt_printf("\t DesPos: %f, DesVel :%f, DesAcc :%f\n",TargetTrajPos_Rad[j],TargetVel_Rad[j],TargetAcc_Rad[j]);
				rt_printf("\t DesPos: %f, DesVel :%f, DesAcc :%f\n",traq[j],traq_d[j],traq_dd[j]);

				rt_printf("\t TarTor: %d, ",				TargetTor[j]);
				rt_printf("\t ActTor: %d,\n",			ActualTor[j]);
				rt_printf("Traj : %d,\n",               TrajFlag_j[j]);
			}
			rt_printf("\n");
			rt_printf("Fx : %f, Fy : %f, Fz: %f, Tx : %f, Ty: %f, Tz : %f\n",FT_Raw_Fx,FT_Raw_Fy,FT_Raw_Fz,FT_Raw_Tx,FT_Raw_Ty,FT_Raw_Tz);
			rt_printf("xd : %f, yd : %f, zd: %f\n",xd[0],xd[1],xd[2]);
			//rt_printf("xddot : %f, yddot : %f, zddot: %f\n",vive_vel[0],vive_vel[1],vive_vel[2]);
			//std::cout<<l_jaco<<endl;
			rt_printf("x : %f, y : %f, z : %f\n",info.act.x(0), info.act.x(1), info.act.x(2));

			rt_printf("r : %f, p : %f, y : %f\n",EulerAng(0),EulerAng(1),EulerAng(2));
			rt_printf("x : %f, y : %f, z : %f\n",vive_pos[0],vive_pos[1],vive_pos[2]);
			cout<<limit_flag<<endl;

		}
		else
		{
			if (count==0){
				rt_printf("%i", stick);
				for(i=0; i<stick; ++i)
					rt_printf(".");
				rt_printf("\n");
			}
		}

		rt_task_wait_period(NULL); //wait for next cycle
	}
}


void plot_run(void *arg)
{
	/*
	 * Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100 ms)
	 */
	rt_task_set_periodic(NULL, TM_NOW, 1e7);	// period = 10 (msec)


	while (1)
	{
		/// TO DO: You have to prepare data for NRMKDataSocket
/*		if (datasocket.hasConnection() && system_ready)
		{
			if (frontIdx < rearIdx)
			{
				datasocket.updateControlData(_loggingBuff[frontIdx].ActualPos, _loggingBuff[frontIdx].ActualVel);
				datasocket.update(_loggingBuff[frontIdx].Time);

				frontIdx++;
			}
			else if (rearIdx == MAX_BUFF_SIZE)
			{
				frontIdx = rearIdx = 0;
			}
		}
		else
		{
			frontIdx = rearIdx = 0;
		}
*/

		//usleep(1000);
		rt_task_wait_period(NULL);
	}
}

/****************************************************************************/
void signal_handler(int signum = 0)
{
	rt_task_delete(&plot_task);
	rt_task_delete(&RTRArm_task);
	rt_task_delete(&print_task);
    printf("\nServo drives Stopped!\n");

    ecatmaster.deactivate();
    exit(1);
}

/****************************************************************************/
int main(int argc, char **argv)
{

	// Perform auto-init of rt_print buffers if the task doesn't do so
    rt_print_auto_init(1);

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	int portNum=1; //COM as default
	char portName[100];
	unsigned int baud = 9600;
	switch (portNum)
	{
		case 0: strcpy(portName, RS485PORT); break;
		case 1: strcpy(portName, COM1); break;
		case 2: strcpy(portName, COM2); break;
		default: strcpy(portName, COM1); break;
	}
	if ((portNum>0) && (baud>RS232_BAUD_LIMIT))
		baud=RS232_BAUD_LIMIT;
	if (baud<1200)
		baud=1200;
	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
	cycle_ns = 1000000; // nanosecond -> 1kHz
	period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit


	_robotNom = new ROBOT(USERNAME, EMAIL, SERIAL);
	cManipulator = new robot;
	Control = new HYUControl::Controller(cManipulator,ROBOT_DOF);
	traj5th_joint =  new hyuCtrl::Trajectory();
	traj5th_task =  new hyuCtrl::Trajectory();

	for(int j=0; j<NUM_AXIS; ++j)
	{
		ecatmaster.addSlaveNRMK_Drive(0, j+1, &ecat_nrmk_drive[j]);
		ecat_nrmk_drive[j].mode_of_operation_ = ecat_nrmk_drive[j].MODE_CYCLIC_SYNC_TORQUE;
	}
	for(int i=0; i<NUM_FT; ++i)
		ecatmaster.addSlave(0, i+NUM_AXIS+1, &ecat_nrmk_indy_tool[i]);




#if defined(USE_DC_MODE)
	ecatmaster.activateWithDC(0, cycle_ns);  //a first arg DC location of MotorDriver?
	rtsercan_fd=SERCAN_open();
#else
	ecatmaster.activate();
#endif
	// TO DO: Create data socket server
	datasocket.setPeriod(period);

	if (datasocket.startServer(SOCK_TCP, NRMK_PORT_DATA))
		printf("Data server started at IP of : %s on Port: %d\n", datasocket.getAddress(), NRMK_PORT_DATA);

	printf("Waiting for Data Scope to connect...\n");
	datasocket.waitForConnection(0);
	

	// RTRArm_task: create and start
	printf("Now running rt task ...\n");
	rt_printf(" sercan_dev_open = %d\n", rtsercan_fd);

	rt_task_create(&RTRArm_task, "RTRArm_task", 0, 99, 0);
	rt_task_start(&RTRArm_task, &RTRArm_run, NULL);

	// printing: create and start
	rt_task_create(&print_task, "printing", 0, 70, 0);
	rt_task_start(&print_task, &print_run, NULL);
	
	// plotting: data socket comm
	rt_task_create(&plot_task, "plotting", 0, 80, 0);
	rt_task_start(&plot_task, &plot_run, NULL);

	// Must pause here
	pause();
	/*
	while (1)
	{
		usleep(1e5);
	}
	*/
	// Finalize
	signal_handler();

    return 0;
}



