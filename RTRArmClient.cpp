
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

hyuEcat::EcatRobotus ecat_robotusft[NUM_FT];
hyuEcat::EcatElmo ecat_elmo[NUM_AXIS];
hyuCtrl::Trajectory *traj5th_joint; //make an instance each of a joint
hyuCtrl::Trajectory *traj5th_task;
JointInfo info;
robot *cManipulator;
HYUControl::Controller *Control;




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
float float_gt=0;

int InitFlag[NUM_AXIS] = {0,};
int TrajFlag_j[NUM_AXIS] = {0,};
int TrajFlag_t[NUM_AXIS] = {0,};
int TrajFlag_homing    = 1;


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

float DF=50.0;
float DT=2000.0;
float Tx[NUM_FT]={0.0};
float Ty[NUM_FT]={0.0};
float Tz[NUM_FT]={0.0};
float Fx[NUM_FT]={0.0};
float Fy[NUM_FT]={0.0};
float Fz[NUM_FT]={0.0};
float ZMP_X[2]={0,};
float ZMP_Y[2]={0,};


//////////////////////////
/****************************************************************************/

// Xenomai RT tasks
RT_TASK RTRArm_task;
RT_TASK print_task;
RT_TASK plot_task;
RT_TASK SNUHand_task;
RT_TASK TCP_task;

// For RT thread management
static int run = 1;
unsigned long fault_count=0;
long ethercat_time=0, worst_time=0;
#define min_time	0
#define max_time	100000
#define hist_step	(100)
unsigned int histdata[hist_step+1];
unsigned int interval_size=350;

float ActualPos_Rad[NUM_AXIS] = {0.0,};
float ActualVel_Rad[NUM_AXIS] = {0.0,};
float ActualAcc_Rad[NUM_AXIS] = {0.0,};
float TargetPos_Rad[NUM_AXIS] = {0.0,};
float TargetVel_Rad[NUM_AXIS] = {0.0,};
float TargetAcc_Rad[NUM_AXIS] = {0.0,};
float TargetToq[NUM_AXIS] = {0.0,};
INT32   MotorDir[NUM_AXIS] = {1,1,1,1,1,1};
int   old_flag[NUM_AXIS]={0,};
int   acc_flag[NUM_AXIS]={0,};
Jointf gmat;
Matrixf mmat;
Matrixf cmat;
Matrix6n6nf adv;

Jaco b_jaco;
InvJaco b_inv;
LinJaco l_jaco, l_jaco_dot;
PinvLJaco DPI_l_jaco;
se3 wrench_test;
Jointf torq;
Vector3f EulerAng;
float time_buf;
int time_flag=0;
int limit_flag=0;


//For Homing management
int j_homing = 0;
int HomingFlag = 0;
int j_=1;


//For Trajectory management
//Joint
int NUM_MOTION=1;
int Motion=0;
float TargetTrajPos_Rad[NUM_AXIS]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};/////////////////////////must change/////////////////////////////////
//float TargetTrajPos_Rad[NUM_AXIS]={0.0, -1.57, 0.0, 1.57, 0.0};
float traj_time = 3;
int traj_changed = 0;
//Task
float TargetTrajPos_Meter[NUM_AXIS]={0,0,};
Vector3f xd, xd_dot, xd_ddot;
Vector3f x_buf;
Vector3f vive_pos;
Vector3f vive_vel;
Vector3f vive_buf;
int vive_flag=0;
float K_tracking_ = 1;
int task_time=0;

Matrix<float,NUM_AXIS,1> qd,qd_dot,qd_ddot ,qd_old, qd_dot_old;
float q_[3]={0.0,};
float traq[NUM_AXIS]={0.0, 0.0, 0.0, -1.5709, 0.0, 0.0};
float traq_d[NUM_AXIS]={0.0,};
float traq_dd[NUM_AXIS]={0.0,};
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

//For TCP-Server
int current_Thread=0;
float received_Data[NUM_JOINT*3];
int flag_TCP=0;
int flag_ctrl=2;
int ctrl_buf=-1;
int TCP_buf=0;
int TCP_Homing = 0;
int TCP_Limit_cnt = 0;


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

class Session : public TCPServerConnection
{
public:
	enum {
		SIZE_HEADER = 52,
		SIZE_COMMAND = 4,
		SIZE_HEADER_COMMAND = 56,
		SIZE_DATA_MAX = 200,
		SIZE_DATA_ASCII_MAX = 32
	};
	Session(const StreamSocket &socket) :TCPServerConnection(socket) {
		//cout << "Session Object Construct" << endl;
	}
	virtual ~Session() {
		//cout << "Session Object Destruct" << endl;
	}
	union Data
	{
		unsigned char byte[SIZE_DATA_MAX];
		float float6dArr[12];
	};
	Data data_rev;
	Data data;

	virtual void run()
	{
		unsigned char readBuff[1024];
		unsigned char writeBuff[1024];

		try
		{
			int recvSize = 0;
			do
			{
				memcpy(writeBuff, data.byte, SIZE_DATA_MAX);

				Timestamp now;
				recvSize = socket().receiveBytes(readBuff, 1024);

				memcpy(data_rev.byte, readBuff, SIZE_DATA_MAX);
				//Received
				received_Data[0] = data_rev.float6dArr[0];   // Motion
				received_Data[1] = data_rev.float6dArr[1];   // desired_01
				received_Data[2] = data_rev.float6dArr[2];   // desired_02
				received_Data[3] = data_rev.float6dArr[3];   // desired_03
				received_Data[4] = data_rev.float6dArr[4];   // desired_04
				received_Data[5] = data_rev.float6dArr[5];   // desired_05
				received_Data[6] = data_rev.float6dArr[6];   // desired_06
				received_Data[7] = data_rev.float6dArr[7];   // null
				if(flag_TCP==0)
					flag_TCP=1;

				//Return
				if(HomingFlag == 1 && TCP_Homing == 1)
				{
					data.float6dArr[0] = 101.0;
					data.float6dArr[1] = 1.0;
					data.float6dArr[2] = 0.0;
					data.float6dArr[3] = 0.0;
					data.float6dArr[4] = 0.0;
					data.float6dArr[5] = 0.0;
					data.float6dArr[6] = 0.0;
					data.float6dArr[7] = 0.0;
					TCP_Homing=0;
				}
				else if(limit_flag == 1 && TCP_Limit_cnt == 0)
				{
					data.float6dArr[0] = 102.0;
					data.float6dArr[1] = 1.0;
					data.float6dArr[2] = 0.0;
					data.float6dArr[3] = 0.0;
					data.float6dArr[4] = 0.0;
					data.float6dArr[5] = 0.0;
					data.float6dArr[6] = 0.0;
					data.float6dArr[7] = 0.0;
					TCP_Limit_cnt=1;
				}
				else
				{
					data.float6dArr[0] = 0.0;
					data.float6dArr[1] = 0.0;
					data.float6dArr[2] = 0.0;
					data.float6dArr[3] = 0.0;
					data.float6dArr[4] = 0.0;
					data.float6dArr[5] = 0.0;
					data.float6dArr[6] = 0.0;
					data.float6dArr[7] = 0.0;
				}

				memcpy(writeBuff, data.byte, SIZE_DATA_MAX);
				socket().sendBytes(writeBuff, 1024);

			} while (recvSize > 0);
			//cout << "Client Disconnected." << endl;
		}
		catch (Poco::Exception& exc)
		{
			//cout << "Session: " << exc.displayText() << endl;
		}

	}
};

class SessionFactory :public TCPServerConnectionFactory
{
public:
	SessionFactory() {}
	virtual ~SessionFactory() {}

	virtual TCPServerConnection* createConnection(const StreamSocket &socket)
	{
		return new Session(socket);
	}
};

/****************************************************************************/
void EncToRad()
{

		for(int i=1; i<=NUM_AXIS; i++)
		{
			if(old_flag[i-1]==0){
				ActualPos_Old[i-1]=ActualPos[i-1];
				old_flag[i-1]=1;
			}
			else if(old_flag[i-1]==1){

				ActualVel[i-1]=(2*0.01-0.001)/(2*0.01+0.001)*ActualVel_Old[i-1]+2/(2*0.01+0.001)*(ActualPos[i-1]-ActualPos_Old[i-1]);

				ActualPos_Rad[i-1]=(float)ActualPos[i-1]/(41721.5134018818*100.0); //262144/2PI
				ActualVel_Rad[i-1]=(float)ActualVel[i-1]/(41721.5134018818*100.0);

				//for Kinematics & Dynamics
				info.act.q[i-1]=ActualPos_Rad[i-1];
				info.act.q_dot[i-1]=ActualVel_Rad[i-1];

				//buffer for calculate velocity
				ActualPos_Old[i-1]=ActualPos[i-1];
				ActualVel_Old[i-1]=ActualVel[i-1];
			}
		}
		info.act.j_q=Map<VectorXf>(info.act.q,NUM_AXIS);
		info.act.j_q_d=Map<VectorXf>(info.act.q_dot,NUM_AXIS);
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
			flag_ctrl=-1;
			limit_flag=1;
		}
	}
}
int isElmoInit(void)
{
	int elmo_count = 0;
	for(int i=0; i<NUM_AXIS; ++i)
	{
		if(ecat_elmo[i].initialized())
			elmo_count++;
	}
	if(elmo_count == NUM_AXIS)
		return 1;
	else
		return 0;
}



int isElmoHoming(void)
{
	if(j_homing == 0)
	{
		HomingFlag = 1;
		for(int i=0;i<NUM_AXIS;i++){
			TrajFlag_j[i] = 0;
		}
		TrajFlag_homing=1;
		j_homing=-1;
		return 0;
	}
	if(ecat_elmo[j_homing-1].isHoming() && TrajFlag_j[j_homing-1] == 2)
	{
		TargetPos[j_homing-1] = (int)roundf(traj5th_joint->Polynomial5th(j_homing-1, float_gt, TrajFlag_j+j_homing-1));
		ecat_elmo[j_homing-1].target_position_ = TargetPos[j_homing-1];
	}
	else if(ecat_elmo[j_homing-1].isHoming() && TrajFlag_j[j_homing-1] == 1) //
	{
		ecat_elmo[j_homing-1].mode_of_operation_ = ecat_elmo[j_homing-1].MODE_CYCLIC_SYNC_POSITION;
		ecat_elmo[j_homing-1].target_position_ = ActualPos[j_homing-1];    //Keep the end of motion
		traj5th_joint->SetPolynomial5th(j_homing-1, (float)ActualPos[j_homing-1], (float)HomePos[j_homing-1], float_gt, 3); //make trajectory to go to the zero position
		TrajFlag_j[j_homing-1] = 2;

		return 0;
	}
	else if(!ecat_elmo[j_homing-1].isHoming())  //check homing and change the mode to homing mode
	{
		ecat_elmo[j_homing-1].mode_of_operation_ = ecat_elmo[j_homing-1].MODE_HOMING;
		TrajFlag_j[j_homing-1] = 1;

		return 0;
	}
	else if(ecat_elmo[j_homing-1].isHoming() && TrajFlag_j[j_homing-1]==0)
	{
		if(j_homing == 6)
			j_homing = 5;
		else if(j_homing == 5)
			j_homing = 3;
		else if(j_homing == 3)
			j_homing = 4;
		else if(j_homing == 4)
			j_homing = 1;
		else if(j_homing == 1)
			j_homing = 2;
		else if(j_homing == 2)
			j_homing = 0;
	}
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


	if(TrajFlag_homing==1) //complete homing and convert to torque mode
	{
		for(int i=0;i<NUM_AXIS;i++)
		{
			ecat_elmo[i].mode_of_operation_ = ecat_elmo[i].MODE_CYCLIC_SYNC_TORQUE;
		}
		TrajFlag_homing=2;
	}
	else if(TrajFlag_homing==2)
	{

	}
	return 0;
}

void TCP_Command()
{
	if(flag_TCP==1)
	{
		switch((int)received_Data[0])
		{
		// 1) Joint Space
		case 101: // Homing
			if(TCP_buf!=101)
			{
			HomingFlag=0;
			j_homing=6;
			TrajFlag_homing=0;
			for(int i=0;i<ROBOT_DOF;i++)
				ecat_elmo[i].reHoming();
			TCP_Homing=1;
			flag_ctrl=-1;
			}
			TCP_buf=(int)received_Data[0];
			break;
		case 102: // Reset (when occured limit)
			for(int i = 0; i<ROBOT_DOF;i++)
				TrajFlag_j[i] = 0;
			limit_flag=0;
			TCP_Limit_cnt = 0;
			flag_ctrl=-1;
			TCP_buf=(int)received_Data[0];
			break;
		case 103: // Rest-Position
			if(TrajFlag_j[0] == 0 && limit_flag == 0)
			{
				TargetTrajPos_Rad[0]=0.0;
				TargetTrajPos_Rad[1]=0.0;
				TargetTrajPos_Rad[2]=0.0;
				TargetTrajPos_Rad[3]=0.0;
				TargetTrajPos_Rad[4]=0.0;
				TargetTrajPos_Rad[5]=0.0;
				traj_time=2.0;
				for(int i=0;i<NUM_AXIS;i++)
				{
					TrajFlag_j[i]=1;
				}
				flag_ctrl=0;
			}
			else if(TCP_buf!=(int)received_Data[0] && limit_flag == 0)// if(TargetTrajPos_Rad[0]!=0.0 || TargetTrajPos_Rad[1]!=0.0 || TargetTrajPos_Rad[2]!=0.0 || TargetTrajPos_Rad[3]!=0.0 || TargetTrajPos_Rad[4]!=0.0 || TargetTrajPos_Rad[5]!=0.0)
			{
				TargetTrajPos_Rad[0]=0.0;
				TargetTrajPos_Rad[1]=0.0;
				TargetTrajPos_Rad[2]=0.0;
				TargetTrajPos_Rad[3]=0.0;
				TargetTrajPos_Rad[4]=0.0;
				TargetTrajPos_Rad[5]=0.0;
				traj_time=2.0;
				for(int i=0;i<NUM_AXIS;i++)
				{
					TrajFlag_j[i]=1;
				}
				flag_ctrl=0;
				traj_changed = 1;
			}
			TCP_buf=(int)received_Data[0];
			break;
		case 104: // Home-Position
			if(TrajFlag_j[0] == 0 && limit_flag == 0)
			{
				TargetTrajPos_Rad[0]=0.0;
				TargetTrajPos_Rad[1]=0.0;
				TargetTrajPos_Rad[2]=0.0;
				TargetTrajPos_Rad[3]=-1.5709;
				TargetTrajPos_Rad[4]=0.0;
				TargetTrajPos_Rad[5]=0.0;
				traj_time=2.0;
				for(int i=0;i<NUM_AXIS;i++)
				{
					TrajFlag_j[i]=1;
				}
				flag_ctrl=0;
			}
			else if(TCP_buf!=(int)received_Data[0] && limit_flag == 0)// if(TargetTrajPos_Rad[0]!=0.0 || TargetTrajPos_Rad[1]!=0.0 || TargetTrajPos_Rad[2]!=0.0 || TargetTrajPos_Rad[3]!=-1.5709 || TargetTrajPos_Rad[4]!=0.0 || TargetTrajPos_Rad[5]!=0.0)
			{
				TargetTrajPos_Rad[0]=0.0;
				TargetTrajPos_Rad[1]=0.0;
				TargetTrajPos_Rad[2]=0.0;
				TargetTrajPos_Rad[3]=-1.5709;
				TargetTrajPos_Rad[4]=0.0;
				TargetTrajPos_Rad[5]=0.0;
				traj_time=2.0;
				for(int i=0;i<NUM_AXIS;i++)
				{
					TrajFlag_j[i]=1;
				}
				flag_ctrl=0;
				traj_changed = 1;
			}
			TCP_buf=(int)received_Data[0];
			break;
		case 105: // Job-Position
			if(TrajFlag_j[0] == 0 && limit_flag == 0)
			{
				TargetTrajPos_Rad[0]=-0.634884;
				TargetTrajPos_Rad[1]=-0.43264;
				TargetTrajPos_Rad[2]=1.273272;
				TargetTrajPos_Rad[3]=-1.69791;
				TargetTrajPos_Rad[4]=0.74784;
				TargetTrajPos_Rad[5]=0.74;
				traj_time=2.0;
				for(int i=0;i<NUM_AXIS;i++)
				{
					TrajFlag_j[i]=1;
				}
				flag_ctrl=0;
			}
			else if(TCP_buf!=(int)received_Data[0] && limit_flag == 0)// if(TargetTrajPos_Rad[0]!=-0.634884 || TargetTrajPos_Rad[1]!=-0.43264 || TargetTrajPos_Rad[2]!=1.273272 || TargetTrajPos_Rad[3]!=-1.69791 || TargetTrajPos_Rad[4]!=0.74784 || TargetTrajPos_Rad[5]!=0.74)
			{
				TargetTrajPos_Rad[0]=-0.634884;
				TargetTrajPos_Rad[1]=-0.43264;
				TargetTrajPos_Rad[2]=1.273272;
				TargetTrajPos_Rad[3]=-1.69791;
				TargetTrajPos_Rad[4]=0.74784;
				TargetTrajPos_Rad[5]=0.74;
				traj_time=2.0;
				for(int i=0;i<NUM_AXIS;i++)
				{
					TrajFlag_j[i]=1;
				}
				flag_ctrl=0;
				traj_changed = 1;
			}
			TCP_buf=(int)received_Data[0];
			break;
		case 106: // Joint desired(fifth-order polynomial)
			if(TrajFlag_j[0] == 0 && limit_flag == 0)
			{
				TargetTrajPos_Rad[0]=received_Data[1];
				TargetTrajPos_Rad[1]=received_Data[2];
				TargetTrajPos_Rad[2]=received_Data[3];
				TargetTrajPos_Rad[3]=received_Data[4];
				TargetTrajPos_Rad[4]=received_Data[5];
				TargetTrajPos_Rad[5]=received_Data[6];
				if(received_Data[7]<1.0)
					traj_time=1.0;
				else
					traj_time=received_Data[7];
				for(int i=0;i<NUM_AXIS;i++)
				{
					TrajFlag_j[i]=1;
				}
				flag_ctrl=0;
			}
			else if(TargetTrajPos_Rad[0]!=received_Data[1] || TargetTrajPos_Rad[1]!=received_Data[2] || TargetTrajPos_Rad[2]!=received_Data[3] || TargetTrajPos_Rad[3]!=received_Data[4] || TargetTrajPos_Rad[4]!=received_Data[5] || TargetTrajPos_Rad[5]!=received_Data[6])
			{
				if(limit_flag == 0)
				{
					TargetTrajPos_Rad[0]=received_Data[1];
					TargetTrajPos_Rad[1]=received_Data[2];
					TargetTrajPos_Rad[2]=received_Data[3];
					TargetTrajPos_Rad[3]=received_Data[4];
					TargetTrajPos_Rad[4]=received_Data[5];
					TargetTrajPos_Rad[5]=received_Data[6];
					if(received_Data[7]<1.0)
						traj_time=1.0;
					else
						traj_time=received_Data[7];
					for(int i=0;i<NUM_AXIS;i++)
					{
						TrajFlag_j[i]=1;
					}
					flag_ctrl=0;
					traj_changed = 1;
				}
			}
			TCP_buf=(int)received_Data[0];
			break;
		// 2) Task Space
/*		case 201:
			xd[0]=info.act.x[0];
			xd[1]=info.act.x[1];
			xd[2]=info.act.x[2];
			xd[0] += 0.10000;
			flag_ctrl=1;
			TCP_buf=(int)received_Data[0];
			break;
		case 202:
			xd[0]=info.act.x[0];
			xd[1]=info.act.x[1];
			xd[2]=info.act.x[2];
			xd[0] -= 0.10000;
			flag_ctrl=1;
			TCP_buf=(int)received_Data[0];
			break;
		case 203:
			xd[0]=info.act.x[0];
			xd[1]=info.act.x[1];
			xd[2]=info.act.x[2];
			xd[1] += 0.10000;
			flag_ctrl=1;
			TCP_buf=(int)received_Data[0];
			break;
		case 204:
			xd[0]=info.act.x[0];
			xd[1]=info.act.x[1];
			xd[2]=info.act.x[2];
			xd[1] -= 0.10000;
			flag_ctrl=1;
			TCP_buf=(int)received_Data[0];
			break;
		case 205:
			xd[0]=info.act.x[0];
			xd[1]=info.act.x[1];
			xd[2]=info.act.x[2];
			xd[2] += 0.10000;
			flag_ctrl=1;
			TCP_buf=(int)received_Data[0];
			break;
		case 206:
			xd[0]=info.act.x[0];
			xd[1]=info.act.x[1];
			xd[2]=info.act.x[2];
			xd[2] -= 0.10000;
			flag_ctrl=1;
			TCP_buf=(int)received_Data[0];
			break;*/
		case 201:
			xd[0]=received_Data[1];
			xd[1]=received_Data[2];
			xd[2]=received_Data[3];
			flag_ctrl=1;
			TCP_buf=(int)received_Data[0];
/*			if(TrajFlag_t[0] == 0)
			{
				TargetTrajPos_Meter[0]=received_Data[1];
				TargetTrajPos_Meter[1]=received_Data[2];
				TargetTrajPos_Meter[2]=received_Data[3];

				if(received_Data[4]<1.5)
					traj_time=3.0;
				else
					traj_time=received_Data[4];
				for(int i=0;i<NUM_AXIS;i++)
				{
					TrajFlag_t[i]=1;
				}
				flag_ctrl=1;
			}
			TCP_buf=(int)received_Data[0];*/
			break;
		// 3) SNU Hand
		case 301:
			TCP_buf=(int)received_Data[0];
			break;
		case 302:
			TCP_buf=(int)received_Data[0];
			break;
		default:
			break;
		}
	}
}

// RTRArm_task
void RTRArm_run(void *arg)
{
	unsigned int runcount=0;
	RTIME now, previous;
	RTIME p1 = 0;
	RTIME p3 = 0;

	cManipulator->robot_update_R(); //update robot
	//ecatmaster.SyncEcatMaster(rt_timer_read());
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */

	//// For Plot data
	string filePath1 = "Position.csv";
	ofstream writeFile1(filePath1.data());
	string filePath2 = "Torque.csv";
	ofstream writeFile2(filePath2.data());
	int cnt_plot=0;

	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

	while (run)
	{
		stringstream s_Pos[25], s_Tor[16];
		runcount++;

		if (!run)
		{
			break;
		}

		previous = rt_timer_read();

		ecatmaster.RxUpdate();
		if(system_ready)
		{
			//read the motor data
			for(int k=0; k<NUM_AXIS; ++k){

				DeviceState[k] = 			ecat_elmo[k].Elmo_DeviceState();
				StatusWord[k] = 			ecat_elmo[k].status_word_;
				ModeOfOperationDisplay[k] = ecat_elmo[k].mode_of_operation_display_;
				ControlWord[k] = 			ecat_elmo[k].control_word_;
				ActualPos[k] = 				ecat_elmo[k].position_;
				//ActualVel[k] = 				ecat_elmo[k].velocity_;
				ActualTor[k] = 				ecat_elmo[k].torque_;
			}
			//ReadFT(0);
			EncToRad();
			cManipulator->pKin->Unflag_isInfoupdate();
			cManipulator->pKin->HTransMatrix(info.act.q);
			cManipulator->pDyn->Prepare_Dynamics(info.act.q, info.act.q_dot);

			TCP_Command();

			isElmoHoming();
			//if(HomingFlag==0)
			//{
				//isElmoHoming();
			//	HomingFlag=1;
			//	TrajFlag_homing=1;
			//}
			if(HomingFlag==1)
			{
				compute();
				Robot_Limit();
				//cManipulator->pDyn->Prepare_Dynamics(traq, traq_d);

				switch(flag_ctrl)
				{
				case -1: // Gravity Compensator
					Control->Gravity(info.act.q, info.act.q_dot,TargetToq);
					break;
				case 0: // Joint Space
					for(int i=0;i<NUM_AXIS;i++)
					{
						if(TrajFlag_j[i]==2)
						{
							traj5th_joint->Polynomial5th(i, float_gt, TrajFlag_j+i, q_);
							traq[i]=q_[0];
							traq_d[i]=q_[1];
							traq_dd[i]=q_[2];
							if(TrajFlag_j[0]==0)
								flag_TCP=0;
						}
						else if(TrajFlag_j[i]==1)
						{
							info.act.dq[i]=traq[i];
							info.act.dq_dot[i]=traq_d[i];
							info.act.dq_ddot[i]=traq_dd[i];
							traj5th_joint->SetPolynomial5th_j(i, &info.act, TargetTrajPos_Rad[i], float_gt, traj_time, q_, traj_changed);
							traq[i]=q_[0];
							traq_d[i]=q_[1];
							traq_dd[i]=q_[2];
							TrajFlag_j[i]=2;
							traj_changed = 0;
						}
					}
					Control->PD_Gravity(info.act.q, info.act.q_dot,traq, traq_d, TargetToq);
					break;
				case 1: // Task Space
					if(ctrl_buf!=1)
						task_time=0;
					else
						task_time=1;
					Control->VSD(info.act.q, info.act.q_dot, xd, TargetToq, float_gt, task_time);
					break;
				default:
					break;
				}
				ctrl_buf=flag_ctrl;
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
				//Control->FrictionIdentify(info.act.q, info.act.q_dot, traq, traq_d, traq_dd, TargetToq, float_gt);
	//			}

				//else if(flag==1)
					//Control->FrictionIdentify(info.act.q, info.act.q_dot, traq, traq_d, traq_dd, TargetToq, float_gt);
					//Control->VSD(info.act.q, info.act.q_dot, xd, TargetToq);
					//Control->Gravity(info.act.q, info.act.q_dot,TargetToq);
					//Control->PD_Gravity(info.act.q, info.act.q_dot,traq, traq_d, TargetToq);
					//Control->ComputedTorque(info.act.q, info.act.q_dot, traq, traq_d, traq_dd, TargetToq);
				//else if(flag==2)//For Robot Limit
					//Control->Gravity(info.act.q, info.act.q_dot,TargetToq);



				Control->TorqueOutput(TargetToq, 1000, MotorDir);
			}



			//write the motor data
			for(int j=0; j<NUM_AXIS; ++j)
			{
				TargetTor[j] = roundf(TargetToq[j]);
				ecat_elmo[j].writeTorque(TargetTor[j]);

				s_Pos[j]<<traq_d[j];
				s_Pos[j+NUM_AXIS]<<info.act.q_dot[j];
				s_Pos[j+2*NUM_AXIS]<<traq[j];
				s_Pos[j+3*NUM_AXIS]<<info.act.q[j];
				s_Tor[j]<<ActualTor[j];
				s_Tor[j+NUM_AXIS]<<info.act.q_dot[j];
			}
			//ecat_elmo[0].writeTorque(TargetTor[0]);
			//ecat_elmo[1].writeTorque(TargetTor[1]);
			//ecat_elmo[2].writeTorque(TargetTor[2]);
			//ecat_elmo[3].writeTorque(TargetTor[3]);
			//ecat_elmo[4].writeTorque(TargetTor[4]);

			//saveLogData(float_gt);

			if(writeFile1.is_open()&&writeFile2.is_open()&&cnt_plot>=9)
			{
				for(int j=0;j<NUM_AXIS; ++j)
				{
					writeFile1<<s_Pos[j].str()<<", "<<s_Pos[j+NUM_AXIS].str()<<", "<<s_Pos[j+2*NUM_AXIS].str()<<", "<<s_Pos[j+3*NUM_AXIS].str()<<", ";
					writeFile2<<s_Tor[j].str()<<", "<<s_Tor[j+NUM_AXIS].str()<<", ";
				}
				writeFile1<<"\n";
				writeFile2<<"\n";
				cnt_plot=0;
			}
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
			float_gt = 0;
			worst_time = 0;
			ethercat_time = 0;
		}
			

		// For EtherCAT performance statistics
		p1 = p3;
		p3 = rt_timer_read();
		now = rt_timer_read();
		float_gt += ((float)(long)(p3 - p1))*1e-9;
		ethercat_time = (long) now - previous;


		if ( isElmoInit() == 1 && (runcount > WAKEUP_TIME*(NSEC_PER_SEC/cycle_ns)))
		{
			system_ready=1;	//all drives have been done

			gt+= period;

			if (worst_time<ethercat_time)
				worst_time=ethercat_time;
			if(ethercat_time > (long)cycle_ns)
				++fault_count;
		}
		rt_task_wait_period(NULL); 	//wait for next cycle
		cnt_plot++;
	}
	writeFile1.close();
	writeFile2.close();
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
			rt_printf("Time=%0.3fs \tFlag : %d\n", float_gt,flag);
			rt_printf("ethercat_dt= %lius, worst_dt= %lins, fault=%d, current_thread=%d\n", ethercat_time/1000, worst_time, fault_count,current_Thread);

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
				rt_printf("homing : %d\n",ecat_elmo[j].isHoming());
			}
			rt_printf("\n");
			//rt_printf("%d\n",flag_ctrl);
			rt_printf("%d\n",open_state);
			rt_printf("%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",received_Data[0],received_Data[1],received_Data[2],received_Data[3],received_Data[4],received_Data[5],received_Data[6],received_Data[7]);
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

//Robo-Limb task
void SNUHand_run(void *arg)
{
	RoboLimb robolimb(rtsercan_fd);
	//rt_printf("RoboLimb is run!\n");
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);	//cycle: 50us
	int hand_buff=0;
	CAN_FRAME txframe;
	HandPacket txpacket;
	while(1)
	{
//		if(Motion==11 && hand_flag==1)
		if((int)received_Data[0]==302 && hand_buff!=302)
		{
			txframe.can_id = 626;
			txframe.can_dlc = 8;
			txpacket.info.reserve = 0x00;
			txpacket.info.opt = HAND_OPEN;

			txpacket.info.data = 277;
			txframe.data[0] = 72;
			txframe.data[1] = 0x93;
			txframe.data[2] = 0x60;
			txframe.data[3] = 3;
			txframe.data[4]	= 1;
			txframe.data[5]	= 0;
			txframe.data[6]	= 0;
			txframe.data[7]	= 0;

			SERCAN_write(rtsercan_fd, txframe);
			//print_CANFrame(txframe);
			usleep(2000);
			hand_flag=0;
			//open_state=1;
		}
//		else if(Motion==6 && hand_flag==1)
		else if((int)received_Data[0]==301 && hand_buff!=301)
		{
			txframe.can_id = 626;
			txframe.can_dlc = 8;
			txpacket.info.reserve = 0x00;
			txpacket.info.opt = HAND_CLOSE;

			txpacket.info.data = 277;
			txframe.data[0] = 72;
			txframe.data[1] = 0x93;
			txframe.data[2] = 0x60;
			txframe.data[3] = 3;
			txframe.data[4]	= 2;
			txframe.data[5]	= 0;
			txframe.data[6]	= 0;
			txframe.data[7]	= 0;
			SERCAN_write(rtsercan_fd, txframe);
			//print_CANFrame(txframe);
			usleep(2000);
			hand_flag=0;
			//open_state=2;
		}
		hand_buff=(int)received_Data[0];

/*		robolimb.IdleHand();
		//robolimb.CloseHand();
		sleep(5);
		robolimb.IdleHand();
		//robolimb.OpenHand();
		sleep(5);
		robolimb.ReadFinger(finger_state);
		*/
		rt_task_wait_period(NULL);
		//usleep(20);
	}

/*void gpio_out_run(void *arg)
{
	int state=0;
	rt_task_set_periodic(NULL, TM_NOW, 5e4);	//cycle: 50us
	while (1)
	{
		if (state==0)
			tp_writeport(port0, port0);
		else
			tp_writeport(port0, port1);
		state^=1;
		rt_task_wait_period(NULL);
	}
}*/
/*void gpio_out_run(void *arg)
{
	int state=0;
	uint8_t inputState0,inputState1,inputState2;
	rt_task_set_periodic(NULL, TM_NOW, 5e4);	//cycle: 100us

	while(1)
	{
		inputState0 = tp_readport(port0);
		if(inputState0!=255)
			printf("%d\n",inputState0);
		//usleep(2000);
	}
*/
/*	while (1)
	{
		if (state==0)
			tp_writeport(port0, port0);
		else
			tp_writeport(port0, port1);
		state^=1;
		rt_task_wait_period(NULL);
	}




}
*/
}

void TCP_run(void *arg)
{
	ServerSocket sock(SERVER_PORT);
	TCPServer server(new SessionFactory(), sock);
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);	//cycle: 50us
	cout << "Simple TCP Server Application." << endl;
	cout << "maxConcurrentConnections: " << server.maxConcurrentConnections() << endl;

	server.start();
	while (true)
	{
		current_Thread = server.currentConnections();
		rt_task_wait_period(NULL);

	}
}


/****************************************************************************/
void signal_handler(int signum = 0)
{
	rt_task_delete(&plot_task);
	rt_task_delete(&RTRArm_task);
	rt_task_delete(&print_task);
	rt_task_delete(&SNUHand_task);
	rt_task_delete(&TCP_task);
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

	cManipulator = new robot;
	Control = new HYUControl::Controller(cManipulator,ROBOT_DOF);
	traj5th_joint =  new hyuCtrl::Trajectory();
	traj5th_task =  new hyuCtrl::Trajectory();

	for(int j=0; j<NUM_AXIS; ++j)
	{
		ecatmaster.addSlaveElmo(0, j, &ecat_elmo[j]);
		ecat_elmo[j].mode_of_operation_ = ecat_elmo[j].MODE_CYCLIC_SYNC_TORQUE;
	}
	for(int i=0; i<NUM_FT; ++i)
		ecatmaster.addSlave(0, i+NUM_AXIS, &ecat_robotusft[i]);




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

	rt_task_create(&SNUHand_task, "SNUHand", 0, 90, 0);
	rt_task_start(&SNUHand_task, &SNUHand_run, NULL);

	rt_task_create(&TCP_task, "TCP", 0, 85, 0);
	rt_task_start(&TCP_task, &TCP_run, NULL);

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



