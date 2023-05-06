#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/PhysicsClientSharedMemory_C_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include <Eigen/Dense>
#include "Utils/b3Clock.h"
#include "Indy7.h"

#include "../../include/MR/modern_robotics.h"
#include "../../include/MR/MR_Indy7.h"
#include <vector>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace mr;


extern const int CONTROL_RATE;
const int CONTROL_RATE = 1000;
const b3Scalar FIXED_TIMESTEP = 1.0 / ((b3Scalar)CONTROL_RATE);
b3SharedMemoryCommandHandle command;
int statusType, ret;
MR_Indy7 control;

int main()
{
	control=MR_Indy7();
	control.MRSetup();
	cout<<"START PROGRAM"<<endl;

	b3PhysicsClientHandle client = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
	if (!b3CanSubmitCommand(client))
	{
	printf("Not connected, start a PyBullet server first, using python -m pybullet_utils.runServer\n");
	exit(0);
	}
	b3RobotSimulatorClientAPI_InternalData data;
	data.m_physicsClientHandle = client;
	data.m_guiHelper = 0;
	b3RobotSimulatorClientAPI_NoDirect sim;
	sim.setInternalData(&data);


	sim.resetSimulation();
	sim.setGravity( btVector3(0 , 0 ,-9.8));
	
	int robotId = sim.loadURDF("model/indy7.urdf");  
	Indy7 indy7(&sim,robotId);
	
	double t = 0;
	double dt= FIXED_TIMESTEP;
	JVec MAX_TORQUES;
	MAX_TORQUES<<431.97,431.97,197.23,79.79,79.79,79.79;
	JVec q_des=JVec::Zero();
	JVec dq_des=JVec::Zero();	
	JVec eint = JVec::Zero();
	while(1){
		JVec q= indy7.getQ( &sim);
		JVec dq= indy7.getQdot( &sim);		
		JVec e = q_des-q;
		eint = eint+ e*dt;
		JVec gravTorq = control.Gravity( q);
		JVec clacTorq = control.ComputedTorqueControl( q, dq, q_des, dq_des); // calcTorque
		JVec clacPIDTorq = control.ComputedTorquePIDControl( q, dq, q_des, dq_des,eint); // calcTorque
		JVec clacHinfTorq=control.HinfControl(  q, dq, q_des, dq_des,eint);

		static int print_count = 0;
		if(++print_count>100){
			cout<<"e   :"<<e.transpose()<<endl;
			cout<<"eint:"<<eint.transpose()<<endl;
			print_count = 0;
		}
		indy7.setTorques(&sim,  clacHinfTorq , MAX_TORQUES);
		sim.stepSimulation();
		b3Clock::usleep(1000. * 1000. * FIXED_TIMESTEP);
		t = t+FIXED_TIMESTEP;	
	}

	return -1;
    
}
