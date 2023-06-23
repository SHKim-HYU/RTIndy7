#include "bullet_Indy7.h"


void print_vec(JVec vec,string str){
	cout<<str<<":";
	for(int i = 0;i<vec.rows()-1;i++)
		cout<<vec[i]<<",";
	cout<<vec[vec.rows()-1];
	cout<<""<<endl;
		
}
void print_mat(SE3 mat,string str){

int strlen = str.length();	
const char* sep ="-";
	for(int i = 0;i<(80-strlen)/2;i++)
		cout<<sep;
	cout<<str;
	if(strlen% 2 == 0){
		for(int i = 0;i<(80-strlen)/2;i++)
			cout<<sep;
	}else{
		for(int i = 0;i<(80-strlen)/2+1;i++)
			cout<<sep;
	}
	
	cout<<""<<endl;
		
	IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
	
	std::cout << mat.format(OctaveFmt)<<endl;
	
cout<<"--------------------------------------------------------------------------------"<<endl;
}
Bullet_Indy7::Bullet_Indy7(class b3RobotSimulatorClientAPI_NoDirect* sim,int robotId){
	this->robotId = robotId;
	int numJoints = sim->getNumJoints(this->robotId);
	int actuated_joint_num = 0;
	for (int i = 0; i < numJoints; i++)
	{
		b3JointInfo jointInfo;
		sim->getJointInfo(this->robotId, i, &jointInfo);
		if (jointInfo.m_jointName[0] && jointInfo.m_jointType!=eFixedType)
		{
			this->actuated_joint_name.push_back(jointInfo.m_jointName);
			this->actuated_joint_id.push_back(i);		
			// initialize motor
			b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
			controlArgs.m_maxTorqueValue  = 0.0;
			sim->setJointMotorControl(robotId,i,controlArgs);				
			b3RobotSimulatorJointMotorArgs controlArgs2(CONTROL_MODE_TORQUE);
			controlArgs2.m_maxTorqueValue  = 0.0;			
			sim->setJointMotorControl(robotId,i,controlArgs2);
			actuated_joint_num++;	
		}
		
	}
	this->eef_num = numJoints-1;
	this->actuated_joint_num = actuated_joint_num;
	sim->enableJointForceTorqueSensor(this->robotId,this->eef_num,1);
	
}

JVec saturate(JVec val, JVec max_val){
	JVec retVal = Map<JVec>(val.data(), val.rows(), val.cols());
	for (int i = 0; i<val.size();i++){
		retVal[i] = min(max(val[i],-max_val[i]),max_val[i]);
	}
	return retVal;
}
void Bullet_Indy7::set_torque(class b3RobotSimulatorClientAPI_NoDirect* sim,JVec  torques ,JVec  max_torques){
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_TORQUE);
	JVec saturated_torques = saturate(torques,max_torques);
	for (int i = 0; i<torques.size();i++){
		controlArgs.m_maxTorqueValue  =saturated_torques[i];
		sim->setJointMotorControl(this->robotId,this->actuated_joint_id.at(i),controlArgs);
	}	
}
JVec Bullet_Indy7::get_q(class b3RobotSimulatorClientAPI_NoDirect* sim){
	JVec q(this->actuated_joint_num);
	b3JointSensorState jointStates;
	
	int numJoints = sim->getNumJoints(this->robotId);
	
	for (int i = 0; i < this->actuated_joint_id.size(); i++)
	{
		if(sim->getJointState(this->robotId,this->actuated_joint_id.at(i), &jointStates)){
			q[i] = jointStates.m_jointPosition;
		}

	}
	
	return q;
}	
Vector6d Bullet_Indy7::get_FT(class b3RobotSimulatorClientAPI_NoDirect* sim){
	Vector6d FT,retFT;
	b3JointSensorState jointStates;
	int numJoints = sim->getNumJoints(this->robotId);
	
	if(sim->getJointState(this->robotId,this->eef_num, &jointStates)){
			for (int i = 0; i < 6; i++)
		{
			FT[i] = jointStates.m_jointForceTorque[i];
		}
	}
	retFT[0] = FT[3];
	retFT[1] = FT[4];
	retFT[2] = FT[5];
	retFT[3] = FT[0];
	retFT[4] = FT[1];
	retFT[5] = FT[2];
	return retFT;
}	
JVec Bullet_Indy7::get_qdot(class b3RobotSimulatorClientAPI_NoDirect* sim){
	JVec qdot(this->actuated_joint_num);
	b3JointSensorState jointStates;
	int numJoints = sim->getNumJoints(this->robotId);
	
	for (int i = 0; i < this->actuated_joint_id.size(); i++)
	{
		if(sim->getJointState(this->robotId,this->actuated_joint_id.at(i), &jointStates)){
			qdot[i] = jointStates.m_jointVelocity;
		}

	}

	return qdot;
}	
SE3 Bullet_Indy7::get_eef_pose(class b3RobotSimulatorClientAPI_NoDirect* sim){
	SE3 pose = SE3::Identity(4,4);
	b3LinkState linkState;
	bool computeVelocity = true;
	bool computeForwardKinematics = true;
	sim->getLinkState(this->robotId, this->eef_num, computeVelocity, computeForwardKinematics, &linkState);
	JVec pos(3,1);
	pos<< linkState.m_worldLinkFramePosition[0], linkState.m_worldLinkFramePosition[1] , linkState.m_worldLinkFramePosition[2];
	btQuaternion orn = btQuaternion(linkState.m_worldLinkFrameOrientation[0],linkState.m_worldLinkFrameOrientation[1],linkState.m_worldLinkFrameOrientation[2],linkState.m_worldLinkFrameOrientation[3]);
	btMatrix3x3 R = btMatrix3x3(orn);
	pose(0,3) = pos[0];
	pose(1,3) = pos[1];
	pose(2,3) = pos[2];		

	for(int i =0;i<3;i++){
		btVector3 r = R[i];
		for(int j =0;j<3;j++){		
			pose(i,j) = r[j];
		}
	}
	
	
	return pose;
}

void Bullet_Indy7::reset_q(class b3RobotSimulatorClientAPI_NoDirect* sim,JVec q){
	for (int i = 0; i<q.size();i++){
		sim->resetJointState(this->robotId,this->actuated_joint_id.at(i),q[i]);
	}
}
void Bullet_Indy7::apply_ext_FT(class b3RobotSimulatorClientAPI_NoDirect* sim,JVec FT){
	btVector3 force ;
	btVector3 torque ;
	torque[0] = -FT(0);
	torque[1] = -FT(1);
	torque[2] = -FT(2);
	force[0] = -FT(3);
	force[1] = -FT(4);
	force[2] = -FT(5);
	btVector3 position;
	position[0]=0;
	position[1]=0;
	position[2]=0;
	sim->applyExternalForce(this->robotId,this->eef_num,force,position,EF_LINK_FRAME);
	sim->applyExternalTorque(this->robotId,this->eef_num,torque,EF_LINK_FRAME);
}
Bullet_Indy7::~Bullet_Indy7(){
	
}
