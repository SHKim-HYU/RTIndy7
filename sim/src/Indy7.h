#ifndef INDY7_SETUP_H
#define INDY7_SETUP_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include "../../include/MR/modern_robotics.h"
#include <vector>
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;
using namespace mr;
class Indy7
{
	int robotId;
	int actuated_joint_num;
	int eef_num;
	vector<int> actuated_joint_id;
	vector<string> actuated_joint_name;
	ScrewList Slist;
	ScrewList Blist;	
	vector<Matrix6d> Glist;	
	vector<SE3> Mlist;			
	SE3 M;				
	
public:
	Indy7(class b3RobotSimulatorClientAPI_NoDirect* sim,int robotId);
	void setTorques(class b3RobotSimulatorClientAPI_NoDirect* sim,JVec  torques ,JVec  max_torques );
	JVec getQ(class b3RobotSimulatorClientAPI_NoDirect* sim);	
	JVec getQdot(class b3RobotSimulatorClientAPI_NoDirect* sim);	
	SE3 getEEFPose(class b3RobotSimulatorClientAPI_NoDirect* sim);	

	int getActuatedJointNum(){
		return this->actuated_joint_num;
	};
	
	virtual ~Indy7();

};
#endif  //INDY7_SETUP_H
