#ifndef BULLET_INDY7_SETUP_H
#define BULLET_INDY7_SETUP_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "../MR/modern_robotics.h"
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <json/json.h>
#include <iomanip>      // std::setprecision
#pragma comment(lib, "jsoncpp.lib")

using namespace std;
using namespace Eigen;
using namespace mr;
class Bullet_Indy7
{
	int robotId;
	int actuated_joint_num;
	int eef_num;
	vector<int> actuated_joint_id;
	vector<string> actuated_joint_name;
		
public:

	Bullet_Indy7(class b3RobotSimulatorClientAPI_NoDirect* sim,int robotId);
	void set_torque(class b3RobotSimulatorClientAPI_NoDirect* sim,JVec  torques ,JVec  max_torques );
	JVec get_q(class b3RobotSimulatorClientAPI_NoDirect* sim);	
	JVec get_qdot(class b3RobotSimulatorClientAPI_NoDirect* sim);	
	SE3 get_eef_pose(class b3RobotSimulatorClientAPI_NoDirect* sim);	
	void reset_q(class b3RobotSimulatorClientAPI_NoDirect* sim,JVec q);
	Vector6d get_FT(class b3RobotSimulatorClientAPI_NoDirect* sim);
	void apply_ext_FT(class b3RobotSimulatorClientAPI_NoDirect* sim,JVec FT);
	int get_actuated_joint_num(){
		return this->actuated_joint_num;
	};
	
	virtual ~Bullet_Indy7();

};
#endif  //BULLET_INDY7_SETUP_H