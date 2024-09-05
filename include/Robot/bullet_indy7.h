#ifndef BULLET_INDY7_SETUP_H
#define BULLET_INDY7_SETUP_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "liegroup_robotics.h"
#include "PropertyDefinition.h"
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>      // std::setprecision

using namespace std;
using namespace Eigen;
using namespace lr;
class Bullet_Indy7
{
	int robotId;
	int actuated_joint_num;
	int eef_num;
	vector<int> actuated_joint_id;
	vector<string> actuated_joint_name;
		
public:

	Bullet_Indy7(class b3RobotSimulatorClientAPI* sim, int robotId);
	void set_torque(JVec  torques ,JVec  max_torques );
	JVec get_q();	
	JVec get_qdot();	
	SE3 get_eef_pose();	
	void reset_q(JVec q);
	Vector6d get_FT();
	void apply_ext_FT(JVec FT);
	int get_actuated_joint_num(){
		return this->actuated_joint_num;
	};
	
	virtual ~Bullet_Indy7();
private:
	class b3RobotSimulatorClientAPI* sim;
};
#endif  //BULLET_INDY7_SETUP_H