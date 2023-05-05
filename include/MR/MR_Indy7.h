#ifndef MR_INDY7_H
#define MR_INDY7_H
#include "include/NRMKSDK/json/json/json.h"
#include "iostream"
#include "modern_robotics.h"

#pragma comment(lib, "jsoncpp.lib")
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
using namespace mr;
class MR_Indy7 {
public:
    MR_Indy7();  // Constructor
    mr::ScrewList Slist;
    mr::ScrewList Blist;
    mr::SE3 M;

	vector<mr::Matrix6d> Glist;	
	vector<mr::SE3> Mlist;	
    mr::JVec q;
    mr::JVec dq;
    mr::JVec ddq;

    mr::JVec q_des;
    mr::JVec dq_des;
    mr::JVec ddq_des;
    

    mr::Vector3d g;
    mr::JVec  torq;

    mr::Matrix6d Kp;
    mr::Matrix6d Kv;
    mr::Matrix6d Ki;

    void MRSetup();
    JVec Gravity( JVec q);
    JVec ComputedTorqueControl( JVec q,JVec dq,JVec q_des,JVec dq_des);
    void saturationMaxTorque(JVec &torque, JVec MAX_TORQUES);
};

#endif // MR_INDY7_H
