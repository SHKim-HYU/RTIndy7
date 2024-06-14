#ifndef LR_CONTROL_H
#define LR_CONTROL_H
#include <jsoncpp/json/json.h>
#include "iostream"
#include "liegroup_robotics.h"
#include <vector>
#pragma comment(lib, "jsoncpp.lib")
#include <Eigen/Dense>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
using namespace Eigen;
using namespace std;
using namespace lr;
class LR_Control {
public:
    LR_Control();  // Constructor
    ScrewList Slist;
    ScrewList Blist;
    SE3 M;

	vector<Matrix6d> Glist;	
	vector<SE3> Mlist;	
    JVec q;
    JVec dq;
    JVec ddq;

    JVec q_des;
    JVec dq_des;
    JVec ddq_des;
    

    Vector3d g;
    JVec  torq;

    MatrixNd Kp;
    MatrixNd Kv;
    MatrixNd Ki;

    Matrix6d Task_Kp;
    Matrix6d Task_Kv;

    MatrixNd Hinf_Kp;
    MatrixNd Hinf_Kv;
    MatrixNd Hinf_Ki;
    MatrixNd Hinf_K_gamma;

    void LRSetup(const char* urdf_path);
    JVec HinfControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des,JVec eint);
    JVec HinfControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des,JVec eint,double eef_mass);
    JVec TaskHinfControl( JVec q,JVec q_dot,JVec q_ddot, SE3 T_des,Vector6d V_des,Vector6d V_dot_des,Vector6d eint);
    void WayPointJointTrajectory(std::vector<JVec> way_points, std::vector<double> delays, double now, JVec& q_des,JVec& q_dot_des,JVec& q_ddot_des);
    void WayPointTaskTrajectory(std::vector<SE3> way_points, std::vector<double> delays, double now, SE3& T_des,Vector6d& V_des,Vector6d& V_dot_des);
};

#endif // LR_CONTROL_H
