
#ifndef LR_TRAJECTORY_H
#define LR_TRAJECTORY_H
#include "liegroup_robotics.h"
#include "../type.h"
using namespace lr;
class LR_Trajectory {
    public:
        LR_Trajectory();  // Constructor
        void WayPointJointTrajectory(std::vector<JVec> way_points, std::vector<double> delays, double now, JVec& q_des,JVec& q_dot_des,JVec& q_ddot_des);
        void WayPointTaskTrajectory(std::vector<SE3> way_points, std::vector<double> delays, double now, SE3& T_des,Vector6d& V_des,Vector6d& V_dot_des);
        void LieScrewScurveTrajectory(const SE3 X0,const SE3 XT,const Vector6d V0,const Vector6d VT,const Vector6d dV0,const Vector6d dVT,Vector6d dlambda_max, Vector6d ddlambda_max,Vector6d dddlambda_max, double dt,std::vector<SE3>& T_des_list,std::vector<Vector6d>& V_des_list,std::vector<Vector6d>& V_des_dot_list,double& max_tt);
        void JointScurveTrajectory(const JVec q0, const JVec qT,const JVec q_dot_0,const JVec q_dot_T,const JVec q_ddot_0,const JVec q_ddot_T,const JVec q_dot_max,const JVec q_ddot_max,const JVec q_dddot_max, double dt,std::vector<JVec>& q_des_list, std::vector<JVec>& q_dot_des_list, std::vector<JVec>& q_ddot_des_list) ;
        ~LR_Trajectory(); 

};
#endif // LR_TRAJECTORY_H