#include "LR_Trajectory.h"
// #include <ruckig/ruckig.hpp>
// using namespace ruckig;
LR_Trajectory::LR_Trajectory(){


}
LR_Trajectory::~LR_Trajectory(){

}


void LR_Trajectory::WayPointTaskTrajectory(std::vector<SE3> way_points, std::vector<double> delays, double now, SE3 &T_des, Vector6d &V_des, Vector6d &V_dot_des)
{
    std::vector<double> time_list;
    time_list.push_back(0);
    double prev_time = 0;
    int idx = 0;
    int N = way_points.size();
    int N_delay = delays.size();
    time_list.resize(N_delay + 1);
    double sum_delays = 0;
    for (int i = 0; i < N_delay; i++)
    {
        sum_delays += delays.at(i);

        time_list.at(i + 1) = sum_delays;
        if (now >= time_list.at(i) && now < time_list.at(i + 1))
        {
            idx = i;
        }
    }
    if (now >= sum_delays)
    {
        idx = N_delay - 1;
    }

    double Tf = time_list.at(idx + 1) - time_list.at(idx);
    double now_ = now - time_list.at(idx);
    lr::LieScrewTrajectory(way_points.at(idx), way_points.at(idx + 1), Vector6d::Zero(), Vector6d::Zero(), Vector6d::Zero(), Vector6d::Zero(), Tf, now_, T_des, V_des, V_dot_des);
}

void LR_Trajectory::WayPointJointTrajectory(std::vector<JVec> way_points, std::vector<double> delays, double now, JVec &q_des, JVec &q_dot_des, JVec &q_ddot_des)
{
    std::vector<double> time_list;
    time_list.push_back(0);
    double prev_time = 0;
    int idx = 0;
    int N = way_points.size();
    int N_delay = delays.size();
    time_list.resize(N_delay + 1);
    double sum_delays = 0;
    for (int i = 0; i < N_delay; i++)
    {
        sum_delays += delays.at(i);

        time_list.at(i + 1) = sum_delays;
        if (now >= time_list.at(i) && now < time_list.at(i + 1))
        {
            idx = i;
        }
    }
    if (now >= time_list.at(time_list.size() - 1))
    {
        idx = N_delay - 1;
    }
    lr::JointTrajectory(way_points.at(idx), way_points.at(idx + 1), time_list.at(idx + 1) - time_list.at(idx), now - time_list.at(idx), 0, q_des, q_dot_des, q_ddot_des);
}


// void LR_Trajectory::LieScrewScurveTrajectory(const SE3 X0,const SE3 XT,const Vector6d V0,const Vector6d VT,const Vector6d dV0,const Vector6d dVT,Vector6d dlambda_max, Vector6d ddlambda_max,Vector6d dddlambda_max, double dt,std::vector<SE3>& T_des_list,std::vector<Vector6d>& V_des_list,std::vector<Vector6d>& V_des_dot_list,double& max_tt){
// 	Vector6d lambda_0,lambda_T,dlambda_0,dlambda_T,ddlambda_0,ddlambda_T,lambda_t,dlambda_t,ddlambda_t;
// 	lambda_0 = Vector6d::Zero();
// 	lambda_T = se3ToVec(MatrixLog6(TransInv(X0)*XT));
// 	dlambda_0 = V0;
// 	dlambda_T = dlog6(-lambda_T)*VT;
// 	ddlambda_0 = dV0;
// 	ddlambda_T = dlog6(-lambda_T)*dVT +ddlog6(-lambda_T,-dlambda_T)*VT;
// 	lambda_t=dlambda_t=ddlambda_t= Vector6d::Zero();

//     Ruckig<6> otg(dt);  // control cycle
//     InputParameter<6> input;
//     OutputParameter<6> output;
//     input.current_position = {lambda_0(0),lambda_0(1),lambda_0(2),lambda_0(3),lambda_0(4),lambda_0(5)};
//     input.current_velocity  = {dlambda_0(0),dlambda_0(1),dlambda_0(2),dlambda_0(3),dlambda_0(4),dlambda_0(5)};
//     input.current_acceleration  = {ddlambda_0(0),ddlambda_0(1),ddlambda_0(2),ddlambda_0(3),ddlambda_0(4),ddlambda_0(5)};

//     input.target_position = {lambda_T(0),lambda_T(1),lambda_T(2),lambda_T(3),lambda_T(4),lambda_T(5)};
//     input.target_velocity  = {dlambda_T(0),dlambda_T(1),dlambda_T(2),dlambda_T(3),dlambda_T(4),dlambda_T(5)};
//     input.target_acceleration = {ddlambda_T(0),ddlambda_T(1),ddlambda_T(2),ddlambda_T(3),ddlambda_T(4),ddlambda_T(5)};

//     input.max_velocity  = {dlambda_max(0),dlambda_max(1),dlambda_max(2),dlambda_max(3),dlambda_max(4),dlambda_max(5)};
//     input.max_acceleration  = {ddlambda_max(0),ddlambda_max(1),ddlambda_max(2),ddlambda_max(3),ddlambda_max(4),ddlambda_max(5)};
//     input.max_jerk  = {dddlambda_max(0),dddlambda_max(1),dddlambda_max(2),dddlambda_max(3),dddlambda_max(4),dddlambda_max(5)};
    
//     while (otg.update(input, output) == Result::Working) {
//         // std::cout << output.time << " | " << join(output.new_position) << std::endl;
//          lambda_t <<output.new_position[0],output.new_position[1],output.new_position[2],output.new_position[3],output.new_position[4],output.new_position[5];
//          dlambda_t<<output.new_velocity[0],output.new_velocity[1],output.new_velocity[2],output.new_velocity[3],output.new_velocity[4],output.new_velocity[5];
//          ddlambda_t<<output.new_acceleration[0],output.new_acceleration[1],output.new_acceleration[2],output.new_acceleration[3],output.new_acceleration[4],output.new_acceleration[5];
// 	    Vector6d V_des = dexp6(-lambda_t)*dlambda_t;
// 	    Vector6d  V_des_dot = dexp6(-lambda_t)*ddlambda_t+ddexp6(-lambda_t,-dlambda_t)*dlambda_t;
// 	    SE3 T_des = X0*MatrixExp6(VecTose3(lambda_t));         
//          T_des_list.push_back(T_des);
//          V_des_list.push_back(V_des);
//          V_des_dot_list.push_back(V_des_dot);
//          max_tt = output.time;
//          output.pass_to_input(input);
//     }
    

// }	

// void LR_Trajectory::JointScurveTrajectory(const JVec q0, const JVec qT,const JVec q_dot_0,const JVec q_dot_T,const JVec q_ddot_0,const JVec q_ddot_T,const JVec q_dot_max,const JVec q_ddot_max,const JVec q_dddot_max, double dt,std::vector<JVec>& q_des_list, std::vector<JVec>& q_dot_des_list, std::vector<JVec>& q_ddot_des_list) {
//     Ruckig<JOINTNUM> otg(dt);  // control cycle
//     InputParameter<JOINTNUM> input;
//     OutputParameter<JOINTNUM> output;
//     input.current_position = {q0(0),q0(1),q0(2),q0(3),q0(4),q0(5)};
//     input.current_velocity  = {q_dot_0(0),q_dot_0(1),q_dot_0(2),q_dot_0(3),q_dot_0(4),q_dot_0(5)};
//     input.current_acceleration  = {q_ddot_0(0),q_ddot_0(1),q_ddot_0(2),q_ddot_0(3),q_ddot_0(4),q_ddot_0(5)};

//     input.target_position = {qT(0),qT(1),qT(2),qT(3),qT(4),qT(5)};
//     input.target_velocity  = {q_dot_T(0),q_dot_T(1),q_dot_T(2),q_dot_T(3),q_dot_T(4),q_dot_T(5)};
//     input.target_acceleration  = {q_ddot_T(0),q_ddot_T(1),q_ddot_T(2),q_ddot_T(3),q_ddot_T(4),q_ddot_T(5)};

//     input.max_velocity  = {q_dot_max(0),q_dot_max(1),q_dot_max(2),q_dot_max(3),q_dot_max(4),q_dot_max(5)};
//     input.max_acceleration  = {q_ddot_max(0),q_ddot_max(1),q_ddot_max(2),q_ddot_max(3),q_ddot_max(4),q_ddot_max(5)};
//     input.max_jerk  = {q_dddot_max(0),q_dddot_max(1),q_dddot_max(2),q_dddot_max(3),q_dddot_max(4),q_dddot_max(5)};
    
//     while (otg.update(input, output) == Result::Working) {
//         JVec q,q_dot,q_ddot;
//         for(int i =0;i<JOINTNUM;i++){
//             q(i) = output.new_position[i];
//             q_dot(i) = output.new_velocity[i];
//             q_ddot(i) = output.new_acceleration[i];
//         }
//          q_des_list.push_back(q);
//          q_dot_des_list.push_back(q_dot);
//          q_ddot_des_list.push_back(q_ddot);
//          output.pass_to_input(input);
//     }
        
// }