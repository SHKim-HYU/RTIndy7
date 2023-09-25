#ifndef CS_INDY7_H
#define CS_INDY7_H

#include "iostream"
#include "json_loader.h"
#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <dlfcn.h>

typedef long long int casadi_int;
typedef int (*eval_t)(const double**, double**, casadi_int*, double*, int);

using namespace Eigen;
using namespace std;
using namespace mr;

class CS_Indy7 {
public:
	CS_Indy7(const string& _modelPath);
	~CS_Indy7(){};

	JsonLoader loader_;

	void load_casadi_function();
	void updateRobot(JVec _q, JVec _dq);

	MassMat M(JVec _q);
	MassMat Minv(JVec _q);
	MassMat C(JVec _q, JVec _dq);
	JVec G(JVec _q);

	SE3 FK(JVec _q);

	Jacobian J_b(JVec _q);
	Jacobian J_s(JVec _q);

	JVec ComputedTorqueControl( JVec q,JVec dq,JVec q_des,JVec dq_des);
    void saturationMaxTorque(JVec &torque, JVec MAX_TORQUES);
    
    JVec HinfControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des,JVec eint);

private:
	JVec q, dq, ddq;
	JVec tau;
	MassMat M, Minv, C;
	JVec G;

	SE3 T_ee;
	Jacobian J_b, J_s;

private:
	bool isUpdated = false;
	string robotModel;
	int n_dof;
	void* fd_handle, M_handle, Minv_handle, C_handle, G_handle, J_s_handle, J_b_handle, FK_handle;
	eval_t fd_eval, M_eval, Minv_eval C_eval, G_eval, J_s_eval, J_b_eval, FK_eval;

	casadi::Function fd_cs, M_cs, Minv_cs, C_cs, G_cs, J_s_cs, J_b_cs, FK_cs;

    MassMat Kp;
    MassMat Kv;
    MassMat Ki;

    MassMat Hinf_Kp;
    MassMat Hinf_Kv;
    MassMat Hinf_Ki;
    MassMat Hinf_K_gamma;
}
#endif // CS_INDY7_H