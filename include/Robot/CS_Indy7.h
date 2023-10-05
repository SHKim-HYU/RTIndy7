#ifndef CS_INDY7_H
#define CS_INDY7_H

#include "iostream"
#include "json_loader.h"
// #include "modern_robotics.h"
#include <Eigen/Dense>
// #include <casadi/casadi.hpp>
#include <dlfcn.h>
#include "PropertyDefinition.h"

typedef long long int casadi_int;
typedef int (*eval_t)(const double**, double**, casadi_int*, double*, int);

using namespace Eigen;
using namespace std;

class CS_Indy7 {
public:
	CS_Indy7();
	
	// ~CS_Indy7(){
	// 	// Free the handle
    // 	dlclose(fd_handle);
    // 	dlclose(M_handle);
    // 	dlclose(Minv_handle);
    // 	dlclose(C_handle);
    // 	dlclose(G_handle);
    // 	dlclose(J_s_handle);
    // 	dlclose(J_b_handle);
    // 	dlclose(FK_handle);
	// };

	// JsonLoader loader_;

	void CSSetup(const string& _modelPath);
	void updateRobot(JVec _q, JVec _dq);

	JVec computeFD(JVec _q, JVec _dq, JVec _tau);

	MassMat computeM(JVec _q);
	MassMat computeMinv(JVec _q);
	MassMat computeC(JVec _q, JVec _dq);
	JVec computeG(JVec _q);

	SE3 computeFK(JVec _q);

	Jacobian computeJ_b(JVec _q);
	Jacobian computeJ_s(JVec _q);

	MassMat getM();
	MassMat getMinv();
	MassMat getC();
	JVec getG();

	SE3 getFK();

	Jacobian getJ_b();
	Jacobian getJ_s();

	JVec ComputedTorqueControl( JVec q,JVec dq,JVec q_des,JVec dq_des);
    void saturationMaxTorque(JVec &torque, JVec MAX_TORQUES);
    
    JVec HinfControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des,JVec eint);

private:
	JVec q, dq, ddq;
	JVec tau, ddq_res;
	MassMat M, Minv, C;
	JVec G;

	SE3 T_ee;
	Jacobian J_b, J_s;

private:
	bool isUpdated = false;
	string robotModel;
	int n_dof;

	void* FD_handle;
	void* M_handle;
	void* Minv_handle;
	void* C_handle;
	void* G_handle;
	void* J_s_handle;
	void* J_b_handle;
	void* FK_handle;
	
	eval_t FD_eval;
	eval_t M_eval;
	eval_t Minv_eval;
	eval_t C_eval;
	eval_t G_eval;
	eval_t J_s_eval;
	eval_t J_b_eval;
	eval_t FK_eval;

	// casadi::Function fd_cs, M_cs, Minv_cs, C_cs, G_cs, J_s_cs, J_b_cs, FK_cs;

    MassMat Kp;
    MassMat Kv;
    MassMat Ki;

    MassMat Hinf_Kp;
    MassMat Hinf_Kv;
    MassMat Hinf_Ki;
    MassMat Hinf_K_gamma;
};
#endif // CS_INDY7_H