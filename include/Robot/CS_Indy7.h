#ifndef CS_INDY7_H
#define CS_INDY7_H

#include "iostream"
#include "json_loader.h"
// #include "modern_robotics.h"
#include <Eigen/Dense>
// #include <casadi/casadi.hpp>
#include <dlfcn.h>
#include "liegroup_robotics.h"
#include "PropertyDefinition.h"

typedef long long int casadi_int;
typedef int (*eval_t)(const double**, double**, casadi_int*, double*, int);

using namespace Eigen;
using namespace std;
using namespace lr;

class CS_Indy7 {
public:
	CS_Indy7();
	

	void CSSetup(const string& _modelPath, double _period);
	void setPIDgain(JVec _Kp, JVec _Kd, JVec _Ki);
	void setHinfgain(JVec _Hinf_Kp, JVec _Hinf_Kd, JVec _Hinf_Ki, JVec _Hinf_K_gamma);
	void setNRICgain(JVec _NRIC_Kp, JVec _NRIC_Ki, JVec _NRIC_K, JVec _NRIC_gamma);
	void setTaskgain(Twist _Kp, Twist _Kv, JVec _K);
	void setTaskImpedancegain(Matrix6d _Kp, Matrix6d _Kv, Matrix6d _Kgamma);

	void updateRobot(JVec _q, JVec _dq, JVec _ddq);

	JVec computeFD(JVec _q, JVec _dq, JVec _tau);
	void computeRK45(JVec _q, JVec _dq, JVec _tau, JVec &_q_nom, JVec &_dq_nom, JVec &_ddq_nom);

	Twist computeF_Tool(Twist _dx, Twist _ddx);
	Twist computeF_Threshold(Twist _F);

	MassMat computeM(JVec _q);
	MassMat computeMinv(JVec _q);
	MassMat computeC(JVec _q, JVec _dq);
	JVec computeG(JVec _q);

	SE3 computeFK(JVec _q);

	Jacobian computeJ_b(JVec _q);
	Jacobian computeJ_s(JVec _q);
	Jacobian computeJdot_b(JVec _q, JVec _dq);
	Jacobian computeJdot_s(JVec _q, JVec _dq);

	double computeManipulability(JVec _q);

	MassMat getM();
	MassMat getMinv();
	MassMat getC();
	JVec getG();

	SE3 getFK();
	SO3 getRMat();

	Jacobian getJ_b();
	Jacobian getJ_s();
	Jacobian getJdot_b();
	Jacobian getJdot_s();
	Twist getBodyTwist();
	double getManipulability();

	JVec FrictionEstimation(JVec dq);

	// Joint Space
	JVec ComputedTorqueControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des);
	JVec ComputedTorqueControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des, JVec _tau_ext);
	JVec PassivityInverseDynamicControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des);
	JVec PassivityInverseDynamicControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des, JVec _tau_ext);
	JVec HinfControl(JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des);
	JVec HinfControl(JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des, JVec _tau_ext);
	
	// Task Space
    JVec TaskInverseDynamicsControl(JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des);
	JVec TaskJTbasedInverseDynamicsControl(JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des);
	JVec TaskPassivityInverseDynamicsControl(JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des);
	JVec TaskImpedanceControl(JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des, Twist F_des, Twist F_ext);
	JVec TaskJTbasedImpedanceControl(JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des, Twist F_des, Twist F_ext);
	JVec TaskPassivityImpedanceControl(JVec q_dot, SE3 T_des, Twist V_des, Twist V_dot_des, Twist F_des, Twist F_ext);
	JVec TaskComplianceImpedanceControl(SE3 T_des, Twist V_des, Twist V_dot_des, Twist F_des, Twist F_ext);
	JVec TaskStablePD(SE3 T_des, Twist V_des, Twist V_dot_des);
	JVec TaskStablePDImpedance(SE3 T_des, Twist V_des, Twist V_dot_des, Twist F_des, Twist F_ext);

	void resetTaskAdmittance();
	void TaskAdmittance(SE3 T_des, Twist V_des, Twist V_dot_des, SE3 &T_adm, Twist &V_adm, Twist &V_dot_adm, Twist F_des, Twist F_ext);

    JVec NRIC(JVec q_r, JVec dq_r, JVec q_n, JVec dq_n);
	
	double computeAlpha(JVec edot, JVec tau_c, JVec tau_ext);
	void saturationMaxTorque(JVec &torque, JVec MAX_TORQUES);

private:
	JVec q, dq, ddq;
	JVec tau, tau_bd, tau_ext, ddq_res;
	JVec e, eint;
	MassMat M, Minv, C;
	JVec G;

	SE3 T_M;
	SE3 T_ee;
	SO3 R_ee;
	ScrewList Slist, Blist;
	Jacobian J_b, J_s;
	Jacobian dJ_b, dJ_s;

	SE3 T_ref;
	Twist V_ref, V_dot_ref;

	Twist V_b, V_s;
	Twist V_dot;
	Twist lambda, lambda_dot, lambda_int;
	Twist lambda_ref;
	
	Twist F_eff, F_eff_int;
	Twist gamma, gamma_int;

	Matrix6d A_tool, B_tool;
	Vector3d r_floor;
    Matrix3d r_ceil;
	Vector6d G_tool;
	Vector6d G_FT;
	Vector6d F_FT;

private:
	bool isUpdated = false;
	string robotModel;
	int n_dof;
	double period;

	void* FD_handle;
	void* M_handle;
	void* Minv_handle;
	void* C_handle;
	void* G_handle;
	void* J_s_handle;
	void* J_b_handle;
	void* dJ_s_handle;
	void* dJ_b_handle;
	void* FK_handle;
	
	eval_t FD_eval;
	eval_t M_eval;
	eval_t Minv_eval;
	eval_t C_eval;
	eval_t G_eval;
	eval_t J_s_eval;
	eval_t J_b_eval;
	eval_t dJ_s_eval;
	eval_t dJ_b_eval;
	eval_t FK_eval;

	// casadi::Function fd_cs, M_cs, Minv_cs, C_cs, G_cs, J_s_cs, J_b_cs, FK_cs;

	Matrix6d Task_Kp;
	Matrix6d Task_Kv;
	Matrix6d Task_Ki;
	JMat Task_K;

	Matrix6d Task_Kp_imp;
	Matrix6d Task_Kv_imp;
	Matrix6d Task_Kgama_imp;
	Matrix6d Task_K_imp;

	Matrix6d A_, D_, K_;
    Matrix6d A_lambda, D_lambda, K_lambda;

    JMat Kp;
    JMat Kv;
    JMat K;

    JMat Hinf_Kp;
    JMat Hinf_Kv;
    JMat Hinf_Ki;
    JMat Hinf_K_gamma;

	JMat NRIC_Kp;
    JMat NRIC_Ki;
    JMat NRIC_K_gamma;

	// Friction model parameters
	JVec Fc;
	JVec Fv1;
	JVec Fv2;

	// double alpha = 0.0;
	double manipulability;

};
#endif // CS_INDY7_H