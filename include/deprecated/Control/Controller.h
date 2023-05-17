/*
 * Controller.h
 *
 *  Created on: 2019. 5. 15.
 *      Author: Administrator
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "../KDL/PropertyDefinition.h"
#include "../KDL/SerialRobot.h"

#define KpBase 10
#define KdBase 0.01
#define KiBase 30


/**
 * @brief
 * @details
 * @author Junho Park
 * @date   20119-05-16
 * @version 1.0.0
 *
 */
using namespace Eigen;
namespace HYUControl {

class Controller : public HYUMotionBase::LieOperator//: public robot
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Controller();
	Controller(int JointNum);
	Controller(robot *pManipulator, int JointNum);
	virtual ~Controller(){};

	//void ClearError(void);

	void SetPIDGain(double _Kp, double _Kd, double _Ki, int _JointNum);
	void PDController_gravity(double *q, double *q_dot, double *dq, double *dq_dot, double *toq, Jointd &g_mat);
	void PD_Gravity(double * q, double * q_dot, double *dq_, double *dq_dot, double * toq);
	void Gravity(double * q, double * q_dot, double * toq);
	void PDController(Jointd &q, Jointd &q_dot, double *dq, double *dq_dot, double *toq);
//  void Impedance(double *_q_dot, Matrix<double, 6, 1> & _x,Matrix<double, 6, 1> & _x_dot, Matrix<double, 6, 1> & _dx, Matrix<double, 6, 1> & _dx_dot, Matrix<double, 6, 1> & _dx_ddot, double * toq);
//	void Impedance(double *q, double *q_dot, double *q_ddot, double *dq, double *dq_dot, double *dq_ddot, double *toq, Matrixd &m_mat, Matrixd &c_mat, Jointd &g_mat);
	//void Impedance(Jointd &q, Jointd &q_dot, Jointd &q_ddot, double *dq, double *dq_dot, double *dq_ddot, double *toq, Matrixd &m_mat, Jointd &g_mat);
	//void Impedance(Jointd &q, Jointd &q_dot, Jointd &q_ddot, double *dq, double *dq_dot, double *dq_ddot, double *toq, Matrixd &m_mat, Jointd &g_mat, Matrixd &c_mat);
	void Inverse_Dynamics_Control(double *q_, double *q_dot, double *dq_, double *dq_dot, double *dq_ddot, double * toq);
    void ComputedTorque(double *q_, double *q_dot, double *dq_, double *dq_dot, double *dq_ddot, double * toq);
    void VSD(double *_q, double *_qdot, Vector3d &xd, double *toq, double gt, int flag);
    void CLIKController( double *_q, double *_qdot, double *_dq, double *_dqdot, const VectorXd *_dx, const VectorXd *_dxdot, const VectorXd &_dqdotNull, double *p_Toq, double &_dt );
    void CLIKController_2nd(double *_q, double *_qdot, Matrix<double,6,1>& dq, Matrix<double,6,1>& dq_dot, Matrix<double,6,1>& dq_ddot);

    void FrictionIdentify(double *_q, double *_qdot, double *dq_, double *dq_dot, double *dq_ddot, double *toq, double _gt);
    double FrictionCompensation(double _qdot, int _JointNum);
	void TorqueOutput(double *p_toq, int maxtoq, int *p_dir);
	//void TorqueOutput(double *p_toq , int maxtoq);
	Jointd return_u0(void);

	//void IKAccel(state *des, state *act);

	int int_flag=0;
    robot *pManipulator;

	MatrixXd Slist;
	MatrixXd Blist;
	vector<MatrixXd> Mlist;
	vector<MatrixXd> Glist;
	MatrixXd MR_M;	
private:
	Matrix<double,ROBOT_DOF,1> Kp;
	Matrix<double,ROBOT_DOF,1> Kd;
	Matrix<double,ROBOT_DOF,1> Ki;

	Matrix<double,ROBOT_DOF,1> Damp;
	Matrix<double,ROBOT_DOF,1> Stiff;

	Matrix<double,ROBOT_DOF,1> e;
	Matrix<double,ROBOT_DOF,1> e_dev;
	Matrix<double,ROBOT_DOF,1> e_int;
	Matrix<double,ROBOT_DOF,1> e_old;

	VectorXd q, qdot, dq, dqdot, dqddot;
	Vector3d x, x_dot, xd_buff;
    VectorXd eTask, edotTask;
    MatrixXd edotTmp;

    VectorXd ToqOut;
    VectorXd KpTask, KdTask;
    VectorXd KiTask;
    MatrixXd G,Gx,M,C;
    MatrixXd _a_jaco, _pinv_jaco,_jaco_dot;
	

    LinJaco l_Jaco, l_Jaco_dot;
    PinvLJaco DPI_l_jaco;

    MatrixXd LinearJacobian;

    VectorXd mvC0_;
    double mvK_, mvKsi_, mvZeta0_, mvZeta1_;
    VectorXd mK_;

    Jointd u0;
    VectorXd ax;
	int m_Jnum;
	int q_flag;
	int t_flag;
	double init_time, init_time_vsd;

    Matrix<double,ROBOT_DOF,1> f_a, f_b, f_c, f_d, f_e, f_f;


    double m_KpBase, m_KdBase, m_KiBase;



};

} /* namespace HYUCtrl */

#endif /* CONTROLLER_H_ */
