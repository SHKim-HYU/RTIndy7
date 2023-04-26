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

	void SetPIDGain(float _Kp, float _Kd, float _Ki, int _JointNum);
	void PDController_gravity(float *q, float *q_dot, float *dq, float *dq_dot, float *toq, Jointf &g_mat);
	void PD_Gravity(float * q, float * q_dot, float *dq_, float *dq_dot, float * toq);
	void Gravity(float * q, float * q_dot, float * toq);
	void PDController(Jointf &q, Jointf &q_dot, float *dq, float *dq_dot, float *toq);
//  void Impedance(float *_q_dot, Matrix<float, 6, 1> & _x,Matrix<float, 6, 1> & _x_dot, Matrix<float, 6, 1> & _dx, Matrix<float, 6, 1> & _dx_dot, Matrix<float, 6, 1> & _dx_ddot, float * toq);
//	void Impedance(float *q, float *q_dot, float *q_ddot, float *dq, float *dq_dot, float *dq_ddot, float *toq, Matrixf &m_mat, Matrixf &c_mat, Jointf &g_mat);
	//void Impedance(Jointf &q, Jointf &q_dot, Jointf &q_ddot, float *dq, float *dq_dot, float *dq_ddot, float *toq, Matrixf &m_mat, Jointf &g_mat);
	//void Impedance(Jointf &q, Jointf &q_dot, Jointf &q_ddot, float *dq, float *dq_dot, float *dq_ddot, float *toq, Matrixf &m_mat, Jointf &g_mat, Matrixf &c_mat);
	void Inverse_Dynamics_Control(float *q_, float *q_dot, float *dq_, float *dq_dot, float *dq_ddot, float * toq);
    void ComputedTorque(float *q_, float *q_dot, float *dq_, float *dq_dot, float *dq_ddot, float * toq);
    void VSD(float *_q, float *_qdot, Vector3f &xd, float *toq, float gt, int flag);
    void CLIKController( float *_q, float *_qdot, float *_dq, float *_dqdot, const VectorXf *_dx, const VectorXf *_dxdot, const VectorXf &_dqdotNull, float *p_Toq, float &_dt );
    void CLIKController_2nd(float *_q, float *_qdot, Matrix<float,6,1>& dq, Matrix<float,6,1>& dq_dot, Matrix<float,6,1>& dq_ddot);

    void FrictionIdentify(float *_q, float *_qdot, float *dq_, float *dq_dot, float *dq_ddot, float *toq, float _gt);
    float FrictionCompensation(float _qdot, int _JointNum);
	void TorqueOutput(float *p_toq, int maxtoq, int *p_dir);
	//void TorqueOutput(float *p_toq , int maxtoq);
	Jointf return_u0(void);

	//void IKAccel(state *des, state *act);

	int int_flag=0;
    robot *pManipulator;
private:
	Matrix<float,ROBOT_DOF,1> Kp;
	Matrix<float,ROBOT_DOF,1> Kd;
	Matrix<float,ROBOT_DOF,1> Ki;

	Matrix<float,ROBOT_DOF,1> Damp;
	Matrix<float,ROBOT_DOF,1> Stiff;

	Matrix<float,ROBOT_DOF,1> e;
	Matrix<float,ROBOT_DOF,1> e_dev;
	Matrix<float,ROBOT_DOF,1> e_int;
	Matrix<float,ROBOT_DOF,1> e_old;

	VectorXf q, qdot, dq, dqdot, dqddot;
	Vector3f x, x_dot, xd_buff;
    VectorXf eTask, edotTask;
    MatrixXf edotTmp;

    VectorXf ToqOut;
    VectorXf KpTask, KdTask;
    VectorXf KiTask;
    MatrixXf G,Gx,M,C;
    MatrixXf _a_jaco, _pinv_jaco,_jaco_dot;

    LinJaco l_Jaco, l_Jaco_dot;
    PinvLJaco DPI_l_jaco;

    MatrixXf LinearJacobian;

    VectorXf mvC0_;
    float mvK_, mvKsi_, mvZeta0_, mvZeta1_;
    VectorXf mK_;

    Jointf u0;
    VectorXf ax;
	int m_Jnum;
	int q_flag;
	int t_flag;
	float init_time, init_time_vsd;


    Matrix<float,ROBOT_DOF,1> f_a, f_b, f_c, f_d, f_e, f_f;


    float m_KpBase, m_KdBase, m_KiBase;

};

} /* namespace HYUCtrl */

#endif /* CONTROLLER_H_ */
